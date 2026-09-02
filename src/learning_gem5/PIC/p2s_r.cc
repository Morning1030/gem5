#include "learning_gem5/PIC/p2s.hh"
#include "learning_gem5/PIC/scheduler.hh"
#include "sim/system.hh"

#include <algorithm>
#include <cstring>
#include <memory>

#include "base/logging.hh"
#include "debug/P2S_R.hh"

#define WORDLINENUMS 512

namespace gem5
{
P2S_R::P2S_R(const P2S_RParams &params) :
    ClockedObject(params),
    instPort(params.name + ".cpu_port", this),
    DMAPort(params.name + ".dma_port", this, MemSidePort::PICPortID::DMA),
    CacheBankPort(params.name + ".cb_port", this, MemSidePort::PICPortID::CB),
    requestorId(system.getRequestorId(this, "P2S_R")),
    pendingReqPkt(nullptr),
    p2sDone(true),
    dmaReadEvent([this]{this->processDMAReadEvent();}, "dmaReadEvent"),
    loadBufferEvent([this]{this->processLoadBufferEvent();}, "loadBufferEvent"),
    bitSliceEvent([this]{this->processBitSliceEvent();}, "bitSliceEvent"),
    writeEvent([this]{this->processWriteEvent();}, "writeBankEvent")
{}
Port &
P2S_R::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port")
        return instPort;

    if (if_name == "dma_port")
        return DMAPort;

    if (if_name == "cb_port")
        return CacheBankPort;

    return ClockedObject::getPort(if_name, idx);
}

P2S_R::CPUSidePort::CPUSidePort(
    const std::string &name,
    P2S_R *owner) :
    ResponsePort(name, owner),
    owner(owner),
    blockedPacket(nullptr)
{}
bool
P2S_R::CPUSidePort::recvTimingReq(PacketPtr pkt) {
    // Just forward to the memobj.
    return owner->handleRequest(pkt);
}
void
P2S_R::CPUSidePort::sendPacket(PacketPtr pkt)
{
    // send p2s done to scheduler
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    if (sendTimingResp(pkt)) {
        owner->pendingReqPkt = nullptr;
        blockedPacket = nullptr;
        // p2sDone = true;
        DPRINTF(P2S_R, "P2S COMPLETE: send back to scheduler\n");
    }
    else blockedPacket = pkt;
}
void
P2S_R::CPUSidePort::recvRespRetry() {
    // retry to send resp to scheduler
    assert(blockedPacket != nullptr);

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
}

P2S_R::MemSidePort::MemSidePort(
    const std::string &name,
    P2S_R *owner
    PICPortID picPortID) :
    RequestPort(name, owner),
    owner(owner),
    portID(picPortID),
    blockedPacket(nullptr)
{}
bool
P2S_R::MemSidePort::recvTimingResp(PacketPtr pkt) {
    if (this->portID == PICPortID::DMA) {
        return owner->handleResponse(pkt);
    }
    else {
        // resp from cache bank: acknowledge a previously sent write
        delete pkt;
        return true;
    }
}
void
P2S_R::MemSidePort::recvReqRetry()
{
    if (this->portID == PICPortID::DMA) {
        assert(blockedPacket != nullptr);

        if (sendTimingReq(blockedPacket)) {
            blockedPacket = nullptr;
            DPRINTF(P2S_R, "DMA request accepted on retry\n");
        }
        // might fail again, wait for another req retry
    }
    // retry req from cache bank, start writeEvent again from cache write queue
    else if (this->portID == PICPortID::CB){
        if (!owner->writeEvent.scheduled()) {
            owner->schedule(owner->writeEvent, owner->clockEdge(Cycles(1)));
        }
    }
    else {
        DPRINTF(P2S_R, "Unknown port id!\n");
    }
}
bool
P2S_R::handleRequest(PacketPtr pkt) {
    // fill the packet field into data members of p2s
    // ask DMA to get data by cache controller
    // fill into regArray
    if (pendingReqPkt != nullptr || p2sDone == false) return false;

    pendingReqPkt = pkt;
    p2sDone = false;
    const P2S_R_Payload *p2s_R_Payload = pkt->getConstPtr<P2S_R_Payload>();

    // fill the packet field into data members of p2s
    dramAddr = p2s_R_Payload->dramAddr;
    base_arrayID_to_store = p2s_R_Payload->base_arrayID_to_store; // Which subarray to put the first selected bit map
    next_row_offset_bytes = p2s_R_Payload->next_row_offset_bytes;                                 // 15bits
    nRows = p2s_R_Payload->nRows;                                                 // Read how many rows
    nCols = p2s_R_Payload->nCols;                                                 // Number of columns to read, max 1024
    precision = p2s_R_Payload->precision;
    bufNum = p2s_R_Payload->bufNum;

    // initialize data memebers
    // bitSlice variables
    bit_ptr = 0;
    curEnqBlockInBufColPtr = 0;
    curBufColPtrInBlock = 0;
    curBlockColPtrGlobal = 0;
    blockNColInMem = std::min(128, nCols - curBlockColPtrGlobal);
    curBufNCols = std::min(8, blockNColInMem - curBufColPtrInBlock);

    // dma and buffer variables
    curBlockRowPtr = 0;
    curBlockColPtr = 0;
    curBlockNCols = std::min(128, nCols - curBlockColPtr);          // equal to blockNColInMem
    curBlockNRows = nRows;
    writeBufRowPtr = 0;
    readMemAddr= 0;
    writeMemAddr = 0;
    curBlockDramBaseAddrPtr = p2s_R_Payload->dramAddr;
    curRowDramAddrOffset = 0;

    get_array_relatice_offset(relative_offset_buf, bufNum);
    arrayID_offset[0] = 0;
    for (int i = 1; i < 8; i++) arrayID_offset[i] = arrayID_offset[i - 1] + relative_offset_buf[i - 1];

    schedule(dmaReadEvent, clockEdge(Cycles(1)));

    return true;
}

bool
P2S_R::handleResponse(PacketPtr pkt) {
    // fill the response to sram block
    // TODO need sender state row to deal with out of order receiving
    uint8_t *dmaData = pkt->getConstPtr<uint8_t>(); // dma send a uint64_t variable but can be expressed as 8 * 8 uint8_t
    size_t pktSize = pkt->getSize();
    size_t writeOffset = writeMemAddr * 8;

    // std::vector<uint8_t> mem0 is 64 * 128, therefore each resp from dma will only take 8 elements in each row
    if (curBlockRowPtr < mem0.size() && (pktSize + writeOffset) <= mem0[curBlockRowPtr].size()) {
        std::memcpy(mem0[curBlockRowPtr].data() + writeOffset, dmaData, pktSize);
    } else {
        panic("P2S_R: mem0 buffer overflow! curBlockRowPtr=%u\n", curBlockRowPtr);
    }

    delete pkt;

    // the whole row finishes
    if ((pktSize + writeOffset) == curBlockNCols) {

        writeMemAddr = 0;
        curBlockRowPtr++;
        curRowDramAddrOffset += next_row_offset_bytes;

        // the whole block finishes
        if (curBlockRowPtr == curBlockNRows) {
            curBlockRowPtr = 0;
            curBlockColPtr += curBlockNCols;
            curRowDramAddrOffset = 0;
            curBlockDramBaseAddrPtr += curBlockNCols;
            readMemAddr = 0;
            // fill dma from SRAM block into buffer
            schedule(loadBufferEvent, clockEdge(Cycles(1)));

        // schedule for the next row DMA
        } else {
            schedule(dmaReadEvent, clockEdge(Cycles(1)));
        }
    }
    else {
        // on the same row
        writeMemAddr++;
    }
    return true;
}


void
P2S_R::processDMAReadEvent() {
    // read 128 col into SRAM block
    RequestPtr request = std::make_shared<Request>(
        curBlockDramBaseAddr +
        static_cast<Addr>(dmaRow) * static_cast<Addr>(next_row_offset_bytes),                            // TODO
        sizeof(DMARPayload),                // next_row_offset_elem, base_dram_addr
        0,                                  // TODO
        requestorId
    );
    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);
    pkt->allocate();

    // ask DMA to get data by cache controller
    curBlockNCols = std::min(128, nCols - curBlockColPtr);
    // DMARPayload *dmaRPayload = new DMARPayload{curBlockNCols, next_row_offset_bytes, curBlockDramBaseAddrPtr + curRowDramAddrOffset};
    // pkt->dataDynamic(reinterpret_cast<uint8_t*>(dmaRPayload));
    DMARPayload dmaRPayload{curBlockNCols, next_row_offset_bytes, curBlockDramBaseAddrPtr + curRowDramAddrOffset};
    pkt->setData(reinterpret_cast<uint8_t*>(&dmaRPayload));
    bool success = DMAPort.sendTimingReq(pkt);
    if (success) {

    }
    else {
        // need to retry
        DMAPort.blockedPacket = pkt;
    } 

}
void
P2S_R::processLoadBufferEvent() {
    // fill the block into the buffer
    uint32_t readOffset = readMemAddr * 8;
    std::memcpy(regArray[writeBufRowPtr], mem0[writeBufRowPtr].data() + readOffset, 8 * sizeof(uint8_t));
    writeBufRowPtr++;

    if (writeBufRowPtr == curBlockNRows) {
        writeBufRowPtr = 0;
        readMemAddr++;
        schedule(bitSliceEvent, clockEdge(Cycles(1)));
    }
    else {
        schedule(loadBufferEvent, clockEdge(Cycles(1)));
    }

}
void
P2S_R::processBitSliceEvent() {
    // extract bits from raw data
    uint64_t bitSlice = extractBits(regArray, curEnqBlockInBufColPtr, bit_ptr);

    // determine the address
    uint64_t curArrayID = base_arrayID_to_store + arrayID_offset[bit_ptr];
    uint64_t arrayAddrEnq = curArrayID * WORDLINENUMS + curBlockColPtrGlobal + curBufColPtrInBlock + curEnqBlockInBufColPtr;
    
    // pack into packets
    RequestPtr request = std::make_shared<Request>(
        0,            // the target MMIO address of cache bank (TODO)
        sizeof(P2SWritePayload),     // store address + bitSlice
        0,                           // flags
        requestorId
    );

    // TODO how to couple p2sWritePayload with Packet?
    PacketPtr bitSlicePkt = new Packet(request, MemCmd::WriteReq);
    bitSlicePkt->allocate();
    // P2SWritePayload *p2sWritePayload = new P2SWritePayload{arrayAddrEnq, bitSlice};
    // bitSlicePkt.dataDynamic(reinterpret_cast<uint8_t*>(p2sWritePayload));
    P2SWritePayload p2sWritePayload{arrayAddrEnq, bitSlice};
    bitSlicePkt->setData(reinterpret_cast<uint8_t*>(&p2sWritePayload));
    // enqueue into write queue
    bitSliceQueue.push_back(bitSlicePkt);
    // write to cache bank
    schedule(writeEvent, clockEdge(Cycles(1)));

    // per bit
    bit_ptr++;
    if (bit_ptr <= precision) {
        schedule(bitSliceEvent, clockEdge(Cycles(1)));
    }
    else {
        // per row in the buffer
        bit_ptr = 0;
        curEnqBlockInBufColPtr++;
        if (curEnqBlockInBufColPtr == curBufNCols) {
            // per buffer
            curEnqBlockInBufColPtr = 0;
            if ((curBufColPtrInBlock + curBufNCols) == blockNColInMem) {
                curBufColPtrInBlock = 0;
                // per block
                if ((curBlockColPtrGlobal + blockNColInMem) == nCols) {
                    // TODO send p2s_done
                }
                else {
                    curBlockColPtrGlobal += blockNColInMem;
                    blockNColInMem = std::min(128, nCols - curBlockColPtrGlobal);
                    // (TO BE CHECKED)
                    schedule(dmaReadEvent, clockEdge(Cycles(1)));
                }
            }
            else {
                curBufColPtrInBlock += curBufNCols;
                curBufNCols = std::min(8, blockNColInMem - curBufColPtrInBlock);
                // (TO BE CHECKED)
                schedule(loadBufferEvent, clockEdge(Cycles(1)));
            }

        }
        else schedule(bitSliceEvent, clockEdge(Cycles(1)));
    }
}

void
P2S_R::processWriteEvent() {
    if (!bitSliceQueue.empty()) {
        PacketPtr pkt = bitSliceQueue.front();
        bool success = CacheBankPort.sendTimingReq(pkt);
        if (success) {
            bitSliceQueue.pop_front();
            schedule(writeEvent, clockEdge(Cycles(1)));
        }
        else {
            // p2s is stalled, need to wait for cache bank notify to retry
        }
    }
    else if (p2sDone) {
        // send p2s_done to scheduler
        pendingReqPkt->makeResponse();
        if (instPort.sendTimingResp(pendingReqPkt)) {
            DPRINTF(
            P2S_R,
            "CONTROL COMPLETE: final CacheBank write accepted\n");

            pendingReqPkt = nullptr;
            p2sDone = false;
        }
    }
}

uint64_t
P2S_R::extractBits(const std::vector<std::vector<uint8_t>> &arr, uint32_t row, uint8_t bit, uint32_t dim) {
    uint64_t extractedBit = 0;
    uint64_t bitSlice = 0;

    assert(row < dim);

    // for each element in the array
    for (int j = 0; j < 64; j++) {
        // here take regArray as regArray.transpose
        extractedBit = (arr[j][row] >> bit) & 0x1;
        bitSlice |= (extractedBit << j);
    }
    return bitSlice;
}
void
P2S_R::get_array_relatice_offset(std::vector<uint8_t> &offset, uint8_t numBuf) { // numBuf is 2 bit in fact
    if (numBuf == 3) offset = std::vector<uint8_t>{4, 4, 4, 4, 4, 4, 4};        // therefore later arrayID_offset could be [0, 4, 8, 12, 16, 20, 24, 28]
    else if (numBuf == 2) offset = std::vector<uint8_t>{1, 3, 1, 3, 1, 3, 1};   // therefore later arrayID_offset could be [0, 1, 4, 5, 8, 9, 12, 13]
    else if (numBuf == 1) offset = std::vector<uint8_t>{1, 1, 2, 1, 1, 2, 1};   // therefore later arrayID_offset could be [0, 1, 2, 4, 5, 6, 8, 9]
}
}
