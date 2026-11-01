#include "learning_gem5/PIC/P2S/p2s.hh"
#include "learning_gem5/PIC/Control/scheduler.hh"
#include "sim/sim_exit.hh"
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
    instPort(params.name + ".inst_port", this),
    DMAPort(params.name + ".dma_port", this, PICPortID::DMA),
    cacheBankPort(params.name + ".cb_port", this, PICPortID::CB),
    requestorId(params.system->getRequestorId(this, "P2S_R")),
    pendingReqPkt(nullptr),
    p2sDone(true),
    exitOnDone(params.exit_on_done),
    exitDelay(params.exit_delay),
    relative_offset_buf(7),
    arrayID_offset(8),
    mem0(64, std::vector<uint8_t>(128, 0)),      // SRAM block is 64 * 128
    regArray(64, std::vector<uint8_t>(8, 0)),    // p2s_R it's 64 * 8
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
        return cacheBankPort;

    return ClockedObject::getPort(if_name, idx);
}

P2S_R::CPUSidePort::CPUSidePort(
    const std::string &name,
    P2S_R *owner) :
    ResponsePort(name),
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
        if (owner->exitOnDone) {
            exitSimLoop(owner->name() + " done", 0,
                        curTick() + owner->cyclesToTicks(owner->exitDelay));
        }
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
    P2S_R *owner,
    PICPortID picPortID) :
    RequestPort(name),
    owner(owner),
    picPortID(picPortID),
    blockedPacket(nullptr)
{}
bool
P2S_R::MemSidePort::recvTimingResp(PacketPtr pkt) {
    return owner->handleResponse(this->picPortID, pkt);
}
void
P2S_R::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");
    if (sendTimingReq(pkt)) {
        if (pkt->isRead()) {
            DPRINTF(P2S_R, "Successfully send request to DMA\n");
        }
        else if(pkt->isWrite()) {
            DPRINTF(P2S_R, "Successfully send request to cache bank\n");
        }
    }
    else {
        blockedPacket = pkt;
    }
}
void
P2S_R::MemSidePort::recvReqRetry()
{
    if (this->picPortID == PICPortID::DMA) {
        panic_if(blockedPacket == nullptr, "There's no blockedPacket to send");

        PacketPtr pkt = blockedPacket;
        blockedPacket = nullptr;
        sendPacket(pkt);
        // might fail again, wait for another req retry
    }
    // retry req from cache bank, start writeEvent again from cache write queue
    else if (this->picPortID == PICPortID::CB){
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
    blockNColInMem = std::min(128U, nCols - curBlockColPtrGlobal);
    curBufNCols = std::min(8U, blockNColInMem - curBufColPtrInBlock);

    // dma and buffer variables
    curBlockRowPtr = 0;
    curBlockColPtr = 0;
    curBlockNCols = std::min(128U, nCols - curBlockColPtr);          // equal to blockNColInMem
    curBlockNRows = nRows;
    writeBufRowPtr = 0;
    readMemAddr= 0;
    writeMemAddr = 0;
    curBlockDramBaseAddrPtr = p2s_R_Payload->dramAddr;
    curRowDramAddrOffset = 0;

    get_array_relative_offset(relative_offset_buf, bufNum);
    arrayID_offset[0] = 0;
    for (int i = 1; i < 8; i++) arrayID_offset[i] = arrayID_offset[i - 1] + relative_offset_buf[i - 1];

    schedule(dmaReadEvent, clockEdge(Cycles(1)));

    return true;
}

bool
P2S_R::handleResponse(PICPortID picPortID, PacketPtr pkt) {
    if (picPortID == PICPortID::DMA) {
        const uint8_t *dmaData = pkt->getConstPtr<uint8_t>(); // dma send a uint64_t variable but can be expressed as 8 * 8 uint8_t
        size_t pktSize = pkt->getSize();
        // DPRINTF(P2S_R, "DMA data packet size = %u\n", pktSize);
        // size_t writeOffset = writeMemAddr * 8;

        // std::vector<uint8_t> mem0 is 64 * 128, one DMA response fills one row of it
        if (curBlockRowPtr < mem0.size() && pktSize <= mem0[curBlockRowPtr].size()) {
            std::memcpy(mem0[curBlockRowPtr].data(), dmaData, pktSize);
            DPRINTF(P2S_R, "Got DMA response packet size = %u for mem0 row %u\n", pktSize, curBlockRowPtr);
        } else {
            panic("P2S_R: mem0 buffer overflow! curBlockRowPtr=%u pktSize=%u\n", curBlockRowPtr, pktSize);
        }

        delete pkt;

        // the whole row finishes
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
            if (!loadBufferEvent.scheduled()) {
                schedule(loadBufferEvent, clockEdge(Cycles(1)));
            }
        }
        // schedule for the next row DMA
        else {
            schedule(dmaReadEvent, clockEdge(Cycles(1)));
        }
        return true;
    }
    else if (picPortID == PICPortID::CB) {
        delete pkt;
        // Done only after the last queued write is acked. The RTL P2S_R
        // busy does not include its sliceDataQueue: it drops to 0 once the
        // last slice is enqueued, with up to 8 writes not yet through the
        // arbiter.
        if (p2sDone && bitSliceQueue.empty()) {
            pendingReqPkt->makeResponse();
            instPort.sendPacket(pendingReqPkt);
        }
        return true;
    }
    else {
        DPRINTF(P2S_R, "Cannot recognize port\n");
        return false;
    }
}


void
P2S_R::processDMAReadEvent() {
    // read one row (up to 128 col) of the block into SRAM block, as RTL
    // P2S_R_ctl does: one DMA request per row, P2S_R steps the row stride
    if (DMAPort.blockedPacket != nullptr) {
        DPRINTF(P2S_R, "There is still DMA blocked Packet, DMA read is stalled\n");
        return;
    }
    curBlockNCols = std::min(128U, nCols - curBlockColPtr);
    const Addr rowAddr = curBlockDramBaseAddrPtr + curRowDramAddrOffset;
    RequestPtr request = std::make_shared<Request>(
        rowAddr,
        curBlockNCols,                      // this row's bytes
        0,
        requestorId
    );
    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);
    pkt->allocate();

    DMADescriptor desc {
        rowAddr,
        0,
        curBlockNCols,
        1,
        next_row_offset_bytes
    };
    pkt->pushSenderState(new DMASenderState(desc));
    DMAPort.sendPacket(pkt);
    DPRINTF(P2S_R, "Send DMA read request to %llx\n", rowAddr);

}
void
P2S_R::processLoadBufferEvent() {
    // fill the block into the buffer, regArray = 64 * 8
    uint32_t readOffset = readMemAddr * 8;
    std::memcpy(regArray[writeBufRowPtr].data(), mem0[writeBufRowPtr].data() + readOffset, 8 * sizeof(uint8_t));    // TBD instead of 8, shoule be curBufNCols
    DPRINTF(P2S_R, "load mem0 data to buffer, from mem0 row %d col %d\n", writeBufRowPtr, readOffset);
    
    writeBufRowPtr++;

    if (writeBufRowPtr == curBlockNRows) {
        writeBufRowPtr = 0;
        readMemAddr++;
        schedule(bitSliceEvent, clockEdge(Cycles(1)));
    }
    else {
        // self loop 64 times
        schedule(loadBufferEvent, clockEdge(Cycles(1)));
    }

}
void
P2S_R::processBitSliceEvent() {
    // extract bits from raw data
    uint64_t bitSlice = extractBits(regArray, curEnqBlockInBufColPtr, bit_ptr);

    // determine the address
    uint64_t curArrayID = base_arrayID_to_store + arrayID_offset[bit_ptr];
    // inform("curArrayID = %llu", curArrayID);
    uint64_t arrayAddrEnq = curArrayID * WORDLINENUMS + curBlockColPtrGlobal + curBufColPtrInBlock + curEnqBlockInBufColPtr;
    
    // pack into packets
    RequestPtr request = std::make_shared<Request>(
        arrayAddrEnq,                // the target MMIO address of cache bank (TODO)
        sizeof(P2SWritePayload),     // store address + bitSlice
        0,                           // flags
        requestorId
    );

    PacketPtr bitSlicePkt = new Packet(request, MemCmd::WriteReq);
    bitSlicePkt->allocate();
    // P2SWritePayload *p2sWritePayload = new P2SWritePayload{arrayAddrEnq, bitSlice};
    // bitSlicePkt.dataDynamic(reinterpret_cast<uint8_t*>(p2sWritePayload));
    P2SWritePayload p2sWritePayload{arrayAddrEnq, bitSlice};
    bitSlicePkt->setData(reinterpret_cast<uint8_t*>(&p2sWritePayload));
    // enqueue into write queue
    bitSliceQueue.push_back(bitSlicePkt);
    DPRINTF(P2S_R, "Send bit=%u bitslice writing to %llx\n", bit_ptr, arrayAddrEnq);
    // write to cache bank
    if (!writeEvent.scheduled()) {
        schedule(writeEvent, clockEdge(Cycles(1)));
    }

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
                    p2sDone = true;
                }
                else {
                    curBlockColPtrGlobal += blockNColInMem;
                    blockNColInMem = std::min(128U, nCols - curBlockColPtrGlobal);
                    // (TO BE CHECKED)
                    schedule(dmaReadEvent, clockEdge(Cycles(1)));
                }
            }
            else {
                curBufColPtrInBlock += curBufNCols;
                curBufNCols = std::min(8U, blockNColInMem - curBufColPtrInBlock);
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
        bool success = cacheBankPort.sendTimingReq(pkt);
        if (success) {
            bitSliceQueue.pop_front();
            schedule(writeEvent, clockEdge(Cycles(1)));
        }
        else {
            // p2s is stalled, need to wait for cache bank notify to retry
        }
    }
}

uint64_t
P2S_R::extractBits(const std::vector<std::vector<uint8_t>> &arr, uint32_t row, uint8_t bit) {
    uint64_t extractedBit = 0;
    uint64_t bitSlice = 0;

    // for each element in the array
    for (int j = 0; j < 64; j++) {
        // here take regArray as regArray.transpose
        extractedBit = (arr[j][row] >> bit) & 0x1;
        bitSlice |= (extractedBit << j);
    }
    return bitSlice;
}
}
