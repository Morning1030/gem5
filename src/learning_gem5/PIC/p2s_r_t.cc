#include "learning_gem5/PIC/p2s.hh"
#include "learning_gem5/PIC/scheduler.hh"
#include "sim/system.hh"

#include <cstring>

#include "debug/P2S_R_T.hh"

#define WORDLINENUMS 512
namespace gem5
{
P2S_R_T::P2S_R_T(const P2S_R_TParams *params) :
    ClockedObject(params),
    instPort(params.name + ".cpu_port", this),
    DMAPort(params.name + ".dma_port", this, MemSidePort::PICPortID::DMA),
    CacheBankPort(params.name + ".cb_port", this, MemSidePort::PICPortID::CB),
    requestorId(system.getRequestorId(this, "P2S_R_T")),
    pendingReqPkt(nullptr),
    p2sDone(false),
    dmaReadEvent([this]{this->processDMAReadEvent();}, "dmaReadEvent"),
    bitSliceEvent([this]{this->processBitSliceEvent();}, "bitSliceEvent"),
    writeEvent([this]{this->processWriteEvent();}, "writeBankEvent")
{}
Port &
P2S_R_T::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port")
        return instPort;

    if (if_name == "dma_port")
        return DMAPort;

    if (if_name == "cb_port")
        return CacheBankPort;

    return ClockedObject::getPort(if_name, idx);
}
P2S_R_T::CPUSidePort::CPUSidePort(
    const std::string &name,
    P2S_R_T *owner) :
    ResponsePort(name, owner),
    owner(owner),
    blockedPacket(nullptr)
{}
bool
P2S_R_T::CPUSidePort::recvTimingReq(PacketPtr pkt) {
    // Just forward to the memobj.
    return owner->handleRequest(pkt);
}
void
P2S_R_T::CPUSidePort::sendPacket(PacketPtr pkt)
{
    // send p2s done to scheduler
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    if (sendTimingResp(pkt)) {
        owner->pendingReqPkt = nullptr;
        blockedPacket = nullptr;
        // p2sDone = true;
        DPRINTF(P2S_R_T, "P2S COMPLETE: send back to scheduler\n");
    }
    else blockedPacket = pkt;
}
void
P2S_R_T::CPUSidePort::recvRespRetry() {
    // retry to send resp to scheduler
    assert(blockedPacket != nullptr);

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
}
P2S_R_T::MemSidePort::MemSidePort(
    const std::string &name,
    P2S_R_T *owner) :
    RequestPort(name, owner),
    owner(owner),
    portID(picPortID),
    blockedPacket(blockedPacket)
{}
bool
P2S_R_T::MemSidePort::recvTimingResp(PacketPtr pkt) {
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
P2S_R_T::MemSidePort::recvReqRetry()
{
    if (this->portID == PICPortID::DMA) {
        assert(blockedPacket != nullptr);

        if (sendTimingReq(blockedPacket)) {
            blockedPacket = nullptr;
            DPRINTF(P2S_R_T, "DMA request accepted on retry\n");
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
        DPRINTF(P2S_R_T, "Unknown port id!\n");
    }
}
bool
P2S_R_T::handleRequest(PacketPtr pkt) {
    //assertion for second request
    // TBD instead of panic, return false to ask upstream to retry
    // panic_if( 
    //     pendingReqPkt != nullptr,
    //     "P2S_R_T received a second control request while one is active");
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
    
    // array offset
    get_array_relatice_offset(relative_offset_buf, bufNum);
    arrayID_offset[0] = 0;
    for (int i = 1; i < 8; i++) arrayID_offset[i] = arrayID_offset[i - 1] + relative_offset_buf[i - 1];

    // initialize data memebers
    bit_ptr = 0;
    row_store_ptr = 0;
    dmaRow = 0;
    schedule(dmaReadEvent, clockEdge(Cycles(1)));

    return true;
}
bool
P2S_R_T::handleResponse(PacketPtr pkt) {
    // fill the response to buffer
    // TODO need sender state row to deal with out of order receiving
    // TODO 一個packet到底是幾byte? 可能不需要dmaRow
    // TODO take care of out of order responses
    const uint8_t *dmaData = pkt->getConstPtr<uint8_t>();
    const size_t pktSize = pkt->getSize();
    size_t offset = dmaRow * 8;

    if (offset + pktSize <= bufArray.size() && pktSize <= sizeof(uint64_t)) {
        std::memcpy(bufArray.data() + offset, dmaData, pktSize);
    } else {
        panic("P2S_R_T: bufArray buffer overflow! dmaRow=%u\n", dmaRow);
    }

    delete pkt;
    dmaRow++;

    if (offset + pktSize == bufArray.size()) {
        dmaRow = 0;
        bit_ptr = 0;

        // finish filling dma into buffer
        schedule(bitSliceEvent, clockEdge(Cycles(1)));

    } else {
        // need to wait for other dmaRows to finish
    }
    return true;
    
}

void
P2S_R_T::processDMAReadEvent() {
    // read one row in R Tile

    RequestPtr request = std::make_shared<Request>(
        dramAddr,                   // TBD
        sizeof(DMARTPayload),       // payload size
        0,                          // flags
        requestorId
    );
    // TODO Read Request should not have dataPayload
    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);
    pkt->allocate();
    // ask DMA to get data by cache controller
    // DMARTPayload* dmaRTPayload = new DMARTPayload{nCols, dramAddr};
    // pkt->dataDynamic(reinterpret_cast<uint8_t*>(dmaRTPayload));
    DMARTPayload dmaRTPayload{nCols, dramAddr};
    pkt->setData(reinterpret_cast<uint8_t*>(&dmaRTPayload));

    bool success = DMAPort.sendTimingReq(pkt);
    if (success) {
        dramAddr += next_row_offset_bytes; // update base_dram_addr for the next round
    }
    else {
        // need to retry
        DMAPort.blockedPacket = pkt;
    }
}
void
P2S_R_T::processBitSliceEvent() {

    // extract bits from raw data
    uint64_t bitSlice = extractBits(bufArray, bit_ptr);

    // determine the address
    uint64_t curArrayID = base_arrayID_to_store + arrayID_offset[bit_ptr];
    uint64_t arrayAddrEnq = curArrayID * WORDLINENUMS + row_store_ptr;
 
    RequestPtr request = std::make_shared<Request>(
        0,                           // TBD
        sizeof(P2SWritePayload),     // store address + bitSlice
        0,                           // TBD
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
    // write to cache bank
    if (!writeEvent.scheduled()) {
        schedule(writeEvent, clockEdge(Cycles(1)));
    }

    bit_ptr++;
    if (bit_ptr <= precision) {
        schedule(bitSliceEvent, clockEdge(Cycles(1)));
    }
    else {
        bit_ptr = 0;
        row_store_ptr++;
        if (row_store_ptr < nRows) {
            schedule(dmaReadEvent, clockEdge(Cycles(1)));
        }
        else {
            // send p2s_done to scheduler
            p2sDone = true;
        }
    }

}
void
P2S_R_T::processWriteEvent() {
    if (!bitSliceQueue.empty()) {
        PacketPtr pkt = bitSliceQueue.front();
        bool success = CacheBankPort.sendTimingReq(pkt);
        if (success) {
            bitSliceQueue.pop_front();
            schedule(writeEvent, clockEdge(Cycles(1)));
        }
        else {
            // p2s is stalled, need to wait for cache bank notify to retry
            DPRINTF(P2S_R_T, "Cache bank busy, stalling writeEvent.\n");
        }
    }
    else if (p2sDone) {
        // send p2s_done to scheduler
        pendingReqPkt->makeResponse();
        instPort.sendPacket(pendingReqPkt);
    }
}

uint64_t
P2S_R_T::extractBits(const std::vector<uint8_t> &buf, uint8_t bit) {
    uint64_t extractedBit = 0;
    uint64_t bitSlice = 0;

    for (int i = 0; i < 64; i++) {
        // extract the bit for element i
        extractedBit = (buf[i] >> bit) & 0x1;

        // shift it to bit and OR to bitSlice
        bitSlice |= (extractedBit << i);
    }
    // element 63, 62, 61, 60 .... 0
    return bitSlice;
}


void
P2S_R::get_array_relatice_offset(std::vector<uint8_t> &offset, uint8_t numBuf) { // numBuf is 2 bit in fact
    if (numBuf == 3) offset = std::vector<uint8_t>{4, 4, 4, 4, 4, 4, 4};        // therefore later arrayID_offset could be [0, 4, 8, 12, 16, 20, 24, 28]
    else if (numBuf == 2) offset = std::vector<uint8_t>{1, 3, 1, 3, 1, 3, 1};   // therefore later arrayID_offset could be [0, 1, 4, 5, 8, 9, 12, 13]
    else if (numBuf == 1) offset = std::vector<uint8_t>{1, 1, 2, 1, 1, 2, 1};   // therefore later arrayID_offset could be [0, 1, 2, 4, 5, 6, 8, 9]
}
}
