#include "learning_gem5/PIC/p2s.hh"
#include "learning_gem5/PIC/scheduler.hh"
#include "sim/system.hh"

#include <algorithm>
#include <cstring>

#include "debug/P2S_L.hh"

namespace gem5
{
P2S_L::P2S_L(const P2S_LParams &params) :
    ClockedObject(params),
    instPort(params.name + ".cpu_port", this, nullptr),
    DMAPort(params.name + ".dma_port", this, MemSidePort::PICPortID::DMA, nullptr),
    CacheBankPort(params.name + ".cb_port", this, MemSidePort::PICPortID::CB, nullptr),
    requestorId(system.getRequestorId(this, "P2S_L")),
    pendingReqPkt(nullptr),
    p2sDone(true),
    dmaReadEvent([this]{this->processDMAReadEvent();}, "dmaReadEvent"),
    bitSliceEvent([this]{this->processBitSliceEvent();}, "bitSliceEvent"),
    writeEvent([this]{this->processWriteEvent();}, "writeBankEvent")  
{}
Port &
P2S_L::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port")
        return instPort;

    if (if_name == "dma_port")
        return DMAPort;

    if (if_name == "cb_port")
        return CacheBankPort;

    return ClockedObject::getPort(if_name, idx);
}

P2S_L::CPUSidePort::CPUSidePort(
    const std::string &name,
    P2S_L *owner,
    PacketPtr blockedPacket) :
    ResponsePort(name, owner),
    owner(owner),
    blockedPacket(blockedPacket)
{}
bool
P2S_L::CPUSidePort::recvTimingReq(PacketPtr pkt){
    // Just forward to the memobj.
    return owner->handleRequest(pkt);
}
void
P2S_L::CPUSidePort::sendPacket(PacketPtr pkt)
{
    // send p2s done to scheduler
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    if (sendTimingResp(pkt)) {
        owner->pendingReqPkt = nullptr;
        blockedPacket = nullptr;
        // p2sDone = true;
        DPRINTF(P2S_L, "P2S COMPLETE: send back to scheduler\n");
    }
    else blockedPacket = pkt;
}
void
P2S_L::CPUSidePort::recvRespRetry()
{
    // retry to send resp to scheduler
    assert(blockedPacket != nullptr);

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
}

P2S_L::MemSidePort::MemSidePort(
    const std::string &name,
    P2S_L *owner,
    PICPortID picPortID) :
    RequestPort(name, owner),
    owner(owner),
    portID(picPortID),
    blockedPacket(blockedPacket)
{}
bool
P2S_L::MemSidePort::recvTimingResp(PacketPtr pkt) {
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
P2S_L::MemSidePort::recvReqRetry()
{
    if (this->portID == DMA) {
        assert(blockedPacket != nullptr);

        if (sendTimingReq(blockedPacket)) {
            blockedPacket = nullptr;
            DPRINTF(P2S_L, "P2S COMPLETE: send back to scheduler\n");
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
        DPRINTF(P2S_L, "Unknown port id!\n");
    }
}
bool
P2S_L::handleRequest(PacketPtr pkt) {
    //assertion for second request
    // TBD instead of panic, return false to ask upstream to retry
    // panic_if( 
    //     pendingControlPkt != nullptr,
    //     "P2S_L received a second control request while one is active");
    
    if (pendingReqPkt != nullptr || p2sDone == false) {
        // needRetry = true;
        return false;
    }
    // currently working on this req
    pendingReqPkt = pkt;
    p2sDone = false;
    const P2S_L_Payload *p2s_L_Payload = pkt->getConstPtr<P2S_L_Payload>();
    
    // fill the packet field into data members of p2s
    base_dram_addr = p2s_L_Payload->base_dramAddr_to_load;
    base_picAddr = p2s_L_Payload->base_picAddr_to_store;
    pic_write_ptr = p2s_L_Payload->base_picAddr_to_store;
    next_row_offset_elem = p2s_L_Payload->next_row_offset_elem;
    next_row_offset_dram = p2s_L_Payload->next_row_offset_elem;
    _L_block_row = p2s_L_Payload->_L_block_row;
    _L_block_row_ptr = 0;
    next_slice_offset_pic = p2s_L_Payload->_L_block_row;
    precision = p2s_L_Payload->precision;
    bit_ptr = 0;
    dmaRow = 0;

    schedule(dmaReadEvent, clockEdge(Cycles(1)));
    return true;
}

bool
P2S_L::handleResponse(PacketPtr pkt) {
    // TODO identiry response from dma / cache bank
    // TODO need sender state row to deal with out of order receiving
    // TBD 一個packet到底是幾byte? 可能不需要dmaRow
    // TODO take care of out of order responses
    const uint8_t *dmaData = pkt->getConstPtr<uint8_t>();
    const size_t pktSize = pkt->getSize();

    if (dmaRow < regArray.size() && pktSize <= regArray[dmaRow].size()) {
        std::memcpy(regArray[dmaRow].data(), dmaData, pktSize);
    } else {
        panic("P2S_L: regArray buffer overflow! dmaRow=%u\n", dmaRow);
    }

    delete pkt;
    dmaRow++;

    if (dmaRow == regArray.size()) {
        dmaRow = 0;
        bit_ptr = 0;

        pic_write_ptr = base_picAddr + _L_block_row_ptr;
        schedule(bitSliceEvent, clockEdge(Cycles(1)));

    } else {
        // need to wait for other dmaRows to finish
    }
    return true;
}

void
P2S_L::processDMAReadEvent() {
    // read one row in L Tile

    RequestPtr request = std::make_shared<Request>(
        base_dram_addr,                     // TBD
        sizeof(DMALPayload),                // next_row_offset_elem, base_dram_addr
        0,                                  // TBD
        requestorId
    );
    // TODO Read Request should not have dataPayload
    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);
    pkt->allocate();
    // ask DMA to get data by cache controller
    // DMALPayload* dmaLPayload = new DMALPayload{next_row_offset_elem, base_dram_addr};
    // pkt->dataDynamic(reinterpret_cast<uint8_t*>(dmaLPayload));
    DMALPayload dmaLPayload{next_row_offset_elem, base_dram_addr};
    pkt->setData(reinterpret_cast<uint8_t*>(&dmaLPayload));

    bool success = DMAPort.sendTimingReq(pkt);
    if (success) {
        base_dram_addr += next_row_offset_dram; // update base_dram_addr for the next round
    }
    else {
        // need to retry
        DMAPort.blockedPacket = pkt;
    }
}
void
P2S_L::processBitSliceEvent() {
    // each bit of elements in the whole row

    // extract bits from raw data
    uint64_t bitSlice = extractBits(regArray, bit_ptr);

    // determine the address and pack into packets

    RequestPtr request = std::make_shared<Request>(
        0,                          // TBD
        sizeof(P2SWritePayload),    // store address + bitSlice
        0,                          // TBD
        requestorId
    );

    PacketPtr bitSlicePkt = new Packet(request, MemCmd::WriteReq);
    bitSlicePkt->allocate();
    // P2SWritePayload *p2sWritePayload = new P2SWritePayload{pic_write_ptr, bitSlice};
    // bitSlicePkt->dataDynamic(reinterpret_cast<uint8_t*>(p2sWritePayload));
    P2SWritePayload p2sWritePayload{pic_write_ptr, bitSlice};
    bitSlicePkt->setData(reinterpret_cast<uint8_t*>(&p2sWritePayload));

    // enqueue into write queue
    bitSliceQueue.push_back(bitSlicePkt);
    if (!writeEvent.scheduled()) {
        schedule(writeEvent, clockEdge(Cycles(1)));
    }

    // update the next address
    pic_write_ptr += next_slice_offset_pic; // next_slice_offset_pic = _L_block_row(?)

    // finish one bit slice, move on to the next bit
    bit_ptr++;
    if (bit_ptr <= precision) {
        schedule(bitSliceEvent, clockEdge(Cycles(1)));
    }
    // finish one row, move on to the next row
    else {
        bit_ptr = 0;
        _L_block_row_ptr++;
        if (_L_block_row_ptr < _L_block_row) {
            schedule(dmaReadEvent, clockEdge(Cycles(1)));
        }
        else {
            // the whole L tile is done
            // TODO this is wrong, should wait for cache bank's response to notify p2s done
            p2sDone = true;
        }
    }
}

uint64_t
P2S_L::extractBits(const std::vector<std::vector<uint8_t>> &arr, uint8_t bit) {
    uint64_t extractedBit = 0;
    uint64_t bitSlice = 0;

    // for each element in the array
    for (int i = 0; i < 64; i++) {
        // extract the bit for element i
        extractedBit = (arr[i / 8][i % 8] >> bit) & 0x1;

        // shift it to bit and OR to bitSlice
        bitSlice |= (extractedBit << i);
    }
    // element 63, 62, 61, 60 .... 0
    return bitSlice;
}

void
P2S_L::processWriteEvent() {
    if (!bitSliceQueue.empty()) {
        PacketPtr pkt = bitSliceQueue.front();
        bool success = CacheBankPort.sendTimingReq(pkt);
        if (success) {
            bitSliceQueue.pop_front();
            schedule(writeEvent, clockEdge(Cycles(1)));
        }
        else {
            // p2s is stalled, need to wait for cache bank notify to retry
            // do nothing and wait until notification
            DPRINTF(P2S_L, "Cache bank busy, stalling writeEvent.\n");
        }
    }
    else if (p2sDone) {
        // send p2s_done to scheduler(might return false, but instPort will handle it)
        // panic_if(pendingReqPkt == nullptr);
        pendingReqPkt->makeResponse();
        instPort.sendPacket(pendingReqPkt);
    }
}

}
