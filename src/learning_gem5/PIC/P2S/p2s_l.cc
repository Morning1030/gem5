#include "learning_gem5/PIC/P2S/p2s.hh"
#include "learning_gem5/PIC/Control/scheduler.hh"
#include "sim/system.hh"

#include <algorithm>
#include <cstring>

#include "debug/P2S_L.hh"

namespace gem5
{
P2S_L::P2S_L(const P2S_LParams &params) :
    ClockedObject(params),
    instPort(params.name + ".inst_port", this),
    DMAPort(params.name + ".dma_port", this, PICPortID::DMA),
    cacheBankPort(params.name + ".cb_port", this, PICPortID::CB),
    requestorId(params.system->getRequestorId(this, "P2S_L")),
    pendingReqPkt(nullptr),
    p2sDone(true),
    regArray(8, std::vector<uint8_t>(8, 0)),
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
        return cacheBankPort;

    return ClockedObject::getPort(if_name, idx);
}

P2S_L::CPUSidePort::CPUSidePort(
    const std::string &name,
    P2S_L *owner) :
    ResponsePort(name),
    owner(owner),
    blockedPacket(nullptr)
{}
bool
P2S_L::CPUSidePort::recvTimingReq(PacketPtr pkt) {
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
    RequestPort(name),
    owner(owner),
    picPortID(picPortID),
    blockedPacket(nullptr)
{}
bool
P2S_L::MemSidePort::recvTimingResp(PacketPtr pkt) {
    return owner->handleResponse(this->picPortID, pkt);
}
void
P2S_L::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");
    if (sendTimingReq(pkt)) {
        if (pkt->isRead()) {
            // DPRINTF(P2S_L, "Successfully send request to DMA\n");
        }
        else if(pkt->isWrite()) {
            // DPRINTF(P2S_L, "Successfully send request to cache bank\n");
        }
    }
    else {
        blockedPacket = pkt;
    }
}
void
P2S_L::MemSidePort::recvReqRetry()
{
    if (this->picPortID ==  PICPortID::DMA) {
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
        DPRINTF(P2S_L, "Unknown port id!\n");
    }
}
bool
P2S_L::handleRequest(PacketPtr pkt) {
    
    if (pendingReqPkt != nullptr || p2sDone == false) return false;
    
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
    precision = p2s_L_Payload->precision;
    bit_ptr = 0;

    schedule(dmaReadEvent, clockEdge(Cycles(1)));
    return true;
}

bool
P2S_L::handleResponse(PICPortID picPortID, PacketPtr pkt) {
    if (picPortID == PICPortID::DMA) {
        const uint8_t *dmaData = pkt->getConstPtr<uint8_t>();
        const size_t pktSize = pkt->getSize();
        DPRINTF(P2S_L, "DMA response size=%u\n", pktSize);
        
        if (pktSize <= regArray.size() * regArray[0].size()) {
            for(int dmaRow = 0; dmaRow < regArray.size(); dmaRow++) {
                size_t copySize = std::min(regArray[dmaRow].size(), pktSize - dmaRow * regArray[dmaRow].size());
                if (copySize > 0) {
                    std::memcpy(regArray[dmaRow].data(), dmaData + dmaRow * regArray[dmaRow].size(), copySize);
                }
            }
        } else {
            panic("P2S_L: regArray buffer overflow!\n");
        }
        delete pkt;

        bit_ptr = 0;
        pic_write_ptr = base_picAddr + _L_block_row_ptr * 8;
        schedule(bitSliceEvent, clockEdge(Cycles(1)));
        return true;
    }
    else if (picPortID == PICPortID::CB) {
        // TODO need to filter whether p2sDone
        delete pkt;
        if (p2sDone) {
            pendingReqPkt->makeResponse();
            instPort.sendPacket(pendingReqPkt);
        }
        return true;
    }
    else {
        DPRINTF(P2S_L, "Cannot recognize port\n");
        return false;
    }
}

void
P2S_L::processDMAReadEvent() {
    // read one row in L Tile
    if (DMAPort.blockedPacket != nullptr) {
        DPRINTF(P2S_L, "There is still DMA blocked Packet, DMA read is stalled\n");
        return;
    }
    RequestPtr request = std::make_shared<Request>(
        base_dram_addr,
        64,                                 // regArray is 64 bytes
        0,                                  // TBD
        requestorId
    );
    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);
    pkt->allocate();

    DMADescriptor desc {
        base_dram_addr,
        0,
        64,
        0,
        0
    };
    pkt->pushSenderState(new DMASenderState(desc));
    DMAPort.sendPacket(pkt);
    DPRINTF(P2S_L, "Send DMA Request to %llx\n", base_dram_addr);
    base_dram_addr += next_row_offset_dram;
}
void
P2S_L::processBitSliceEvent() {
    // extract bits from raw data
    uint64_t bitSlice = extractBits(regArray, bit_ptr);

    // determine the address
    RequestPtr request = std::make_shared<Request>(
        pic_write_ptr,
        sizeof(uint64_t),           // bitSlice
        0,                          // TBD
        requestorId
    );

    PacketPtr bitSlicePkt = new Packet(request, MemCmd::WriteReq);
    bitSlicePkt->allocate();
    // P2SWritePayload *p2sWritePayload = new P2SWritePayload{pic_write_ptr, bitSlice};
    // bitSlicePkt->dataDynamic(reinterpret_cast<uint8_t*>(p2sWritePayload));
    // P2SWritePayload p2sWritePayload{pic_write_ptr, bitSlice};
    bitSlicePkt->setData(reinterpret_cast<uint8_t*>(&bitSlice));

    // enqueue into write queue
    bitSliceQueue.push_back(bitSlicePkt);
    DPRINTF(P2S_L, "Send Request writing bit %d to %llx\n", bit_ptr, pic_write_ptr);
    if (!writeEvent.scheduled()) {
        schedule(writeEvent, clockEdge(Cycles(1)));
    }

    // update the next address
    pic_write_ptr += _L_block_row * 8;

    // finish one bit slice, move on to the next bit
    bit_ptr++;
    if (bit_ptr <= precision) {
        schedule(bitSliceEvent, clockEdge(Cycles(1)));
    }
    // finish one row, move on to the next row
    else {
        _L_block_row_ptr++;
        if (_L_block_row_ptr < _L_block_row) {
            if (!dmaReadEvent.scheduled()) {
                schedule(dmaReadEvent, clockEdge(Cycles(1)));
            }
        }
        else {
            // the whole L tile is done
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
        bool success = cacheBankPort.sendTimingReq(pkt);
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
}

}
