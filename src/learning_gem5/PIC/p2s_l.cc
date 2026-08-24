#include "learning_gem5/PIC/p2s.hh"
#include "learning_gem5/PIC/scheduler.hh"
#include "sim/system.hh"
#include <algorithm>
#include <cstring>
#include "debug/P2S_L.hh"

/* DATA MEMBERS
precision=precision
uint64_t base_dram_addr = base_dramAddr_to_load
uint64_t base_picAddr=RegInit(0.U((sysCfg.accessCacheFullAddrLen).W))
uint64_t pic_write_ptr = base_picAddr_to_store;
next_row_offset_elem=next_row_offset_elem
next_row_offset_dram=(next_row_offset_elem*bits_per_ele)>>log2Ceil(8)
_L_block_row =_L_block_row
_L_block_row_ptr=RegInit(0.U(sysCfg._L_nRow_sigLen.W))
next_slice_offset_pic=_L_block_row
*/

/*
P2S_L REQUEST PARAMETERS
base_dramAddr_to_load
base_picAddr_to_store
_L_block_row
next_row_offset_elem
precision
*/
namespace gem5
{
P2S_L::P2S_L(const P2S_LParams &params) :
    ClockedObject(params),
    instPort(name() + ".inst_port", this),
    DMAPort(name() + ".dma_port", this),
    CacheBankPort(name() + ".cb_port", this),
    requestorId(params.system->getRequestorId(this, "P2S_L")),
    regArray(8, std::vector<uint8_t>(8, 0)),
    dmaReadEvent([this]{ processDMAReadEvent(); }, "dmaReadEvent"),
    bitSliceEvent([this]{ processBitSliceEvent(); }, "bitSliceEvent"),
    writeEvent([this]{ processWriteEvent(); }, "writeBankEvent")
{}


//===== gem5 port glue ===//
P2S_L::CPUSidePort::CPUSidePort(
    const std::string &name,
    P2S_L *owner) :
    ResponsePort(name, owner),
    owner(owner)
{}

bool
P2S_L::CPUSidePort::sendPacket(PacketPtr pkt)
{
    return sendTimingResp(pkt);
}

void
P2S_L::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    panic("P2S_L functional access is not implemented");
}

void
P2S_L::CPUSidePort::recvRespRetry()
{
    owner->retryControlResponse();
}

AddrRangeList
P2S_L::CPUSidePort::getAddrRanges() const
{
    return {};
}

P2S_L::MemSidePort::MemSidePort(
    const std::string &name,
    P2S_L *owner) :
    RequestPort(name, owner),
    owner(owner)
{}

void
P2S_L::MemSidePort::recvReqRetry()
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
//===========================//

bool
P2S_L::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    return owner->handleRequest(pkt);
}

void
P2S_L::MemSidePort::sendPacket(PacketPtr pkt)
{
    sendTimingReq(pkt);
}

// TODO take care of out of order responses
bool
P2S_L::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    const uint8_t *dmaData =
        pkt->getConstPtr<uint8_t>();

    const size_t pktSize = pkt->getSize();

    if (pktSize == 64) {
        panic_if(
            owner->regArray.size() != 8,
            "P2S_L: expected 8 regArray words, got %zu",
            owner->regArray.size());

        for (uint32_t word = 0; word < 8; ++word) {
            panic_if(
                owner->regArray[word].size() != 8,
                "P2S_L: regArray[%u] expected 8 bytes, got %zu",
                word,
                owner->regArray[word].size());

            std::memcpy(
                owner->regArray[word].data(),
                dmaData + word * 8,
                8);
        }

        delete pkt;

        owner->dmaRow = 0;
        owner->bit_ptr = 0;

        owner->pic_write_ptr =
            owner->base_picAddr +
            owner->_L_block_row_ptr;

        if (!owner->bitSliceEvent.scheduled()) {
            owner->schedule(
                owner->bitSliceEvent,
                owner->clockEdge(Cycles(1)));
        }

        return true;
    }

    panic_if(
        owner->dmaRow >= owner->regArray.size(),
        "P2S_L: dmaRow %u outside regArray",
        owner->dmaRow);

    panic_if(
        pktSize > owner->regArray[owner->dmaRow].size(),
        "P2S_L: DMA beat %zu exceeds row buffer %zu",
        pktSize,
        owner->regArray[owner->dmaRow].size());

    std::memcpy(
        owner->regArray[owner->dmaRow].data(),
        dmaData,
        pktSize);

    delete pkt;

    owner->dmaRow++;

    if (owner->dmaRow == owner->regArray.size()) {
        owner->dmaRow = 0;
        owner->bit_ptr = 0;

        owner->pic_write_ptr =
            owner->base_picAddr +
            owner->_L_block_row_ptr;

        if (!owner->bitSliceEvent.scheduled()) {
            owner->schedule(
                owner->bitSliceEvent,
                owner->clockEdge(Cycles(1)));
        }
    }
    return true;
}

bool
P2S_L::handleRequest(PacketPtr pkt)
{
    panic_if( //assertion for second request
        pendingControlPkt != nullptr,
        "P2S_L received a second control request while one is active");

    pendingControlPkt = pkt;

    datapathDone = false; //fix

    const P2S_L_Payload *payload =
        pkt->getConstPtr<P2S_L_Payload>();

    base_dram_addr = payload->base_dramAddr_to_load;
    base_picAddr = payload->base_picAddr_to_store;
    pic_write_ptr = payload->base_picAddr_to_store;
    next_row_offset_elem = payload->next_row_offset_elem;

    // original expression: (next_row_offset_elem * 8) >> log2Ceil(8)
    next_row_offset_dram = payload->next_row_offset_elem;

    _L_block_row = payload->_L_block_row;
    _L_block_row_ptr = 0;
    next_slice_offset_pic = payload->_L_block_row;
    precision = payload->precision;
    bit_ptr = 0;
    dmaRow = 0;

    schedule(
        dmaReadEvent,
        clockEdge(Cycles(1)));

    return true;
}

void
P2S_L::processDMAReadEvent()
{
    static constexpr uint32_t rowBytes = 64;

    RequestPtr request = std::make_shared<Request>(
        base_dram_addr,
        rowBytes,
        0,
        requestorId);

    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);

    pkt->allocate();

    const bool success = DMAPort.sendTimingReq(pkt);

    if (success) {
        base_dram_addr += next_row_offset_dram;
    }
}

void
P2S_L::processBitSliceEvent()
{
    const uint64_t bitSlice =
        extractBits(regArray, bit_ptr);

    RequestPtr request = std::make_shared<Request>(
        0,
        sizeof(P2SWritePayload),
        0,
        requestorId);

    PacketPtr bitSlicePkt =
        new Packet(request, MemCmd::WriteReq);

    bitSlicePkt->allocate();

    const P2SWritePayload payload{
        pic_write_ptr,
        bitSlice
    };

    bitSlicePkt->setData(
        reinterpret_cast<const uint8_t *>(&payload));

    bitSliceQueue.push_back(bitSlicePkt);

    if (!writeEvent.scheduled()) {
        schedule(
            writeEvent,
            clockEdge(Cycles(1)));
    }

    pic_write_ptr += next_slice_offset_pic;

    bit_ptr++;

    // precision is the highest valid bit index.
    // precision=7 therefore means emit bit0..bit7 (8 bit-slices).
    if (bit_ptr <= precision) { //fix
        schedule(
            bitSliceEvent,
            clockEdge(Cycles(1)));
    }
    else {
        bit_ptr = 0;
        _L_block_row_ptr++;

        if (_L_block_row_ptr < _L_block_row) {
            schedule(
                dmaReadEvent,
                clockEdge(Cycles(1)));
        }
        else {
            datapathDone = true; //fix
        }
    }
}


// Added for test & integration:
// notify Scheduler only after the P2S_L operation has fully completed.
void
P2S_L::completeControlRequest()
{
    panic_if( //assertion: complete but no control packet
        pendingControlPkt == nullptr,
        "%s completion without pending control packet",
        name());

    if (!pendingControlPkt->isResponse()) {
        pendingControlPkt->makeResponse();
    }

    if (instPort.sendPacket(pendingControlPkt)) {
        DPRINTF(
            P2S_L,
            "CONTROL COMPLETE: final CacheBank write accepted\n");

        pendingControlPkt = nullptr;
        datapathDone = false;
    }
}

void
P2S_L::retryControlResponse()
{
    if (pendingControlPkt == nullptr ||
        !pendingControlPkt->isResponse()) {
        return;
    }

    if (instPort.sendPacket(pendingControlPkt)) {
        pendingControlPkt = nullptr;
        datapathDone = false;
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
P2S_L::processWriteEvent()
{
    if (bitSliceQueue.empty()) {
        return;
    }

    PacketPtr pkt = bitSliceQueue.front();

    const bool success = CacheBankPort.sendTimingReq(pkt);

    if (!success) {
        return;
    }

    bitSliceQueue.pop_front();

    // keep p2s active until all queued array-write requests are accepted
    if (!bitSliceQueue.empty()) {
        schedule(
            writeEvent,
            clockEdge(Cycles(1)));

        return;
    }

    if (datapathDone) {
        completeControlRequest();
    }
}

}

