#include "learning_gem5/PIC/p2s.hh"
#include "learning_gem5/PIC/scheduler.hh"
#include "sim/system.hh"
#include <cstring>
#include "debug/P2S_R_T.hh"

namespace gem5
{
P2S_R_T::P2S_R_T(const P2S_R_TParams &params) :
    ClockedObject(params),
    instPort(name() + ".inst_port", this),
    DMAPort(name() + ".dma_port", this, true),
    CacheBankPort(name() + ".cb_port", this, false),
    requestorId(params.system->getRequestorId(this, "P2S_R_T")),
    wordlineNums(params.wordline_nums),
    dmaReadEvent([this]{ processDMAReadEvent(); }, "dmaReadEvent"),
    bitSliceEvent([this]{ processBitSliceEvent(); }, "bitSliceEvent"),
    writeEvent([this]{ processWriteEvent(); }, "writeBankEvent")
{
}

//===== gem5 port glue =====//
P2S_R_T::CPUSidePort::CPUSidePort(
    const std::string &name,
    P2S_R_T *owner) :
    ResponsePort(name, owner),
    owner(owner)
{}

bool
P2S_R_T::CPUSidePort::sendPacket(PacketPtr pkt)
{
    return sendTimingResp(pkt);
}

void
P2S_R_T::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    panic("P2S_R_T functional access is not implemented");
}

void
P2S_R_T::CPUSidePort::recvRespRetry()
{
    owner->retryControlResponse();
}

AddrRangeList
P2S_R_T::CPUSidePort::getAddrRanges() const
{
    return {};
}

P2S_R_T::MemSidePort::MemSidePort(
    const std::string &name,
    P2S_R_T *owner,
    bool dmaSide) :
    RequestPort(name, owner),
    owner(owner),
    dmaSide(dmaSide)
{}

void
P2S_R_T::MemSidePort::sendPacket(PacketPtr pkt)
{
    sendTimingReq(pkt);
}

void
P2S_R_T::MemSidePort::recvReqRetry()
{
    if (!dmaSide) {
        if (!owner->writeEvent.scheduled() && !owner->bitSliceQueue.empty()) {
            owner->schedule(owner->writeEvent, owner->clockEdge(Cycles(1)));
        }
        return;
    }
    // dmaSide == true: no retry-holding mechanism exists yet on this path
    // (processDMAReadEvent()'s "need to retry" comment below has no actual
    // implementation) -- pre-existing gap, left as-is; out of scope for
    // the CB/arbiter connection.
}

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
//===========================//

bool
P2S_R_T::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    return owner->handleRequest(pkt);
}

bool
P2S_R_T::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    if (!dmaSide) {
        // AccessBankArb's ack: fires at grant, not bank-commit (see its
        // header comment), carries no payload -- just free the packet.
        // The write queue resumes via recvReqRetry() once refused, or via
        // processWriteEvent()'s own self-reschedule when not.
        delete pkt;
        return true;
    }

    const uint8_t *dmaData = pkt->getConstPtr<uint8_t>();
    const size_t pktSize = pkt->getSize();

    panic_if(
        pktSize != owner->nCols,
        "P2S_R_T: DMA row response size %zu != requested nCols %u",
        pktSize,
        owner->nCols);

    panic_if(
        pktSize > owner->bufArrayOutReFormat.size(),
        "P2S_R_T: DMA row size %zu exceeds P2S_R_T buffer capacity %zu",
        pktSize,
        owner->bufArrayOutReFormat.size());

    std::memcpy(owner->bufArrayOutReFormat.data(),dmaData,pktSize);

    delete pkt;

    owner->bit_ptr = 0;
    owner->schedule(owner->bitSliceEvent,owner->clockEdge(Cycles(1)));

    return true;
}

bool
P2S_R_T::handleRequest(PacketPtr pkt)
{
    if (active || pendingControlPkt != nullptr) {
        return false;
    }

    pendingControlPkt = pkt;
    datapathDone = false;
    active = true;

    const P2S_R_Payload *payload = pkt->getConstPtr<P2S_R_Payload>();

    dramAddr = payload->dramAddr;
    base_arrayID_to_store = payload->base_arrayID_to_store;
    next_row_offset_bytes = payload->next_row_offset_bytes;
    nRows = payload->nRows;
    nCols = payload->nCols;
    precision = payload->precision;
    bufNum = payload->bufNum;

    get_array_relatice_offset(relative_offset_buf, bufNum);

    arrayID_offset[0] = 0;
    for (int i = 1; i < 8; ++i) {
        arrayID_offset[i] = arrayID_offset[i-1] + relative_offset_buf[i-1];
    }

    bit_ptr = 0;
    row_store_ptr = 0;

    schedule(dmaReadEvent,clockEdge(Cycles(1)));
    return true;
}

void
P2S_R_T::processDMAReadEvent()
{
    panic_if(
        nCols == 0,
        "P2S_R_T: nCols must be non-zero");

    RequestPtr request = std::make_shared<Request>(
        dramAddr,
        nCols,
        0,
        requestorId);

    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);

    pkt->allocate();

    const bool success = DMAPort.sendTimingReq(pkt);

    if (success) {
        dramAddr += next_row_offset_bytes;
    }
    else {
        // need to retry
    }
}

void
P2S_R_T::processBitSliceEvent()
{
    const uint64_t bitSlice = extractBits_R_T(bufArrayOutReFormat,bit_ptr);

    const uint64_t curArrayID = base_arrayID_to_store + arrayID_offset[bit_ptr];

    // Equivalent to original:
    // (curArrayID << log2Ceil(coreCfg.wordlineNums)) + row_store_ptr
    const uint64_t arrayAddrEnq = curArrayID * wordlineNums + row_store_ptr;

    RequestPtr request = std::make_shared<Request>(
        0,
        sizeof(P2SWritePayload),
        0,
        requestorId);

    PacketPtr bitSlicePkt = new Packet(request, MemCmd::WriteReq);

    bitSlicePkt->allocate();

    const P2SWritePayload payload{
        arrayAddrEnq,
        bitSlice
    };

    bitSlicePkt->setData(reinterpret_cast<const uint8_t *>(&payload));
    bitSliceQueue.push_back(bitSlicePkt);

    if (!writeEvent.scheduled()) {
        schedule(writeEvent,clockEdge(Cycles(1)));
    }

    const bool lastBit = (bit_ptr == precision);

    if (!lastBit) {
        bit_ptr++;

        schedule(bitSliceEvent,clockEdge(Cycles(1)));
    }
    else {
        bit_ptr = 0;
        row_store_ptr++;

        if (row_store_ptr < nRows) {
            schedule(dmaReadEvent,clockEdge(Cycles(1)));
        }
        else {
            datapathDone = true;
        }
    }
}


// Added for test & integration:
// notify Scheduler only after the P2S_RT operation has fully completed.
void
P2S_R_T::completeControlRequest()
{
    panic_if(
        pendingControlPkt == nullptr,
        "%s completion without pending control packet",
        name());

    if (!pendingControlPkt->isResponse()) {
        pendingControlPkt->makeResponse();
    }

    if (instPort.sendPacket(pendingControlPkt)) {
        DPRINTF(
            P2S_R_T,
            "CONTROL COMPLETE: output queue drained\n");

        pendingControlPkt = nullptr;
        datapathDone = false;
    }
}

void
P2S_R_T::retryControlResponse()
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


void
P2S_R_T::processWriteEvent()
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

    // keep p2s active until all queued array-write requests are accepted.
    if (!bitSliceQueue.empty()) {
        schedule(writeEvent,clockEdge(Cycles(1)));
        return;
    }

    if (datapathDone) {
        active = false;
        completeControlRequest();
    }
}

uint64_t
P2S_R_T::extractBits_R_T(const std::vector<uint8_t> &buf, uint8_t bit) {
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
P2S_R_T::get_array_relatice_offset(
    std::vector<uint8_t> &offset,
    uint8_t numBuf)
{

    //fix
    if (numBuf == 3) {
        offset = {4, 4, 4, 4, 4, 4, 4};
    }
    else if (numBuf == 2) {
        offset = {1, 3, 1, 3, 1, 3, 1};
    } 
    else if (numBuf == 1) {
        offset = {1, 1, 2, 1, 1, 2, 1};
    } 
    else {
        panic("P2S_R_T: invalid bufNum=%u", numBuf);
    }
}
}