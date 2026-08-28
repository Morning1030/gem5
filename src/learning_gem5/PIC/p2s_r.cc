#include "learning_gem5/PIC/p2s.hh"
#include <algorithm>
#include <cstring>
#include <memory>
#include "base/logging.hh"
#include "debug/P2S_R.hh"
#include "learning_gem5/PIC/scheduler.hh"
#include "sim/system.hh"

namespace gem5
{

namespace
{
constexpr uint32_t P2SRMaxRows = 64;
constexpr uint32_t P2SRBlockCols = 128;
constexpr uint32_t P2SRWorkingCols = 8;
} // anonymous namespace



P2S_R::P2S_R(const P2S_RParams &params) :
    ClockedObject(params),
    instPort(name() + ".inst_port", this),
    DMAPort(name() + ".dma_port", this, true),
    CacheBankPort(name() + ".cb_port", this, false),
    requestorId(params.system->getRequestorId(this, "P2S_L")),
    wordlineNums(params.wordline_nums),
    dmaReadEvent([this]{ processDMAReadEvent(); }, "dmaReadEvent"),
    loadBufferEvent([this] { processLoadBufferEvent(); }, name() + ".loadBufferEvent"),
    bitSliceEvent([this] { processBitSliceEvent(); }, name() + ".bitSliceEvent"),
    writeEvent([this] { processWriteEvent(); }, name() + ".writeEvent")
{
}

//===== gem5 port glue ===//
P2S_R::CPUSidePort::CPUSidePort(
    const std::string &name,
    P2S_R *owner)
    : ResponsePort(name, owner),
      owner(owner)
{
}


bool
P2S_R::CPUSidePort::sendPacket(PacketPtr pkt)
{
    return sendTimingResp(pkt);
}


void
P2S_R::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    panic("P2S_R functional access is not implemented");
}


bool
P2S_R::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    return owner->handleRequest(pkt);
}


void
P2S_R::CPUSidePort::recvRespRetry()
{
    owner->retryControlResponse();
}


AddrRangeList
P2S_R::CPUSidePort::getAddrRanges() const
{
    return {};
}


P2S_R::MemSidePort::MemSidePort(
    const std::string &name,
    P2S_R *owner,
    bool dmaSide)
    : RequestPort(name, owner),
      owner(owner),
      dmaSide(dmaSide)
{
}


void
P2S_R::MemSidePort::sendPacket(PacketPtr pkt)
{
    sendTimingReq(pkt);
}


bool
P2S_R::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    if (dmaSide) {
        return owner->handleDMAResponse(pkt);
    }

    delete pkt;
    return true;
}

void
P2S_R::MemSidePort::recvReqRetry()
{
    if (dmaSide) {
        owner->retryDMARequest();
        return;
    }

    if (!owner->writeEvent.scheduled() &&
        !owner->bitSliceQueue.empty()) {

        owner->schedule(
            owner->writeEvent,
            owner->clockEdge(Cycles(1)));
    }
}

Port &
P2S_R::getPort(
    const std::string &if_name,
    PortID idx)
{
    if (if_name == "inst_port")
        return instPort;

    if (if_name == "dma_port")
        return DMAPort;

    if (if_name == "cb_port")
        return CacheBankPort;

    return ClockedObject::getPort(
        if_name,
        idx);
}
//===========================//


bool
P2S_R::handleRequest(PacketPtr pkt)
{
    if (active || pendingControlPkt != nullptr) {
        return false;
    }

    pendingControlPkt = pkt;
    datapathDone = false;

    panic_if(
        pkt->getSize() != sizeof(P2S_R_Payload),
        "%s P2S_R expected payload size %zu, got %u",
        name(),
        sizeof(P2S_R_Payload),
        pkt->getSize());

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

    for (uint32_t i = 1; i < arrayID_offset.size(); ++i) {
        arrayID_offset[i] = arrayID_offset[i - 1] + relative_offset_buf[i - 1];
    }


    for (auto &row : mem0) {
        std::fill(row.begin(),row.end(),0);
    }

    for (auto &row : regArray) {
        std::fill(row.begin(),row.end(),0);
    }


    // Restore block/buffer traversal state.
    curBlockColPtrGlobal = 0;

    curBlockNCols =
        std::min<uint32_t>(P2SRBlockCols,nCols);

    curBlockDramBaseAddr = dramAddr;

    dmaRow = 0;

    curBufColPtrInBlock = 0;
    curBufNCols = 0;
    loadBufferRow = 0;

    curEnqBlockInBufColPtr = 0;
    bit_ptr = 0;

    active = true;

    schedule(
        dmaReadEvent,
        clockEdge(Cycles(1)));
    return true;
}


void
P2S_R::get_array_relatice_offset(
    std::vector<uint8_t> &offset,
    uint8_t numBuf)
{
    //fix
    if (numBuf == 3) {
        offset = {
            4, 4, 4, 4, 4, 4, 4
        };
    }
    else if (numBuf == 2) {
        offset = {
            1, 3, 1, 3, 1, 3, 1
        };
    }
    else if (numBuf == 1) {
        offset = {
            1, 1, 2, 1, 1, 2, 1
        };
    }
    else {
        panic(
            "%s P2S_R invalid bufNum=%u",
            name(),
            numBuf);
    }
}


void
P2S_R::processDMAReadEvent()
{
    panic_if(
        !active,
        "%s P2S_R DMA event fired while idle",
        name());

    panic_if(
        blockedDmaPkt != nullptr,
        "%s P2S_R tried to issue a second blocked DMA packet",
        name());

    const Addr rowAddr =
        curBlockDramBaseAddr +
        static_cast<Addr>(dmaRow) * static_cast<Addr>(next_row_offset_bytes);

    RequestPtr request =
        std::make_shared<Request>(rowAddr,curBlockNCols,0,requestorId);

    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);

    pkt->allocate();

    if (!DMAPort.sendTimingReq(pkt)) {
        blockedDmaPkt = pkt;
    }
}


void
P2S_R::retryDMARequest()
{
    panic_if(
        blockedDmaPkt == nullptr,
        "%s P2S_R DMA retry arrived without a blocked packet",
        name());

    if (DMAPort.sendTimingReq(
            blockedDmaPkt)) {

        blockedDmaPkt = nullptr;
    }
}


bool
P2S_R::handleDMAResponse(PacketPtr pkt)
{
    panic_if(
        !active,
        "%s P2S_R received DMA response while idle",
        name());

    panic_if(
        dmaRow >= nRows,
        "%s P2S_R DMA row %u outside nRows=%u",
        name(),
        dmaRow,
        nRows);

    panic_if(
        pkt->getSize() != curBlockNCols,
        "%s P2S_R DMA response size %u != current block width %u",
        name(),
        pkt->getSize(),
        curBlockNCols);

    std::memcpy(mem0[dmaRow].data(),pkt->getConstPtr<uint8_t>(),curBlockNCols);

    delete pkt;

    ++dmaRow;

    if (dmaRow < nRows) {
        schedule(dmaReadEvent,clockEdge(Cycles(1)));
        return true;
    }

    curBufColPtrInBlock = 0;

    curBufNCols = std::min<uint32_t>(P2SRWorkingCols, curBlockNCols);

    loadBufferRow = 0;

    schedule(loadBufferEvent, clockEdge(Cycles(1)));

    return true;
}

void
P2S_R::processLoadBufferEvent()
{
    panic_if(
        loadBufferRow >= nRows,
        "%s P2S_R loadBufferRow=%u outside nRows=%u",
        name(),
        loadBufferRow,
        nRows);

    if (loadBufferRow == 0) {
        for (auto &row : regArray) {
            std::fill(row.begin(),row.end(),0);
        }
    }

    for (uint32_t col = 0;col < curBufNCols;++col) {
        regArray[loadBufferRow][col] = mem0[loadBufferRow][curBufColPtrInBlock + col];
    }

    ++loadBufferRow;

    if (loadBufferRow < nRows) {
        schedule(loadBufferEvent,clockEdge(Cycles(1)));
        return;
    }

    loadBufferRow = 0;
    curEnqBlockInBufColPtr = 0;
    bit_ptr = 0;

    schedule(
        bitSliceEvent,
        clockEdge(Cycles(1)));
}

uint64_t
P2S_R::extractBits_R(
    const std::vector<std::vector<uint8_t>> &arr,
    uint32_t column,
    uint8_t bit,
    uint32_t dim)
{
    panic_if(
        column >= dim,
        "%s P2S_R column=%u outside working-buffer width %u",
        name(),
        column,
        dim);

    uint64_t bitSlice = 0;

    for (uint32_t row = 0;row < P2SRMaxRows; ++row) {

        const uint64_t extractedBit =
            (static_cast<uint64_t>(arr[row][column]) >> bit) & 0x1ULL;

        bitSlice |= extractedBit << row;
    }

    return bitSlice;
}


void
P2S_R::processBitSliceEvent()
{
    panic_if(
        curEnqBlockInBufColPtr >= curBufNCols,
        "%s P2S_R working column %u outside current width %u",
        name(),
        curEnqBlockInBufColPtr,
        curBufNCols);

    const uint64_t bitSlice =
        extractBits_R(
            regArray,
            curEnqBlockInBufColPtr,
            bit_ptr,
            P2SRWorkingCols);

    const uint64_t curArrayID = base_arrayID_to_store + arrayID_offset[bit_ptr];

    const uint64_t arrayAddrEnq =
        curArrayID *
            static_cast<uint64_t>(
                wordlineNums) +
        static_cast<uint64_t>(
            curBlockColPtrGlobal) +
        static_cast<uint64_t>(
            curBufColPtrInBlock) +
        static_cast<uint64_t>(
            curEnqBlockInBufColPtr);

    const P2SWritePayload payload{
        arrayAddrEnq,
        bitSlice
    };

    RequestPtr request =
        std::make_shared<Request>(0,sizeof(P2SWritePayload),0,requestorId);

    PacketPtr bitSlicePkt = new Packet(request,MemCmd::WriteReq);

    bitSlicePkt->allocate();

    bitSlicePkt->setData(
        reinterpret_cast<const uint8_t *>(&payload));

    bitSliceQueue.push_back(bitSlicePkt);

    if (!writeEvent.scheduled()) {
        schedule(writeEvent,clockEdge(Cycles(1)));
    }

    //fix
    if (bit_ptr < precision) {
        ++bit_ptr;

        schedule(
            bitSliceEvent,
            clockEdge(Cycles(1)));

        return;
    }

    bit_ptr = 0;
    ++curEnqBlockInBufColPtr;

    if (curEnqBlockInBufColPtr < curBufNCols) {
        schedule(bitSliceEvent,clockEdge(Cycles(1)));
        return;
    }

    curEnqBlockInBufColPtr = 0;

    curBufColPtrInBlock +=curBufNCols;

    if (curBufColPtrInBlock < curBlockNCols) {

        curBufNCols =
            std::min<uint32_t>(
                P2SRWorkingCols,
                curBlockNCols -
                    curBufColPtrInBlock);

        loadBufferRow = 0;

        schedule(loadBufferEvent,clockEdge(Cycles(1)));

        return;
    }

    curBlockColPtrGlobal += curBlockNCols;

    if (curBlockColPtrGlobal >= nCols) {
        datapathDone = true;
        return;
    }

    curBlockNCols =
        std::min<uint32_t>(
            P2SRBlockCols,
            nCols -
                curBlockColPtrGlobal);

    curBlockDramBaseAddr =
        dramAddr +
        static_cast<uint64_t>(
            curBlockColPtrGlobal);

    dmaRow = 0;

    curBufColPtrInBlock = 0;
    curBufNCols = 0;
    loadBufferRow = 0;

    for (auto &row : mem0) {
        std::fill(row.begin(),row.end(),0);
    }

    schedule(dmaReadEvent,clockEdge(Cycles(1)));
}

void
P2S_R::completeControlRequest()
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
            P2S_R,
            "CONTROL COMPLETE: output queue drained\n");

        pendingControlPkt = nullptr;
        datapathDone = false;
    }
}


void
P2S_R::retryControlResponse()
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
P2S_R::processWriteEvent()
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

} // namespace gem5
