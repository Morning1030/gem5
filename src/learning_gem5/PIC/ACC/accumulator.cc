#include "learning_gem5/PIC/ACC/accumulator.hh"
#include "learning_gem5/PIC/Control/scheduler.hh"

#include <cassert>
#include <cstdint>

#include "base/logging.hh"
#include "debug/Accumulator.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"
#include "sim/system.hh"

namespace gem5
{
Accumulator::Accumulator(const AccumulatorParams &params)
    : ClockedObject(params),
      instPort(name() + ".inst_port", this),
      cacheBankPort(name() + ".bank_port", this),
      requestorId(params.system->getRequestorId(this, "Accumulator")),
      wordlineNums(params.wordline_nums),
      arraysPerMat(params.arrays_per_mat),
      readEvent([this] { processReadEvent(); },".read_event"),
      writeEvent([this] { processWriteEvent(); }, "write_event")
{
    panic_if(wordlineNums == 0, "%s wordline_nums must be non-zero", name());
    panic_if(arraysPerMat == 0, "%s arrays_per_mat must be non-zero", name());
}

Port &
Accumulator::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port") {
        return instPort;
    }
    if (if_name == "bank_port") {
        return cacheBankPort;
    }

    return ClockedObject::getPort(if_name, idx);
}

Accumulator::CPUSidePort::CPUSidePort(
    const std::string &name, Accumulator *owner)
    : ResponsePort(name), owner(owner), blockedPacket(nullptr)
{}

bool
Accumulator::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    return owner->handleRequest(pkt);
}
void
Accumulator::CPUSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr,
             "%s attempted to queue two blocked control responses",
             name());

    if (sendTimingResp(pkt)) {
        owner->pendingReqPkt = nullptr;
        blockedPacket = nullptr;
    }
    else blockedPacket = pkt;
}

void
Accumulator::CPUSidePort::recvRespRetry()
{
    panic_if(blockedPacket == nullptr, "Should never try to send if blocked!");

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
    
}

Accumulator::MemSidePort::MemSidePort(
    const std::string &name, Accumulator *owner)
    : RequestPort(name), owner(owner), blockedPacket(nullptr)
{}

bool
Accumulator::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(pkt);
}
void
Accumulator::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr,
        "%s tried to issue a second blocked bank request",
        name());

    if (sendTimingReq(pkt)) {
        blockedPacket = nullptr;
        if (pkt->isRead()) {
            owner->schedule(owner->readEvent, owner->clockEdge(Cycles(1)));
        }
        else if (pkt->isWrite()) {
            owner->schedule(owner->writeEvent, owner->clockEdge(Cycles(1)));
        }
    }
    else {
        blockedPacket = pkt;
    }
}
void
Accumulator::MemSidePort::recvReqRetry()
{
    // retry to send resp to scheduler
    panic_if(blockedPacket == nullptr, "There's no blockedPacket!\n");

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
}

bool
Accumulator::handleRequest(PacketPtr pkt)
{
    if (state != State::IDLE || pendingReqPkt != nullptr ||
        instPort.responseBlocked()) {
        return false;
    }

    panic_if(!pkt->isWrite(), "%s ACC control packet must be a WriteReq", name());
    panic_if(pkt->getSize() != sizeof(AccPayload),
             "%s ACC control payload size mismatch: got=%u expected=%u",
             name(), pkt->getSize(),
             static_cast<unsigned>(sizeof(AccPayload)));

    const AccPayload *payload = pkt->getConstPtr<AccPayload>();

    panic_if(payload->row_num == 0,
             "%s ACC row_num must be non-zero (official RTL assumes row_num >= 1)",
             name());

    pendingReqPkt = pkt;

    baseSrcPicAddr = payload->base_src_picAddr;
    destPicAddr = payload->dest_picAddr;
    totalRows = payload->row_num;
    DPRINTF(Accumulator, "ACC START row_num: %u\n", payload->row_num);
    sourceCount = payload->src_arrayNum;
    acc32Bit = (payload->bitWidth != 0);

    rowPtr = 0;
    accumulatorBuf = 0;
    readSourcePtr = 0;
    readResponses = 0;
    state = State::READ;

    DPRINTF(Accumulator,
            "ACC START baseSrc=%#llx dest=%#llx sources=%u rows=%u width=%s\n",
            static_cast<unsigned long long>(baseSrcPicAddr),
            static_cast<unsigned long long>(destPicAddr),
            static_cast<unsigned>(sourceCount),
            totalRows,
            acc32Bit ? "32" : "16");

    if (sourceCount == 0) {
        schedule(writeEvent, clockEdge(Cycles(1)));
        state = State::WRITE_BACK;
    }
    else {
        schedule(readEvent, clockEdge(Cycles(1)));
    }

    return true;
}



void
Accumulator::processReadEvent()
{
    // if last round send packet is blocked then don't continue
    if (state != State::READ || cacheBankPort.blockedPacket != nullptr) {
        return;
    }

    const Addr addr = sourceAddress(readSourcePtr);

    RequestPtr request = std::make_shared<Request>(
        addr,
        sizeof(uint64_t),
        Request::Flags(),
        requestorId);

    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);
    pkt->allocate();

    DPRINTF(Accumulator,
            "ACC READ REQ row=%u source=%u addr=%#llx\n",
            rowPtr,
            readSourcePtr,
            static_cast<unsigned long long>(addr));

    cacheBankPort.sendPacket(pkt);

    readSourcePtr++;

    if (readSourcePtr < sourceCount) {
        if (!readEvent.scheduled()) {
            schedule(readEvent, clockEdge(Cycles(1)));
        }
    }
}

void
Accumulator::processWriteEvent()
{
    if (state != State::WRITE_BACK || cacheBankPort.blockedPacket != nullptr) {
        return;
    }

    const Addr addr = destPicAddr + rowPtr;

    RequestPtr request = std::make_shared<Request>(
        addr,
        sizeof(uint64_t),
        Request::Flags(),
        requestorId);

    PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
    pkt->allocate();
    pkt->setLE<uint64_t>(accumulatorBuf);

    DPRINTF(Accumulator,
            "ACC WRITE ISSUE row=%u addr=%#llx data=%#llx\n",
            rowPtr,
            static_cast<unsigned long long>(addr),
            static_cast<unsigned long long>(accumulatorBuf));

    cacheBankPort.sendPacket(pkt);
    rowPtr++;

    if (rowPtr >= totalRows) {
        panic_if(pendingReqPkt == nullptr,
             "%s finished ACC without a pending control packet",
             name());
        panic_if(pendingReqPkt->isResponse(), "Should be request packet");

        DPRINTF(Accumulator, "ACC COMPLETE rows=%u\n", totalRows);

        state = State::IDLE;
        accumulatorBuf = 0;
        readSourcePtr = 0;
        readResponses = 0;

        pendingReqPkt->makeResponse();
        instPort.sendPacket(pendingReqPkt);
    }
    else {
        accumulatorBuf = 0;
        readSourcePtr = 0;
        readResponses = 0;
        state = State::READ;

        if (!readEvent.scheduled()) {
            schedule(readEvent, clockEdge(Cycles(1)));
        }
    }
}

bool
Accumulator::handleResponse(PacketPtr pkt)
{
    if (!pkt->isRead()) {
        delete pkt;
        return true;
    }

    panic_if(state != State::READ,
             "%s received a bank ReadResp outside READ state",
             name());
    panic_if(pkt->getSize() != sizeof(uint64_t),
             "%s expected 64-bit bank ReadResp, got %u bytes",
             name(), pkt->getSize());

    const uint64_t data = pkt->getLE<uint64_t>();
    delete pkt;

    accumulatorBuf = acc32Bit ?
        add32(accumulatorBuf, data) :
        add16(accumulatorBuf, data);

    ++readResponses;

    DPRINTF(Accumulator,
            "ACC READ RESP row=%u received=%u/%u acc=%#llx\n",
            rowPtr,
            readResponses,
            static_cast<unsigned>(sourceCount),
            static_cast<unsigned long long>(accumulatorBuf));

    if (readSourcePtr == sourceCount &&
        readResponses == sourceCount) {

        state = State::WRITE_BACK;

        if (!writeEvent.scheduled()) {
            schedule(writeEvent, clockEdge(Cycles(1)));
        }
    }
    return true;
}

uint64_t
Accumulator::add32(uint64_t acc, uint64_t data) const
{
    const uint32_t accLo = static_cast<uint32_t>(acc);
    const uint32_t accHi = static_cast<uint32_t>(acc >> 32);
    const uint32_t dataLo = static_cast<uint32_t>(data);
    const uint32_t dataHi = static_cast<uint32_t>(data >> 32);

    const uint32_t sumLo = static_cast<uint32_t>(
        static_cast<uint64_t>(accLo) + dataLo);
    const uint32_t sumHi = static_cast<uint32_t>(
        static_cast<uint64_t>(accHi) + dataHi);

    return (static_cast<uint64_t>(sumHi) << 32) |
           static_cast<uint64_t>(sumLo);
}

uint64_t
Accumulator::add16(uint64_t acc, uint64_t data) const
{
    uint64_t result = 0;

    for (unsigned lane = 0; lane < 4; ++lane) {
        const unsigned shift = lane * 16;
        const uint16_t accLane = static_cast<uint16_t>(acc >> shift);
        const uint16_t dataLane = static_cast<uint16_t>(data >> shift);
        const uint16_t sumLane = static_cast<uint16_t>(
            static_cast<uint32_t>(accLane) + dataLane);

        result |= static_cast<uint64_t>(sumLane) << shift;
    }

    return result;
}

} // namespace gem5
