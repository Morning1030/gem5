#include "learning_gem5/PIC/ACC/accumulator.hh"

#include <cassert>
#include <cstdint>

#include "base/logging.hh"
#include "debug/Accumulator.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"
#include "sim/system.hh"

namespace gem5
{

Accumulator::ControlPort::ControlPort(
    const std::string &name, Accumulator *owner)
    : ResponsePort(name, owner), owner(owner)
{
}

Tick
Accumulator::ControlPort::recvAtomic(PacketPtr pkt)
{
    panic("%s atomic access unsupported", name());
}

void
Accumulator::ControlPort::recvFunctional(PacketPtr pkt)
{
    panic("%s functional access unsupported", name());
}

bool
Accumulator::ControlPort::recvTimingReq(PacketPtr pkt)
{
    return owner->handleControlRequest(pkt);
}

void
Accumulator::ControlPort::recvRespRetry()
{
    assert(blockedResponse != nullptr);

    PacketPtr pkt = blockedResponse;
    if (sendTimingResp(pkt)) {
        blockedResponse = nullptr;
    }
}

AddrRangeList
Accumulator::ControlPort::getAddrRanges() const
{
    return {};
}

void
Accumulator::ControlPort::sendResponse(PacketPtr pkt)
{
    panic_if(blockedResponse != nullptr,
             "%s attempted to queue two blocked control responses",
             name());

    if (!sendTimingResp(pkt)) {
        blockedResponse = pkt;
    }
}

Accumulator::BankPort::BankPort(
    const std::string &name, Accumulator *owner)
    : RequestPort(name, owner), owner(owner)
{
}

bool
Accumulator::BankPort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleBankResponse(pkt);
}

void
Accumulator::BankPort::recvReqRetry()
{
    owner->retryBlockedBankRequest();
}

void
Accumulator::BankPort::recvRangeChange()
{
}

Accumulator::Accumulator(const AccumulatorParams &params)
    : ClockedObject(params),
      instPort(name() + ".inst_port", this),
      bankPort(name() + ".bank_port", this),
      requestorId(params.system->getRequestorId(this, "Accumulator")),
      wordlineNums(params.wordline_nums),
      arraysPerMat(params.arrays_per_mat),
      readIssueEvent([this] { processReadIssue(); },
                     name() + ".read_issue_event"),
      writeIssueEvent([this] { processWriteIssue(); },
                      name() + ".write_issue_event")
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
        return bankPort;
    }

    return ClockedObject::getPort(if_name, idx);
}

bool
Accumulator::handleControlRequest(PacketPtr pkt)
{
    if (state != State::IDLE || pendingControlPkt != nullptr ||
        instPort.responseBlocked()) {
        return false;
    }

    panic_if(!pkt->isWrite(), "%s ACC control packet must be a WriteReq", name());
    panic_if(pkt->getSize() != sizeof(AccRequestPayload),
             "%s ACC control payload size mismatch: got=%u expected=%u",
             name(), pkt->getSize(),
             static_cast<unsigned>(sizeof(AccRequestPayload)));

    const auto *payload = pkt->getConstPtr<AccRequestPayload>();

    panic_if(payload->rowNum == 0,
             "%s ACC rowNum must be non-zero (official RTL assumes row_num >= 1)",
             name());

    pendingControlPkt = pkt;

    baseSrcPicAddr = payload->baseSrcPicAddr;
    destPicAddr = payload->destPicAddr;
    totalRows = payload->rowNum;
    sourceCount = payload->sourceCount;
    acc32Bit = payload->acc32Bit != 0;

    rowPtr = 0;
    accumulatorBuf = 0;
    nextReadSource = 0;
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
        schedule(writeIssueEvent, clockEdge(Cycles(1)));
        state = State::WRITE_BACK;
    } else {
        schedule(readIssueEvent, clockEdge(Cycles(1)));
    }

    return true;
}

uint64_t
Accumulator::sourceAddress(uint32_t sourceIndex) const
{
    const uint64_t sourceStride = wordlineNums * arraysPerMat;
    return baseSrcPicAddr + rowPtr + sourceIndex * sourceStride;
}

void
Accumulator::processReadIssue()
{
    if (state != State::READ || blockedBankPkt != nullptr) {
        return;
    }

    if (nextReadSource >= sourceCount) {
        maybeFinishReads();
        return;
    }

    const Addr addr = sourceAddress(nextReadSource);

    RequestPtr request = std::make_shared<Request>(
        addr,
        sizeof(uint64_t),
        Request::Flags(),
        requestorId);

    PacketPtr pkt = Packet::createRead(request);
    pkt->allocate();

    DPRINTF(Accumulator,
            "ACC READ ISSUE row=%u source=%u addr=%#llx\n",
            rowPtr,
            nextReadSource,
            static_cast<unsigned long long>(addr));

    sendBankPacket(pkt, BankReqKind::READ);
}

void
Accumulator::processWriteIssue()
{
    if (state != State::WRITE_BACK || blockedBankPkt != nullptr) {
        return;
    }

    const Addr addr = destPicAddr + rowPtr;

    RequestPtr request = std::make_shared<Request>(
        addr,
        sizeof(uint64_t),
        Request::Flags(),
        requestorId);

    PacketPtr pkt = Packet::createWrite(request);
    pkt->allocate();
    pkt->setLE<uint64_t>(accumulatorBuf);

    DPRINTF(Accumulator,
            "ACC WRITE ISSUE row=%u addr=%#llx data=%#llx\n",
            rowPtr,
            static_cast<unsigned long long>(addr),
            static_cast<unsigned long long>(accumulatorBuf));

    sendBankPacket(pkt, BankReqKind::WRITE);
}

void
Accumulator::sendBankPacket(PacketPtr pkt, BankReqKind kind)
{
    panic_if(blockedBankPkt != nullptr,
             "%s tried to issue a second blocked bank request",
             name());

    if (!bankPort.sendTimingReq(pkt)) {
        blockedBankPkt = pkt;
        blockedBankKind = kind;
        return;
    }

    bankRequestAccepted(kind);
}

void
Accumulator::retryBlockedBankRequest()
{
    panic_if(blockedBankPkt == nullptr,
             "%s received bank retry without a blocked request",
             name());

    PacketPtr pkt = blockedBankPkt;
    const BankReqKind kind = blockedBankKind;

    if (!bankPort.sendTimingReq(pkt)) {
        return;
    }

    blockedBankPkt = nullptr;
    blockedBankKind = BankReqKind::NONE;
    bankRequestAccepted(kind);
}

void
Accumulator::bankRequestAccepted(BankReqKind kind)
{
    if (kind == BankReqKind::READ) {
        ++nextReadSource;

        if (nextReadSource < sourceCount) {
            if (!readIssueEvent.scheduled()) {
                schedule(readIssueEvent, clockEdge(Cycles(1)));
            }
        } else {
            maybeFinishReads();
        }
        return;
    }

    panic_if(kind != BankReqKind::WRITE,
             "%s accepted invalid bank request kind",
             name());

    ++rowPtr;

    if (rowPtr >= totalRows) {
        finishOperation();
        return;
    }

    beginRow();
}

bool
Accumulator::handleBankResponse(PacketPtr pkt)
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

    maybeFinishReads();
    return true;
}

void
Accumulator::maybeFinishReads()
{
    if (state != State::READ) {
        return;
    }

    if (nextReadSource == sourceCount &&
        readResponses == sourceCount &&
        blockedBankPkt == nullptr) {

        state = State::WRITE_BACK;

        if (!writeIssueEvent.scheduled()) {
            schedule(writeIssueEvent, clockEdge(Cycles(1)));
        }
    }
}

void
Accumulator::beginRow()
{
    accumulatorBuf = 0;
    nextReadSource = 0;
    readResponses = 0;
    state = State::READ;

    if (!readIssueEvent.scheduled()) {
        schedule(readIssueEvent, clockEdge(Cycles(1)));
    }
}

void
Accumulator::finishOperation()
{
    panic_if(pendingControlPkt == nullptr,
             "%s finished ACC without a pending control packet",
             name());

    DPRINTF(Accumulator, "ACC COMPLETE rows=%u\n", totalRows);

    state = State::IDLE;
    accumulatorBuf = 0;
    nextReadSource = 0;
    readResponses = 0;

    PacketPtr pkt = pendingControlPkt;
    pendingControlPkt = nullptr;

    if (!pkt->isResponse()) {
        pkt->makeResponse();
    }

    instPort.sendResponse(pkt);
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
