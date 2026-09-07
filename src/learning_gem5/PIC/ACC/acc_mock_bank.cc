#include "learning_gem5/PIC/ACC/acc_mock_bank.hh"

#include <cassert>
#include <cstdint>

#include "base/logging.hh"
#include "mem/packet_access.hh"

namespace gem5
{

AccMockBank::BankSidePort::BankSidePort(
    const std::string &name, AccMockBank *owner)
    : ResponsePort(name, owner), owner(owner)
{
}

Tick
AccMockBank::BankSidePort::recvAtomic(PacketPtr pkt)
{
    panic("%s atomic access unsupported", name());
}

void
AccMockBank::BankSidePort::recvFunctional(PacketPtr pkt)
{
    panic("%s functional access unsupported", name());
}

bool
AccMockBank::BankSidePort::recvTimingReq(PacketPtr pkt)
{
    return owner->handleRequest(pkt);
}

void
AccMockBank::BankSidePort::recvRespRetry()
{
    panic_if(owner->blockedResponse == nullptr,
             "%s received response retry without blocked response",
             name());

    PacketPtr pkt = owner->blockedResponse;
    if (sendTimingResp(pkt)) {
        owner->blockedResponse = nullptr;

        if (!owner->pendingReadResponses.empty() &&
            !owner->responseEvent.scheduled()) {
            owner->schedule(
                owner->responseEvent,
                owner->clockEdge(Cycles(1)));
        }
    }
}

AddrRangeList
AccMockBank::BankSidePort::getAddrRanges() const
{
    return {};
}

AccMockBank::AccMockBank(const AccMockBankParams &params)
    : ClockedObject(params),
      port(name() + ".port", this),
      baseSrcAddr(params.base_src_addr),
      destAddr(params.dest_addr),
      sourceCount(params.source_count),
      rowCount(params.row_count),
      acc32Bit(params.acc32_bit),
      wordlineNums(params.wordline_nums),
      arraysPerMat(params.arrays_per_mat),
      responseLatency(Cycles(params.response_latency)),
      responseEvent([this] { processResponse(); },
                    name() + ".response_event")
{
    panic_if(rowCount == 0, "%s row_count must be non-zero", name());
    panic_if(wordlineNums == 0, "%s wordline_nums must be non-zero", name());
    panic_if(arraysPerMat == 0, "%s arrays_per_mat must be non-zero", name());
    panic_if(params.response_latency == 0,
             "%s response_latency must be >= 1 cycle", name());
}

Port &
AccMockBank::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port") {
        return port;
    }

    return ClockedObject::getPort(if_name, idx);
}

void
AccMockBank::startup()
{
    const uint64_t sourceStride = wordlineNums * arraysPerMat;

    for (uint32_t source = 0; source < sourceCount; ++source) {
        for (uint32_t row = 0; row < rowCount; ++row) {
            const Addr addr = baseSrcAddr + row + source * sourceStride;
            storage[addr] = sourceWord(source, row);
        }
    }

    inform(
        "%s initialized ACC mock bank: sources=%u rows=%u width=%s stride=%llu",
        name(), sourceCount, rowCount, acc32Bit ? "32" : "16",
        static_cast<unsigned long long>(sourceStride));
}

bool
AccMockBank::handleRequest(PacketPtr pkt)
{
    panic_if(pkt->getSize() != sizeof(uint64_t),
             "%s expected 64-bit array access, got %u bytes",
             name(), pkt->getSize());

    const Addr addr = pkt->getAddr();

    if (pkt->isRead()) {
        const auto it = storage.find(addr);
        panic_if(it == storage.end(),
                 "%s ACC read from uninitialized PIC row addr=%#llx",
                 name(), static_cast<unsigned long long>(addr));

        pkt->setLE<uint64_t>(it->second);
        pkt->makeResponse();
        pendingReadResponses.push_back(pkt);

        if (blockedResponse == nullptr && !responseEvent.scheduled()) {
            schedule(responseEvent, clockEdge(responseLatency));
        }

        return true;
    }

    panic_if(!pkt->isWrite(),
             "%s expected ReadReq or WriteReq", name());

    panic_if(writesSeen >= rowCount,
             "%s received extra ACC destination write addr=%#llx",
             name(), static_cast<unsigned long long>(addr));

    const Addr expectedAddr = destAddr + writesSeen;
    const uint64_t got = pkt->getLE<uint64_t>();
    const uint64_t expected = expectedRow(writesSeen);

    panic_if(addr != expectedAddr,
             "%s ACC destination address mismatch row=%u got=%#llx expected=%#llx",
             name(), writesSeen,
             static_cast<unsigned long long>(addr),
             static_cast<unsigned long long>(expectedAddr));

    panic_if(got != expected,
             "%s ACC data mismatch row=%u got=%#llx expected=%#llx",
             name(), writesSeen,
             static_cast<unsigned long long>(got),
             static_cast<unsigned long long>(expected));

    storage[addr] = got;

    inform(
        "%s ACC ROW PASS row=%u addr=%#llx data=%#llx",
        name(), writesSeen,
        static_cast<unsigned long long>(addr),
        static_cast<unsigned long long>(got));

    ++writesSeen;
    delete pkt;

    if (writesSeen == rowCount) {
        inform("%s ACC MOCK BANK FULL PASS: %u rows", name(), rowCount);
    }

    return true;
}

void
AccMockBank::processResponse()
{
    if (blockedResponse != nullptr || pendingReadResponses.empty()) {
        return;
    }

    PacketPtr pkt = pendingReadResponses.front();
    pendingReadResponses.pop_front();

    if (!port.sendTimingResp(pkt)) {
        blockedResponse = pkt;
        return;
    }

    if (!pendingReadResponses.empty()) {
        schedule(responseEvent, clockEdge(Cycles(1)));
    }
}

uint64_t
AccMockBank::sourceWord(uint32_t source, uint32_t row) const
{
    if (acc32Bit) {
        const uint32_t lo =
            1000u * (source + 1u) + 10u * row + 1u;
        const uint32_t hi =
            2000u * (source + 1u) + 10u * row + 2u;

        return (static_cast<uint64_t>(hi) << 32) | lo;
    }

    uint64_t word = 0;
    for (unsigned lane = 0; lane < 4; ++lane) {
        const uint16_t value = static_cast<uint16_t>(
            100u * (source + 1u) + 10u * row + lane + 1u);
        word |= static_cast<uint64_t>(value) << (lane * 16);
    }
    return word;
}

uint64_t
AccMockBank::expectedRow(uint32_t row) const
{
    uint64_t acc = 0;
    for (uint32_t source = 0; source < sourceCount; ++source) {
        const uint64_t data = sourceWord(source, row);
        acc = acc32Bit ? add32(acc, data) : add16(acc, data);
    }
    return acc;
}

uint64_t
AccMockBank::add32(uint64_t acc, uint64_t data) const
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
AccMockBank::add16(uint64_t acc, uint64_t data) const
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
