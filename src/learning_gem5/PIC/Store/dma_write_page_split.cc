#include "learning_gem5/PIC/Store/dma_write_page_split.hh"

#include <algorithm> // std::min

#include "base/logging.hh" // gem5 handle panic, fatal, warn
#include "base/trace.hh" // gem5 handle DPRINTF
#include "debug/DMAWrite.hh" //come up after compile with debug flags
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

namespace
{
constexpr uint64_t pageSize = 4096;
constexpr uint64_t bitlineNum = 64;
}


DMAWritePageSplit::DMAWritePageSplit(const DMAWritePageSplitParams &params) :
    ClockedObject(params),
    instPort(params.name + ".req_side", this),
    tlbPort(params.name + ".tlb_side", this, PICPortID::TLB),
    dramPort(params.name + ".mem_side", this, PICPortID::DRAM),
    requestorId(params.system->getRequestorId(this)),
    pendingReqPkt(nullptr),
    cacheBankData(pageSize + bitlineNum / 8),
    tlbQueryEvent([this]{this->processTlbQueryEvent();}, "tlbQueryEvent"),
    issueWriteEvent([this]{this->processIssueWriteEvent();}, "issueWriteEvent")
{}

Port &
DMAWritePageSplit::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "req_side")
        return instPort;
    if (if_name == "tlb_side")
        return tlbPort;
    if (if_name == "mem_side")
        return dramPort;
    return ClockedObject::getPort(if_name, idx);
}

// ---------------------------------------------------------------------------
// Ports (gem5 only: valid/ready handshakes become timing req/resp + retry)
// ---------------------------------------------------------------------------
DMAWritePageSplit::CPUSidePort::CPUSidePort(
    const std::string &name,
    DMAWritePageSplit *owner) :
    ResponsePort(name),
    owner(owner),
    blockedPacket(nullptr)
{}
// Scala: io.req.ready -- refuse while a row is in progress.
bool
DMAWritePageSplit::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    return owner->handleRequest(pkt);
}

void
DMAWritePageSplit::CPUSidePort::recvRespRetry()
{
    // retry to send resp to scheduler
    assert(blockedPacket != nullptr);

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
}
void
DMAWritePageSplit::CPUSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    if (sendTimingResp(pkt)) {
        owner->pendingReqPkt = nullptr;
        blockedPacket = nullptr;

        DPRINTF(DMAWrite, "DMA write COMPLETE: send back to bank fetch\n");
    }
    else blockedPacket = pkt;
}

DMAWritePageSplit::MemSidePort::MemSidePort(
    const std::string &name,
    DMAWritePageSplit *owner,
    PICPortID picPortID) :
    RequestPort(name),
    owner(owner),
    picPortID(picPortID),
    blockedPacket(nullptr)
{}

bool
DMAWritePageSplit::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(this->picPortID, pkt);
}

void
DMAWritePageSplit::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");
    if (sendTimingReq(pkt)) {
        // to dma
        if (pkt->isWrite()) {
            // if (owner->cacheBankPort.needRetry == true) {
            //     owner->cacheBankPort.sendRetryReq();
            // }
            // DPRINTF(DMAWritePageSplit, "Successfully send request to DMA\n");
        }
    }
    else {
        blockedPacket = pkt;
    }
}

void
DMAWritePageSplit::MemSidePort::recvReqRetry()
{
    panic_if(blockedPacket == nullptr, "There's no blockedPacket to send");

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
    // might fail again, wait for another req retry
}

bool
DMAWritePageSplit::handleRequest(PacketPtr pkt)
{
    if (pendingReqPkt != nullptr) return false;
    pendingReqPkt = pkt;
    // clear the vector for later fill in
    std::fill(cacheBankData.begin(), cacheBankData.end(), 0);

    vaddrPtr = pkt->getAddr();
    validNBytes = pkt->getSize();
    std::memcpy(cacheBankData.data(), pkt->getConstPtr<uint8_t>(), validNBytes);

    DPRINTF(DMAWrite, "row write %#x nByte %u%s\n", vaddrPtr, validNBytes);

    if (!tlbQueryEvent.scheduled()) {
        schedule(tlbQueryEvent, clockEdge(Cycles(1)));
    }
    return true;
}

bool
DMAWritePageSplit::handleResponse(PICPortID picPortID, PacketPtr pkt)
{
    if (picPortID == PICPortID::TLB) {
        // physical address
        paddrPtr = pkt->getLE<uint64_t>();
        // vpn = vaddrPtr / PageBytes;
        // ppn = paddrPtr / PageBytes;
        // pageValid = true;

        DPRINTF(DMAWrite, "TLB vaddrPtr %#x -> paddrPtr %#x\n", vaddrPtr, paddrPtr);

        if (!issueWriteEvent.scheduled()) {
            schedule(issueWriteEvent, clockEdge(Cycles(1)));
        }
    }
    else if (picPortID == PICPortID::DRAM) {
        // move on to the next one
        pendingReqPkt->makeResponse();
        instPort.sendPacket(pendingReqPkt);
    }
    delete pkt;
    return true;
}
void
DMAWritePageSplit::processTlbQueryEvent()
{
    RequestPtr req = std::make_shared<Request>(
        vaddrPtr, 
        8,          // to put the physical address
        0, 
        requestorId
    );
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    pkt->allocate();

    tlbPort.sendPacket(pkt);
}
void
DMAWritePageSplit::processIssueWriteEvent()
{
    RequestPtr req = std::make_shared<Request>(
        paddrPtr,
        validNBytes,
        0,
        requestorId
    );
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();
    pkt->setData(cacheBankData.data());

    DPRINTF(DMAWrite, "write paddrPtr %#x size %u \n", paddrPtr, validNBytes);
    dramPort.sendPacket(pkt);
}


}  // namespace gem5
