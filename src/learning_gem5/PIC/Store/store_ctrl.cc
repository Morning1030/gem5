#include "learning_gem5/PIC/Store/store_ctrl.hh"
#include "learning_gem5/PIC/Control/scheduler.hh"
#include "sim/system.hh"

#include <algorithm>
#include <cstring>

#include "debug/StoreCtrl.hh"
namespace gem5
{
StoreCtrl::StoreCtrl(const StoreCtrlParams &params) :
    ClockedObject(params),
    instPort(params.name + ".inst_port", this),
    bankFetchPort(params.name + ".bank_fetch_port", this),
    requestorId(params.system->getRequestorId(this, "StoreCtrl")),
    pendingReqPkt(nullptr),
    one_shot(false),
    row_store_ptr(0),
    bankFetchEvent([this]{this->processBankFetchEvent();}, "bankFetchEvent")
{}
Port&
StoreCtrl::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port")
        return instPort;
    if (if_name == "bank_fetch_port")
        return bankFetchPort;
    return ClockedObject::getPort(if_name, idx);
}
StoreCtrl::CPUSidePort::CPUSidePort(
    const std::string &name,
    StoreCtrl *owner) :
    ResponsePort(name),
    owner(owner),
    blockedPacket(nullptr)
{}
bool
StoreCtrl::CPUSidePort::recvTimingReq(PacketPtr pkt) {
    return owner->handleRequest(pkt);
}
void
StoreCtrl::CPUSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    if (sendTimingResp(pkt)) {
        owner->pendingReqPkt = nullptr;
        blockedPacket = nullptr;

        DPRINTF(StoreCtrl, "Store COMPLETE: send back to scheduler\n");
    }
    else blockedPacket = pkt;
}
void
StoreCtrl::CPUSidePort::recvRespRetry()
{
    // retry to send resp to scheduler
    assert(blockedPacket != nullptr);

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
}

StoreCtrl::MemSidePort::MemSidePort(
    const std::string &name,
    StoreCtrl *owner) :
    RequestPort(name),
    owner(owner),
    blockedPacket(nullptr),
    needRetry(false)
{}
bool
StoreCtrl::MemSidePort::recvTimingResp(PacketPtr pkt) {
    return owner->handleResponse(pkt);
}
void
StoreCtrl::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");
    if (sendTimingReq(pkt)) {
        // to cache bank
        if(pkt->isRead()) {
            // DPRINTF(StoreCtrl, "Successfully send request to bank fetch\n");
        }
    }
    else {
        blockedPacket = pkt;
    }
}
void
StoreCtrl::MemSidePort::recvReqRetry()
{
    panic_if(blockedPacket == nullptr, "There's no blockedPacket to send");

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
    // might fail again, wait for another req retry
}
bool
StoreCtrl::handleRequest(PacketPtr pkt)
{
    
    if (pendingReqPkt != nullptr) return false;
    
    pendingReqPkt = pkt;

    const LSPayload* lsPayload= pkt->getConstPtr<LSPayload>();
    src = (lsPayload->src * 8);  // row index shift 3 bits
    dst = lsPayload->dst;
    row = lsPayload->row;                   
    byte_per_row = lsPayload->byte_per_row;
    offset = lsPayload->offset;
    row_store_ptr = 0;

    one_shot = (byte_per_row == offset);

    if (one_shot) {
        fetchSize = row * byte_per_row;
        row = 1U;
    }
    else {
        fetchSize = byte_per_row;
    }
    
    if (!bankFetchEvent.scheduled()) {
        schedule(bankFetchEvent, clockEdge(Cycles(1)));
    }
    return true;
}
bool
StoreCtrl::handleResponse(PacketPtr pkt) {
    // means this bankfetch->dma_write_page_split is done
    delete pkt;

    row_store_ptr++;
    if (row_store_ptr == row) {
        // store done
        pendingReqPkt->makeResponse();
        instPort.sendPacket(pendingReqPkt);
    }
    else {
        src += byte_per_row;
        dst += offset;
        schedule(bankFetchEvent, clockEdge(Cycles(1)));
    }    

    return true;
}
void
StoreCtrl::processBankFetchEvent()
{
    // prepare request to bank fetch
    RequestPtr request = std::make_shared<Request>(
        src,
        fetchSize,
        0,
        requestorId
    );
    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);
    pkt->allocate();

    BankFetchReq bankFetchReq{src, dst, fetchSize};
    pkt->pushSenderState(new BankFetchSenderState(bankFetchReq));
    bankFetchPort.sendPacket(pkt);
    
    DPRINTF(StoreCtrl, "Send bankFetch Request to %llx, requesting %u bytes\n", src, fetchSize);
    
}
}