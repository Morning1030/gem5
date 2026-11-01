#include "src/learning_gem5/PIC/Store/store_ctrl.hh"
#include "sim/system.hh"

#include <algorithm>
#include <cstring>

#include "debug/StoreCtrl.hh"
namespace gem5
{
StoreCtrl::StoreCtrl(const StoreCtrlParams &params) :
    ClockedObject(params),
    instPort(params.name + ".inst_port", this),
    dmaPort(params.name + ".dma_port", this),
    bankFetchPort(params.name + ".bank_fetch_port", this),
    requestorId(params.system->getRequestorId(this, "StoreCtrl")),
    pendingReqPkt(nullptr),
    storeDone(true),
    one_shot(false),
    row_store_ptr(0),
    bankFetchEvent([this]{this->processBankFetchEvent();}, "bankFetchEvent"),
    dmaWriteEvent([this]{this->processDMAWriteEvent();}, "dmaWriteEvent")
{}
Port&
StoreCtrl::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port")
        return instPort;
    if (if_name == "dma_port")
        return dmaPort;
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
    StoreCtrl *owner,
    PICPortID picPortID) :
    RequestPort(name),
    owner(owner),
    picPortID(picPortID),
    blockedPacket(nullptr),
    needRetry(false)
{}
bool
StoreCtrl::MemSidePort::recvTimingResp(PacketPtr pkt) {
    return owner->handleResponse(this->picPortID, pkt);
}
void
StoreCtrl::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");
    if (sendTimingReq(pkt)) {
        // to dma
        if (pkt->isWrite()) {
            if (owner->bankFetchPort.needRetry == true) {
                owner->bankFetchPort.sendRetryReq();
            }
            // DPRINTF(StoreCtrl, "Successfully send request to DMA\n");
        }
        // to cache bank
        else if(pkt->isRead()) {
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
    
    if (pendingReqPkt != nullptr || storeDone == false) return false;
    
    pendingReqPkt = pkt;
    storeDone = false;

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
StoreCtrl::handleResponse(PICPortID picPortID, PacketPtr pkt) {
    if (picPortID == PICPortID::DMA) {
        delete pkt;

        // the whole thing is done
        if (storeDone) {
            pendingReqPkt->makeResponse();
            instPort.sendPacket(pendingReqPkt);
        } 
        return true;
    }
    else if (picPortID == PICPortID::BANKFETCH) {
        // reject new data when dma is busy
        if (dmaPort.blockedPacket != nullptr) {
            bankFetchPort.needRetry = true;
            return false;
        }

        dataSize = pkt->getSize();
        const uint8_t* p = pkt->getConstPtr<uint8_t>();
        bankFetchPayload.assign(p, p + pkt->getSize());
        delete pkt;

        if (!dmaWriteEvent.scheduled()) {
            schedule(dmaWriteEvent, clockEdge(Cycles(1)));
        }
        return true;
    }
    else {
        DPRINTF(StoreCtrl, "Cannot recognize port id\n");
        return false;
    }
}
void
StoreCtrl::processBankFetchEvent()
{
    
    RequestPtr request = std::make_shared<Request>(
        src,
        fetchSize,
        0,
        requestorId
    );
    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);
    pkt->allocate();

    bankFetchPort.sendPacket(pkt);
    DPRINTF(StoreCtrl, "Send bankFetch Request to %llx, requesting %u bytes\n", src, fetchSize);
    
}
void
StoreCtrl::processDMAWriteEvent()
{
    RequestPtr request = std::make_shared<Request>(
        dst,
        dataSize,
        0,
        requestorId
    );
        
    PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
    pkt->allocate();

    pkt->setData(reinterpret_cast<uint8_t*>(bankFetchPayload.data()));
    dmaPort.sendPacket(pkt);
    row_store_ptr++;

    if (row_store_ptr == row) {
        // store done
        storeDone = true;
    }
    else {
        src += byte_per_row;
        dst += offset;
        schedule(bankFetchEvent, clockEdge(Cycles(1)));
    }    
}
}