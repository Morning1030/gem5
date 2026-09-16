#include "learning_gem5/PIC/AutoLoadL.hh"
#include "learning_gem5/PIC/Control/scheduler.hh"
#include "sim/system.hh"

#include <algorithm>
#include <cstring>

#include "debug/AutoLoadL.hh"

namespace gem5
{
AutoLoadL::AutoLoadL(const AutoLoadLParams &params) :
ClockedObject(params),
instPort(params.name + "inst_port", this),
cacheBankPort(params.name + "cb_port", this),
requestorId(system.getRequestorId(this, "AutoLoadL")),
pendingReqPkt(nullptr),
loadReqEvent([this]{this->processLoadReqEvent();}, "loadReqEvent"),
recvRespEvent([this]{this->processRecvRespEvent();}, "recvRespEvent"),
respBusy(false)
{}

Port &
AutoLoadL::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port")
        return instPort;

    if (if_name == "cb_port")
        return cacheBankPort;

    return ClockedObject::getPort(if_name, idx);
}
AutoLoadL::CPUSidePort::CPUSidePort(
    const std::string &name,
    AutoLoadL* owner) :
    ResponsePort(name, owner),
    owner(owner),
    blockedPacket(nullptr)
{}

bool
AutoLoadL::CPUSidePort::recvTimingReq(PacketPtr pkt){
    return owner->handleRequest(pkt);
}

void
AutoLoadL::CPUSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    if (sendTimingResp(pkt)) {
        owner->pendingReqPkt = nullptr;
        owner->respBusy = false;
        blockedPacket = nullptr;

        DPRINTF(AutoLoadL, "AutoLoadL: send data to cache bank\n");
        sendRetryReq();
    }
    else blockedPacket = pkt;
}
void
AutoLoadL::CPUSidePort::recvRespRetry()
{
    // retry to send resp to scheduler
    assert(blockedPacket != nullptr);

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
}

AutoLoadL::MemSidePort::MemSidePort(
    const std::string &name,
    AutoLoadL *owner) :
    RequestPort(name, owner),
    owner(owner),
    blockedPacket(nullptr)
{}
bool
AutoLoadL::MemSidePort::recvTimingResp(PacketPtr pkt) {
    return owner->handleResponse(pkt);
}
void
AutoLoadL::MemSidePort::recvReqRetry()
{
    // retry to send resp to scheduler
    assert(blockedPacket != nullptr);

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
}
void
AutoLoadL::MemSidePort::sendPacket(PacketPtr pkt) {
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    if (sendTimingReq(pkt)) {
        blockedPacket = nullptr;

        DPRINTF(AutoLoadL, "AutoLoadL: send data to cache bank\n");
    }
    else blockedPacket = pkt;
}
bool
AutoLoadL::handleRequest(PacketPtr pkt) {
    // busy
    if (pendingReqPkt != nullptr) return false;

    // idle
    pendingReqPkt = pkt;
    schedule(loadReqEvent, clockEdge(Cycles(1)));
    return true;
}
void
AutoLoadL::processLoadReqEvent() {
    const AutoLoadLPayload *autoLoadLPayload = pendingReqPkt->getConstPtr<AutoLoadLPayload>();

    RequestPtr request = std::make_shared<Request>(
        0,                     // TBD
        sizeof(uint64_t),                // next_row_offset_elem, base_dram_addr
        0,                                  // TBD
        requestorId
    );
    // TODO Read Request should not have dataPayload
    PacketPtr reqPkt = new Packet(request, MemCmd::ReadReq);
    reqPkt->allocate();

    reqPkt->setData(reinterpret_cast<const uint8_t*>(&(autoLoadLPayload->data)));
    cacheBankPort->sendPacket(reqPkt);
}
void
AutoLoadL::processRecvRespEvent() {
    pendingReqPkt->makeResponse();

    AutoLoadLPayload payload = *pendingReqPkt->getConstPtr<AutoLoadLPayload>();
    payload.data = dataReadFromBank;

    pendingReqPkt->setData(reinterpret_cast<const uint8_t*>(&payload));
    instPort.sendPacket(pendingReqPkt);
}
bool
AutoLoadL::handleResponse(PacketPtr pkt) {
    assert(pendingReqPkt != nullptr);
    // resp not send back yet
    if (respBusy) return false;
    respBusy = true;

    dataReadFromBank = *(pkt->getConstPtr<uint64_t>());
    delete pkt;
    schedule(recvRespEvent, clockEdge(Cycles(1)));
    return true;
}
}