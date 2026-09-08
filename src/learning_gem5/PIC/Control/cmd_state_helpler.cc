#include "learning_gem5/PIC/Control/cmd_state_helper.hh"
#include "sim/system.hh"

#include <algorithm>
#include <cstring>

#include "debug/cmd_state_helper.hh"

namespace gem5
{

CmdStateHelper::CmdStateHelper(const CmdStateHelperParams &params) :
ClockedObject(params),
instPort(params.name + ".inst_port", this),
client_num(QryTabClient::total_client),
req_cmdID(0),
pendingReqPkt(nullptr),
RRArbiterLastChoose(0),
arbiterEvent([this]{this->processArbiterEvent();}, "ArbiterEvent"),
initEvent([this]{this->processInitEvent();}, "InitEvent"),
setFinishEvent([this]{this->processSetFinishEvent();}, "setFinishEvent"),
checkFinishEvent([this]{this->processCheckFinishEvent();}, "checkFinishEvent"),
setInvalidEvent([this]{this->processSetInvalidEvent();}, "setInvalidEvent")
{}

Port&
CmdStateHelper::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port") {
        return instPort;
    }
    return ClockedObject::getPort(if_name, idx);
}

CmdStateHelper::CPUSidePort::CPUSidePort(
    const std::string& name,
    CmdStateHelper *owner) :
    ResponsePort(name, owner),
    owner(owner),
    blockedPacket(nullptr)
{}

bool
CmdStateHelper::CPUSidePort::recvTimingReq(PacketPtr pkt) {
    return owner->handleRequest(pkt);
}
void
CmdStateHelper::CPUSidePort::sendPacket(PacketPtr pkt)
{
    // send p2s done to scheduler
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    if (sendTimingResp(pkt)) {
        owner->pendingReqPkt = nullptr;
        blockedPacket = nullptr;

        DPRINTF(QUERY, "QUERY COMPLETE: send back to scheduler\n");
    }
    else blockedPacket = pkt;
}
void
CmdStateHelper::CPUSidePort::recvRespRetry()
{
    // retry to send resp to scheduler
    assert(blockedPacket != nullptr);

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
}
bool
CmdStateHelper::handleRequest(PacketPtr pkt) {
    const QueryPayload *queryPayload = pkt->getConstPtr<QueryPayload>();
    if (query_req[queryPayload->clientID].size() >= maxQueryReqSize) {
        return false;
    }
    else query_req[queryPayload->clientID].push_back(pkt);
    DPRINTF(CmdStateHelper, "Got request from %d", queryPayload->clientID);

    // prevent from scheduling every cycle
    if (!arbiterEvent.scheduled() && !initEvent.scheduled() &&
                  !setFinishEvent.scheduled() && !checkFinishEvent.scheduled() &&
                  !setInvalidEvent.scheduled()) { 
        schedule(arbiterEvent, clockEdge(Cycles(1)));
    }
    return true;    
}
void
CmdStateHelper::processArbiterEvent() {
    if (query_req.empty()) return;

    int chosen_id = -1;

    for (int i = 0; i < client_num; ++i) {
        int idx = (RRArbiterLastChoose + i) % client_num;
        if (!query_req[idx].empty()) {
            chosen_id = idx;
            break;
        }
    }
    if (chosen_id == -1) return;
    pendingReqPkt = query_req[chosen_id].front();
    const QueryPayload *qp = pendingReqPkt->getConstPtr<QueryPayload>();
    query_req[chosen_id].pop_front();
    RRArbiterLastChoose = (chosen_id + 1) % client_num;
    
    req_cmdID = qp->cmdID;
    if (chosen_id == QryTabClient::READER) {
        schedule(checkFinishEvent, clockEdge(Cycles(1)));
    }
    else {
        if (qp->is_finish) schedule(setFinishEvent, clockEdge(Cycles(1)));
        else schedule(initEvent, clockEdge(Cycles(1)));
    }
}
void
CmdStateHelper::processInitEvent() {
    cmd_state_table[req_cmdID].VALID = true;
    cmd_state_table[req_cmdID].FINISH = false;
    schedule(arbiterEvent, clockEdge(Cycles(1)));
}
void
CmdStateHelper::processSetFinishEvent() {
    cmd_state_table[req_cmdID].VALID = true;
    cmd_state_table[req_cmdID].FINISH = true;

    // just ACK resp
    pendingReqPkt->makeResponse();
    instPort.sendPacket(pendingReqPkt);
    schedule(arbiterEvent, clockEdge(Cycles(1)));
}
void
CmdStateHelper::processCheckFinishEvent() {
    // Resp includes correct information
    pendingReqPkt->makeResponse();

    pendingReqPkt->getPtr<QueryPayload>()->is_finish = cmd_state_table[req_cmdID].FINISH;
    instPort.sendPacket(pendingReqPkt);

    // cmd finiished
    if (cmd_state_table[req_cmdID].FINISH)
        schedule(setInvalidEvent, clockEdge(Cycles(1)));
    // not yet finish
    else schedule(arbiterEvent, clockEdge(Cycles(1)));
}
void
CmdStateHelper::processSetInvalidEvent() {
    cmd_state_table[req_cmdID].VALID = false;
    schedule(arbiterEvent, clockEdge(Cycles(1)));
    // assert(table_read_out_wire.VALID === false.B,"The cmdID is inited!")
}
}