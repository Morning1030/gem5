#include "learning_gem5/PIC/Control/cmd_state_helper.hh"
#include "learning_gem5/PIC/Control/scheduler.hh"
#include "sim/system.hh"

#include <algorithm>
#include <cstring>

#include "debug/CmdStateHelper.hh"
#include "debug/QUERY.hh"

namespace gem5
{

CmdStateHelper::CmdStateHelper(const CmdStateHelperParams &params) :
ClockedObject(params),
instPort(params.name + ".inst_port", this),
client_num(static_cast<uint8_t>(QryTabClient::total_client)),
cmd_state_table(CmdIdMax),
req_cmdID(0),
query_req(static_cast<size_t>(QryTabClient::total_client)),
pendingReqPkt(nullptr),
RRArbiterLastChoose(0),
arbiterEvent([this]{this->processArbiterEvent();}, "ArbiterEvent"),
initEvent([this]{this->processInitEvent();}, "InitEvent"),
setFinishEvent([this]{this->processSetFinishEvent();}, "setFinishEvent"),
checkFinishEvent([this]{this->processCheckFinishEvent();}, "checkFinishEvent"),
setInvalidEvent([this]{this->processSetInvalidEvent();}, "setInvalidEvent")
{
    for (int i = 0; i < CmdIdMax; i++) {
        cmd_state_table[i].valid = false;
    }
}

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
    ResponsePort(name),
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
    panic_if(blockedPacket == nullptr, "There's no blockedPacket!\n");

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
}
bool
CmdStateHelper::handleRequest(PacketPtr pkt) {
    const QueryPayload *queryPayload = pkt->getConstPtr<QueryPayload>();
    if (query_req[static_cast<uint8_t>(queryPayload->clientID)].size() >= maxQueryReqSize) {
        return false;
    }
    else query_req[static_cast<uint8_t>(queryPayload->clientID)].push_back(pkt);
    DPRINTF(CmdStateHelper, "Got request from %d\n", static_cast<uint8_t>(queryPayload->clientID));

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
    DPRINTF(CmdStateHelper, "Arbiter chose client_id=%d \n", chosen_id);
    req_cmdID = qp->cmdID;
    if (chosen_id == static_cast<uint8_t>(QryTabClient::READER)) {
        DPRINTF(CmdStateHelper, "Check if event is finish\n");
        schedule(checkFinishEvent, clockEdge(Cycles(1)));
    }
    else {
        if (qp->is_finish) schedule(setFinishEvent, clockEdge(Cycles(1)));
        else schedule(initEvent, clockEdge(Cycles(1)));
    }
}
void
CmdStateHelper::processInitEvent() {
    panic_if(cmd_state_table[req_cmdID].valid, "The cmdID is inited!");
    DPRINTF(CmdStateHelper, "Initiate Cmd state table\n");
    cmd_state_table[req_cmdID].valid = true;
    cmd_state_table[req_cmdID].finish = false;
    schedule(arbiterEvent, clockEdge(Cycles(1)));
}
void
CmdStateHelper::processSetFinishEvent() {
    panic_if(cmd_state_table[req_cmdID].valid == false,"The cmdID is not inited!");
    DPRINTF(CmdStateHelper, "Set cmd state table[%d] finish\n", req_cmdID);
    cmd_state_table[req_cmdID].valid = true;
    cmd_state_table[req_cmdID].finish = true;

    // just ACK resp
    pendingReqPkt->makeResponse();
    instPort.sendPacket(pendingReqPkt);
    schedule(arbiterEvent, clockEdge(Cycles(1)));
}
void
CmdStateHelper::processCheckFinishEvent() {
    panic_if(cmd_state_table[req_cmdID].valid == false, "The cmdID is not inited!");
    // Resp includes correct information
    pendingReqPkt->makeResponse();

    pendingReqPkt->getPtr<QueryPayload>()->is_finish = cmd_state_table[req_cmdID].finish;
    instPort.sendPacket(pendingReqPkt);

    // cmd finiished
    if (cmd_state_table[req_cmdID].finish) {
        DPRINTF(CmdStateHelper, "Check cmd state table[%d] finished\n", req_cmdID);
        schedule(setInvalidEvent, clockEdge(Cycles(1)));
    }
    // not yet finish
    else {
        DPRINTF(CmdStateHelper, "Check cmd state table[%d] not yet finished\n", req_cmdID);
        schedule(arbiterEvent, clockEdge(Cycles(1)));
    }
}
void
CmdStateHelper::processSetInvalidEvent() {
    panic_if(cmd_state_table[req_cmdID].valid == false,"The cmdID is not inited!");
    DPRINTF(CmdStateHelper, "Set cmd state table[%d] Invalid\n", req_cmdID);
    cmd_state_table[req_cmdID].valid = false;
    schedule(arbiterEvent, clockEdge(Cycles(1)));
    // panic_if(table_read_out_wire.valid == true,"The cmdID is inited!")
}
}