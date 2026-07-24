/*
 * Copyright (c) 2017 Jason Lowe-Power
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "learning_gem5/PIC/scheduler.hh"

#include "debug/Scheduler.hh"

namespace gem5
{
Scheduler::Scheduler(SchedulerParams *params) :
    ClockedObject(params),
    instPort(params->name + ".inst_port", this),
    memPort(params->name + ".mem_side", this),
    taskScheduler(this),
    switchController(this),
    decodeEvent([this]{this->processDecodeEvent();}, "decodeEvent")
{
}
Port&
Scheduler::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port") {
        return instPort;
    }
    else if (if_name == "mem_side") {
        return memPort;
    }
    else {
        return ClockedObject::getPort(if_name, idx);
    }
}

void
Scheduler::handleFunctional(PacketPtr pkt){}

bool
Scheduler::handleRequest(PacketPtr pkt)
{
    if (instQueue.size() == maxInstQueueSize) {
        return false;
    }
    DPRINTF(Scheduler, "Got request for addr %#x\n", pkt->getAddr());
    // push to the IQ
    instQueue.push_back(pkt);
    return true;
}

bool
Scheduler::handleResponse(PacketPtr pkt)
{
    DPRINTF(Scheduler, "Got response for addr %#x\n", pkt->getAddr());
    return switchController.handleResponse(pkt);
}

AddrRangeList
Scheduler::getAddrRanges() const {}

void
Scheduler::sendRangeChange()
{
    instPort.sendRangeChange();
}

void
Scheduler::processDecodeEvent()
{
    // dequeue and decode
    if (taskScheduler.currState != IDLE || instQueue.empty()) return;

    PacketPtr pkt = instQueue.front();
    instQueue.pop_front();
    // to ensure the bytes of datapayload
    // unsigned int pktSize = pkt->getSize();
    // DPRINTF(Scheduler, "PacketPtr get Size is %u bytes\n", pktSize);
    uint64_t pktAddr = pkt->getAddr();
    uint64_t offset = pktAddr - MMIOBase;   // Base address not yet defined
    DPRINTF(Scheduler, "Decoding Request at addr %#x\n", pktAddr);

    Task nextTask;
    if (pkt->isWrite()) {
        uint64_t dataPayload = pkt->getRaw<uint64_t>();
        switch(offset):
            // switch inst
            case 0x00:
                nextTask.taskOp = SWITCH;
                wayID = (uint32_t) dataPayload;     // data member wayID
                nextTask.taskParam.wayID = wayID;
                DPRINTF(Scheduler, "WayID: %u is requested to switch mode\n", nextTask.taskParam.wayID);

                triggerTS(nextTask);
                break;
            default:
    }

    if (taskScheduler.currState == IDLE && !instQueue.empty()) {
        schedule(decodeEvent, curTick() + cycles(1))    // temporarily set to 1
    }

}
Scheduler::CPUSidePort::CPUSidePort(const std::string& name, Scheduler *owner) :
    ResponsePort(name, owner),
    owner(owner)
{

}


AddrRangeList
Scheduler::CPUSidePort::getAddrRanges() const
{
    // CPUSidePort simply forwards to Scheduler
    return owner->getAddrRanges();
}

void
Scheduler::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    return owner->handleFunctional(pkt);
}

bool
Scheduler::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    if (!owner->handleRequest(pkt)) {
        // handle failed
        return false;
    }
    else {
        return true;
    }
}

Scheduler::MemSidePort::MemSidePort(const std::string& name, Scheduler *owner)
    RequestPort(name, owner),
    owner(owner)
{

}

void
Scheduler::MemSidePort::sendPacket(PacketPtr pkt)
{

}
void
Scheduler::MemSidePort::sendTimingReq(PacketPtr pkt)
{
    if (!RequestPort::sendTimingReq(pkt)) {
        // owner->blocked = true;
        // owner->retryPkt = pkt;
        return false;
    }
    return true;
}
bool
Scheduler::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(pkt);
}

void
Scheduler::MemSidePort::recvReqRetry()
{

}
void
Scheduler::MemSidePort::recvRangeChange()
{
    owner->sendRangeChange();
}

Scheduler::TaskScheduler::TaskScheduler(Scheduler *owner) :
    owner(owner),
    currState(TaskState::IDLE)
{
}

void
Scheduler::TaskScheduler::triggerTS(Task t)
{
    // need to wait
    if (currState != IDLE) {
        return;
    }
    if (t.taskOp == SWITCH) {
        currState = SWITCHING;
        // trigger switch controller
        schedule(switchEvent, curTick() + cycles(1));

        schedule(decodeEvent, curTick() + cycles(1));
    }
    else {

    }
}

Scheduler::SwitchController::SwitchController(Scheduler* owner) :
    owner(owner);
    switchEvent([this]{this->processSwitchEvent();}, "switchEvent"),
    queryEvent([this]{this->processQueryEvent();}, "queryEvent"),
    requestFlushEvent([this]{this->processRequestFlushEvent();}, "flushEvent"),
    switch2PICEvent([this]{this->processSwitch2PICEvent();}, "switch2PICEvent"),
    switch2CacheEvent([this]{this->processSwitch2CacheEvent();}, "switch2CacheEvent")
{

}
bool
Scheduler::SwitchController::handleResponse(PacketPtr pkt)
{
    assert(pkt->isResponse());

    // uint32_t setID = (payload >> 32) & 0xFFFFFFFF;
    // uint32_t wayID = (payload >> 8)  & 0xFFFFFF;
    // uint8_t  state = payload & 0xFF;

    RespPayload respPayload;
    pkt->writeData(reinterpret_cast<uint8_t*>(&resp));

    flushAddr = respPayload.addr;
    wayStateValid = respPayload.state;
    DPRINTF(Scheduler, "Response: Address=%p, wayState=%d", flushAddr, wayStateValid);
    delete pkt;
    return true;
}
void
Scheduler::SwitchController::processSwitchEvent()
{
    // traverse through set
    for (int i = 0; i < 1024; i++) {
        owner->setID = i;
        // query directory
        schedule(queryEvent, curTick() + cycles(1));
        // valid
        if (wayStateValid) {
            // request flush
            schedule(requestFlushEvent, curTick() + cycles(1));
        }
    }
    schedule(switch2PICEvent, curTick() + cycles(1));
}
void
Scheduler::SwitchController::processQueryEvent()
{
    size_t pktSize = std::max(sizeof(QueryPayload, sizeof(RespPayload)));
    RequestorID requestorId = system.getRequestorId(this, "Scheduler");

    RequestPtr request = std::make_shared<Request>(
        pioAddr + offset    // the target MMIO address of cache controller
        pktSize,  // setID, wayID
        Request::QueryWay,                  // flag
        requestorId
    );

    PacketPtr pkt = new Packet(request, MemCmd::QueryReq);
    pkt->allocate();

    QueryPayload queryPayload = {setID, wayID};
    pkt->setData(reinterpret_cast<const uint8_t*>(&queryPayload));

    bool success = owner->memSidePort.sendTimingReq(pkt);
    // if (!success) {
    // owner->retryQueue.push_back(pkt);
}

void
Scheduler::SwitchController::processRequestFlushEvent()
{
    RequestorID requestorId = system.getRequestorId(this, "Scheduler");

    RequestPtr request = std::make_shared<Request>(
        pioAddr + offset    // the target MMIO address of cache controller
        sizeof(Addr),       // flush addr
        Request::FlushWay,                  // flag
        requestorId
    );

    PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
    pkt->allocate();

    pkt->setLE<Addr>(flushAddr);

    bool success = owner->memSidePort.sendTimingReq(pkt);
    // if (!success) {
    // owner->retryQueue.push_back(pkt);
}
void
Scheduler::SwitchController::processSwitch2PICEvent()
{
    // send request to cache controller to switch mode cache -> PIC
    RequestorID requestorId = system.getRequestorId(this, "Scheduler");

    RequestPtr request = std::make_shared<Request>(
        pioAddr + offset    // the target MMIO address of cache controller
        sizeof(uint32_t),       // wayID
        Request::Cache2PIC,     // flag
        requestorId
    );

    PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
    pkt->allocate();

    pkt->setLE<uint32_t>(wayID);

    bool success = owner->memSidePort.sendTimingReq(pkt);
    // if (!success) {
    // owner->retryQueue.push_back(pkt);
}
void
Scheduler::SwitchController::processSwitch2CacheEvent()
{
    // send request to cache controller to switch mode cache -> PIC
    RequestorID requestorId = system.getRequestorId(this, "Scheduler");

    RequestPtr request = std::make_shared<Request>(
        pioAddr + offset    // the target MMIO address of cache controller
        sizeof(uint32_t),       // wayID
        Request::PIC2Cache,     // flag
        requestorId
    );

    PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
    pkt->allocate();

    pkt->setLE<uint32_t>(wayID);

    bool success = owner->memSidePort.sendTimingReq(pkt);
    // if (!success) {
    // owner->retryQueue.push_back(pkt);
}



} // namespace gem5
