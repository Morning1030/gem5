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
    cacheControllerPort(params->name + ".mem_side_cc", this),
    DPMPort(params->name + ".mem_side_dpm", this),
    cacheBankPort(params->name + ".mem_side_cb", this),
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
    else if (if_name == "mem_side_cc") {
        return cacheControllerPort;
    }
    else if (if_name == "mem_side_dpm") {
        return DPMPort;
    }
    else if (if_name == "mem_side_cb") {
        return cacheBankPort;
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
Scheduler::handleResponse(PortID portID, PacketPtr pkt)
{
    DPRINTF(Scheduler, "Got response for addr %#x\n", pkt->getAddr());
    switch(portID):
        case(CC):
            return switchController->handleResponse(pkt);
        // p2s_is_done from DPM
        case(DPM):
            delete pkt;
            return true;
        case(CB):
        default:
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
    uint64_t dataPayload = pkt->getRaw<uint64_t>();
    
    DPRINTF(Scheduler, "Decoding Request at addr %#x\n", pktAddr);

    if (pkt->isWrite()) {   // pkt contains data payload
        switch(offset):
            // SET_SRC
            case 0x00:
                src = dataPayload;
                DPRINTF(Scheduler, "SET_SRC to %#x\n", src);
                break;

            // SET_DST
            case 0x02:
                dst = dataPayload;
                DPRINTF(Scheduler, "SET_DST to %#x\n", dst);
                break;
            
            // SET_SIZE
            case 0x04:
                row = static_cast<uint32_t>(dataPayload >> 32);
                col = static_cast<uint32_t>(dataPayload & 0xFFFFFFFF);
                DPRINTF(Scheduler, "SET_SIZE to (%d, %d)\n", row, col);
                break;

            // SET_PARAM 
            case 0x06:
                taskScheduler->prepareTask(pkt);
                break;
            default:
    }
    // waiting for instructions, self looping at each cycle
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
    currState(TaskState::IDLE),
    p2sEvent([this]{this->processP2SEvent();}, "p2sEvent")
{
}
void
Scheduler::TaskScheduler::prepareTask(PacketPtr paramPkt, Addr currentSrc, Addr currentDst, uint32_t currentRow, uint32_t currentCol)
{
    uint64_t dataPayload = paramPkt->getRaw<uint64_t>();
    funcID = 
        switch(funcID):
            // load
            case 0:
                taskScheduler->nextTask.taskOp = LOAD;
                taskScheduler->nextTask.push_back();
                DPRINTF(Scheduler, "SET_PARAM LOAD\n");
            // store
            case 1:
                taskScheduler->nextTask.taskOp = STORE;
                taskScheduler->nextTask.push_back();
                DPRINTF(Scheduler, "SET_PARAM STORE\n");
            // p2s
            case 2:
                // TODO　make packet for p2s(later send to dpm)
                size_t pktSize = sizeof(...)+
                RequestorID requestorId = system.getRequestorId(this, "Scheduler");

                RequestPtr request = std::make_shared<Request>(
                    pioAddr + offset    // the target MMIO address of dpm
                    sizeof(Addr),       // flush addr
                    Request::FlushWay,  // flag
                    requestorId
                );

                PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
                pkt->allocate();

                pkt->setLE<Addr>(flushAddr);

                taskScheduler->nextTask.taskOp = P2S;
                taskScheduler->nextTask.push_back(pkt);
                DPRINTF(Scheduler, "SET_PARAM P2S\n");
                break;
            // cal
            case 3:
                taskScheduler->nextTask.taskOp = CAL;
                taskScheduler->nextTask.push_back();
                DPRINTF(Scheduler, "SET_PARAM CAL\n");
                break;
            // acc
            case 4:
                taskScheduler->nextTask.taskOp = ACC;
                taskScheduler->nextTask.push_back();
                DPRINTF(Scheduler, "SET_PARAM ACC\n");
                break;
            // switch
            case 5:
                // decode datapayload and set wayID and switch type
                wayID = static_cast<uint32_t>(dataPayload & 0xFFFFFFFF);
                uint32 st = static_cast<uint32_t>(dataPayload >> 32);   // switch type
                if (st == 0) switchController->switchType = PIC2Cache;
                else switchController->switchType = Cache2PIC;
                
                taskScheduler->nextImmTask.taskOp = SWITCH;
                taskScheduler->nextImmTask.pkt = pkt;
                DPRINTF(Scheduler, "SET_PARAM SWITCH\n");
                break;
    // TODO
    // call triggerTS
}
void
Scheduler::TaskScheduler::triggerTS()
{
    // TODO
    // schedule the task requests 
    // for every cycle/trigger
    // check if there's immTask if yes then check hardware condition(IDLE/BUSY)
    // check if there's task in the queue, if yes then check hardware condition(IDLE/BUSY)

    
    // need to wait
    if (currState != IDLE) {
        return;
    }
    if (t.taskOp == SWITCH) {
        currState = SWITCHING;
        // trigger switch controller
        schedule(owner->switchController->switchEvent, curTick() + cycles(1));
    }
    else if (t.taskOp == P2S) {
        currState = P2SING;
        schedule(P2SEvent, curTick() + cycles(1));
    }
}

void
Scheduler::TaskScheduler::processP2SEvent() {
    // send the nextTask packet to DPM
    bool success = owner->DPMPort.sendTimingReq(nextImmTask.pkt);
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

    // query way's response
    if (pkt->req->isQueryWay()) {
        RespPayload respPayload;
        pkt->writeData(reinterpret_cast<uint8_t*>(&respPayload));

        flushAddr = respPayload.addr;
        wayStateValid = respPayload.state;
        DPRINTF(Scheduler, "Response: Address=%p, wayState=%d", flushAddr, wayStateValid);
        delete pkt;

        // request flush if way state valid
        if (waystatevalid) {
            schedule(requestFlushEvent, curTick() + cycles(1));
        }
        else {
            processNextSet();
        }
    }
    // flush controller's response
    else if (pkt->req->isFlushWay()) {
        // advance to next set
        processNextSet();
    }
    // switch way state response
    else if (pkt->req->isCache2PIC() || pkt->req->isPIC2Cache()) {
        delete pkt;
        // notify scheduler switch finish
        owner->taskScheduler->currState = TaskState::IDLE;
    }
    return true;
}
void
Scheduler::SwitchController::processSwitchEvent()
{
    // schedule first setID switch
    schedule(queryEvent, curTick() + cycles(1));
}
void
Scheduler::SwitchController::processNextSet()
{
    setID++;

    if (setID < 1024) {
        schedule(queryEvent, curTick() + cycles(1));
    }
    else {
        if (currSwitchType == Cache2PIC) schedule(switch2PICEvent, curTick() + cycles(1));
        else if (currSwitchType == PIC2Cache) schedule(switch2CacheEvent, curTick() + cycles(1));
    }
}
void
Scheduler::SwitchController::processQueryEvent()
{
    size_t pktSize = std::max(sizeof(QueryPayload, sizeof(RespPayload)));
    RequestorID requestorId = system.getRequestorId(this, "Scheduler");

    RequestPtr request = std::make_shared<Request>(
        pioAddr + offset                    // the target MMIO address of cache controller
        pktSize,                            // setID, wayID
        Request::QueryWay,                  // flag
        requestorId
    );

    PacketPtr pkt = new Packet(request, MemCmd::QueryReq);
    pkt->allocate();

    QueryPayload queryPayload = {setID, wayID};
    pkt->setData(reinterpret_cast<const uint8_t*>(&queryPayload));
    bool success = owner->cacheControllerPort.sendTimingReq(pkt);
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

    bool success = owner->cacheControllerPort.sendTimingReq(pkt);
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

    bool success = owner->cacheControllerPort.sendTimingReq(pkt);
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

    bool success = owner->cacheControllerPort.sendTimingReq(pkt);
    // if (!success) {
    // owner->retryQueue.push_back(pkt);
}



} // namespace gem5
