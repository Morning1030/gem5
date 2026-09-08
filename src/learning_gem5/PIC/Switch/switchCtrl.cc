Scheduler::SwitchController::SwitchController(Scheduler *owner)
    : owner(owner),
      setID(0),
      wayID(0),
      wayStateValid(0),
      flushAddr(0),
      currSwitchType(SwitchType::PIC2Cache),
      switchEvent([this] { processSwitchEvent(); }, "switchEvent"),
      queryEvent([this] { processQueryEvent(); }, "queryEvent"),
      requestFlushEvent([this] { processRequestFlushEvent(); }, "flushEvent"),
      switch2PICEvent([this] { processSwitch2PICEvent(); }, "switch2PICEvent"),
      switch2CacheEvent([this] { processSwitch2CacheEvent(); }, "switch2CacheEvent")
{
}

bool
Scheduler::SwitchController::handleResponse(PacketPtr pkt)
{
    return true;
}

void
Scheduler::SwitchController::processSwitchEvent()
{
}

void
Scheduler::SwitchController::processNextSet()
{
}

void
Scheduler::SwitchController::processQueryEvent()
{
    size_t pktSize = std::max(sizeof(QueryPayload), sizeof(RespPayload));
    RequestorID requestorId = system.getRequestorId(this, "Scheduler");

    RequestPtr request = std::make_shared<Request>(
        pioAddr + offset                    // the target MMIO address of cache controller
        pktSize,                            // setID, wayID
        Request::QueryWay,                  // flag
        requestorId
    );

    PacketPtr pkt = new Packet(request, MemCmd::QueryReq);
    QueryPayload *queryPayload = new QueryPayload{setID, wayID};
    pkt->dataDynamic(reinterpret_cast<uint8_t*>(queryPayload));

    bool success = owner->cacheControllerPort.sendTimingReq(pkt);
    // if (!success) {
    // owner->retryQueue.push_back(pkt);
}

void
Scheduler::SwitchController::processRequestFlushEvent()
{
}

void
Scheduler::SwitchController::processSwitch2PICEvent()
{
}

void
Scheduler::SwitchController::processSwitch2CacheEvent()
{
}