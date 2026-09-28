#include "learning_gem5/PIC/cache_controller.hh"

#include <algorithm>
#include <cassert>

#include "base/trace.hh"
#include "debug/CacheController.hh"
#include "learning_gem5/PIC/scheduler.hh"

namespace gem5
{

#if 0
CacheController::CacheController(CacheControllerParams *params) :
    ClockedObject(params),
    instPort(params.name + ".cpu_port", this),
    memPort(params.name + ".mem_port", this)
{
}
#endif

CacheController::CPUSidePort::CPUSidePort(
    const std::string& name, CacheController *owner)
    : BaseCache::CpuSidePort(name, *owner, "pic_control"),
      owner(owner)
{
}

CacheController::MemSidePort::MemSidePort(
    const std::string& name, CacheController *owner)
    : BaseCache::MemSidePort(name, owner, "pic_control"),
      owner(owner)
{
}

bool
CacheController::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    if (pkt->cmd == MemCmd::QueryReq) {
        return owner->handleQueryWayState(pkt);
    }
    return false;
}

bool
CacheController::handleQueryWayState(PacketPtr pkt)
{
    QueryPayload qPayload{};

    pkt->writeData(reinterpret_cast<uint8_t*>(&qPayload));

    const uint32_t setID = qPayload.setID;
    const uint32_t wayID = qPayload.wayID;
    CacheBlk *blk = tags->getBlockByWaySet(wayID, setID);
    const bool valid = blk != nullptr && blk->isValid();
    const Addr addr = blk != nullptr ? blk->getAddr() : 0;

    RespPayload *respPayload = new RespPayload{valid, addr};
    pkt->makeResponse();
    pkt->dataDynamic(reinterpret_cast<uint8_t*>(respPayload));

    cpuSidePort.schedTimingResp(pkt, curTick());

    return true;
}

bool
CacheController::handleFlushReq(PacketPtr pkt)
{
    const Addr flushAddr = pkt->getLE<Addr>();
    CacheBlk *blk = tags->findBlock({flushAddr, pkt->isSecure()});

    if (blk && blk->isValid()) {
        if (blk->isDirty()) {
            PacketPtr wb_pkt = writebackBlk(blk);
            allocateWriteBuffer(wb_pkt, curTick());
        }

        invalidateBlock(blk);
    }

    if (pkt->needsResponse()) {
        pkt->makeResponse();

        // Use BaseCache's queued CPU-side response path so retry is not lost
        cpuSidePort.schedTimingResp(pkt, curTick());
    }
    else {
        delete pkt;
    }

    return true;
}

bool
CacheController::handleCache2PIC(PacketPtr pkt)
{
    const uint32_t wayID = pkt->getLE<uint32_t>();

    // PICLLCTags::setWayPICMode(pic=true) requires the way already
    // flushed (see its own doc comment) -- same writeback-then-invalidate
    // simplification handleFlushReq() above already uses (no wait for
    // the writeback to actually land before invalidating).
    for (CacheBlk *blk : tags->getDirtyBlocksInWay(wayID)) {
        PacketPtr wb_pkt = writebackBlk(blk);
        allocateWriteBuffer(wb_pkt, curTick());
    }
    tags->invalidateWay(wayID);
    tags->setWayPICMode(wayID, true);

    if (pkt->needsResponse()) {
        pkt->makeResponse();
        cpuSidePort.schedTimingResp(pkt, curTick());
    }
    else {
        delete pkt;
    }

    return true;
}

bool
CacheController::handlePIC2Cache(PacketPtr pkt)
{
    const uint32_t wayID = pkt->getLE<uint32_t>();
    // FIXED: was setWayPICMode(wayID, true), a copy-paste bug that made
    // this identical to handleCache2PIC() -- PIC2Cache must switch BACK.
    tags->setWayPICMode(wayID, false);

    if (pkt->needsResponse()) {
        pkt->makeResponse();
        cpuSidePort.schedTimingResp(pkt, curTick());
    }
    else {
        delete pkt;
    }

    return true;
}

} // namespace gem5
