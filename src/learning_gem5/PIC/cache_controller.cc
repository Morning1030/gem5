#include "learning_gem5/PIC/cache_controller.hh"

#include <algorithm>
#include <cassert>

#include "base/trace.hh"
#include "debug/CacheController.hh"
#include "learning_gem5/PIC/scheduler.hh"

namespace gem5
{

CacheBlk*
PICTags::findVictim(const CacheBlk::KeyType& key,
                    const std::size_t size,
                    std::vector<CacheBlk*>& evict_blks,
                    const uint64_t partition_id)
{
    std::vector<ReplaceableEntry*> entries = indexingPolicy->getPossibleEntries(key);

    auto it = entries.begin();
    while (it != entries.end()) {
        CacheBlk* blk = static_cast<CacheBlk*>(*it);
        if (blk && isWayPICMode(blk->getWay())) {
            it = entries.erase(it);
        }
        else {
            ++it;
        }
    }

    if (partitionManager) {
        partitionManager->filterByPartition(entries, partition_id);
    }

    CacheBlk* victim = entries.empty() ? nullptr :
        static_cast<CacheBlk*>(replacementPolicy->getVictim(entries));

    evict_blks.push_back(victim);
    return victim;
}

bool
PICTags::getSetWayValid(const uint32_t setID, const uint32_t wayID)
{
    for (CacheBlk &blk : blks) {
        if (blk.getSet() == setID && blk.getWay() == wayID) {
            return blk.isValid();
        }
    }
    return false;
}

Addr
PICTags::getSetWayAddr(const uint32_t setID, const uint32_t wayID)
{
    for (CacheBlk &blk : blks) {
        if (blk.getSet() == setID && blk.getWay() == wayID) {
            return blk.getAddr();
        }
    }
    return 0;
}

bool
PICTags::isWayPICMode(const uint32_t wayID) const
{
    return wayID < PIC_mode.size() && PIC_mode[wayID];
}

void
PICTags::setWayPICMode(const uint32_t wayID, bool picMode)
{
    if (PIC_mode.size() < allocAssoc) {
        PIC_mode.resize(allocAssoc, false);
    }

    panic_if(wayID >= allocAssoc,
             "PICTags::setWayPICMode way %u out of range assoc=%u",
             wayID, allocAssoc);

    PIC_mode[wayID] = picMode;
}

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
    const bool valid = tags->getSetWayValid(setID, wayID);
    const Addr addr = tags->getSetWayAddr(setID, wayID);

    RespPayload respPayload{};
    respPayload.state = valid;
    respPayload.addr = addr;

    pkt->makeResponse();
    pkt->setData(reinterpret_cast<const uint8_t*>(&respPayload));

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

} // namespace gem5
