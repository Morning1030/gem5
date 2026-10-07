#include "learning_gem5/PIC/cache_controller.hh"

#include <algorithm>
#include <cassert>
#include <cstring>

#include "base/trace.hh"
#include "debug/CacheController.hh"
#include "learning_gem5/PIC/Control/scheduler.hh"
#include "learning_gem5/PIC/Switch/pic_payloads.hh"

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
    return getBlk(setID, wayID)->isValid();
}

Addr
PICTags::getSetWayAddr(const uint32_t setID, const uint32_t wayID)
{
    return regenerateBlkAddr(getBlk(setID, wayID));
}

Addr
PICTags::getSetWayTag(const uint32_t setID, const uint32_t wayID)
{
    return getBlk(setID, wayID)->getTag();
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

void
PICTags::setWayRangePICMode(const uint32_t beginWay, const uint32_t endWay,
                            bool picMode)
{
    panic_if(beginWay > endWay, "PICTags: bad way range [%u, %u]",
             beginWay, endWay);
    for (uint32_t w = beginWay; w <= endWay; ++w) {
        setWayPICMode(w, picMode);
    }
}


CacheController::CacheController(const CacheControllerParams &params)
    : /*BaseCache(params, params.blk_size),*/
      picDirPort(params.name + ".pic_ctrl_port", this),
      picFlushPort(params.name + ".pic_flush_port", this)
{
}

Port &
CacheController::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "pic_ctrl_port") return picCtrlPort;
    if (if_name == "pic_flush_port") return picFlushPort;
    return BaseCache::getPort(if_name, idx);
}

bool
CacheController::handleQueryWayState(PacketPtr pkt)
{
    const auto *q = pkt->getConstPtr<CacheWayQueryPayload>();
    const uint32_t setID    = q->setID;
    const uint32_t beginWay = q->beginWay;
    const uint32_t endWay   = q->endWay;

    panic_if(beginWay > endWay ||
             endWay >= (uint32_t)picTags()->getWayAllocationMax() ||
             endWay - beginWay + 1 > PIC_MAX_WAYS,          // ← D-7
             "QueryWayState: bad range set=%u ways=[%u,%u]",
             setID, beginWay, endWay);

    pkt->makeResponse();
    auto *resp = pkt->getPtr<CacheWayQueryRespPayload>();
    *resp = CacheWayQueryRespPayload{};

    // RTL 只回報 state != INVALID 的 way
    for (uint32_t w = beginWay; w <= endWay; ++w) {
        CacheBlk *blk = picTags()->getBlk(setID, w);
        if (blk->isValid()) {
            resp->wayID[resp->numValid] = w;
            resp->tag[resp->numValid]   = blk->getTag();
            resp->numValid++;
        }
    }

    DPRINTF(CacheController, "QueryWayState: set=%u ways=[%u,%u] valid=%u\n",
            setID, beginWay, endWay, resp->numValid);

    const Cycles lat = lookupLatency + Cycles(endWay - beginWay + 1);
    picCtrlPort.schedTimingResp(pkt, clockEdge(lat));
    return true;
}

bool
CacheController::handleDrainQuery(PacketPtr pkt)
{
    const uint32_t wayID = pkt->getConstPtr<DrainQueryPayload>()->wayID;
    const bool isDrained = mshrQueue.isEmpty();

    DPRINTF(CacheController, "DrainQuery: wayID=%u isDrained=%d\n",
            wayID, isDrained);

    pkt->makeResponse();
    pkt->getPtr<DrainQueryRespPayload>()->isDrained = isDrained ? 1u : 0u;

    picCtrlPort.schedTimingResp(pkt, curTick());

    return true;
}

bool
CacheController::handleFlushReq(PacketPtr pkt)
{
    // DCF
    const auto *f = pkt->getConstPtr<CacheFlushPayload>();
    CacheBlk *blk = nullptr;
    for (uint32_t w = 0; w < (uint32_t)picTags()->getWayAllocationMax(); ++w) {
        CacheBlk *cand = picTags()->getBlk(f->setID, w);
        if (cand->isValid() && cand->getTag() == f->tag) { 
            blk = cand; 
            break; 
        }
    }

    if (blk && blk->isValid()) {
        if (blk->isSet(CacheBlk::DirtyBit)) {
            PacketPtr wb_pkt = writebackBlk(blk);
            allocateWriteBuffer(wb_pkt, curTick());
        }

        invalidateBlock(blk);
    }

    pkt->makeResponse();
    picFlushPort.schedTimingResp(pkt, curTick());
    return true;
}

bool
CacheController::handleWayMode(PacketPtr pkt)
{
    const auto *m = pkt->getConstPtr<WayModePayload>(); 

    picTags()->setWayRangePICMode(m->beginWay, m->endWay, m->picMode != 0);

    pkt->makeResponse();
    picCtrlPort.schedTimingResp(pkt, clockEdge(Cycles(1)));
    return true;
}

bool
CacheController::handlePicCtrlReq(PacketPtr pkt)
{
    panic_if(!pkt->isWrite(), "%s expected a PIC dir WriteReq", name());

    const auto reg = static_cast<PicCtrlReg>(pkt->getAddr() - PicCtrlBase);
    panic_if(!isDirReg(reg), "%s bad dir offset %#llx", name(),
             (unsigned long long)(pkt->getAddr() - PicCtrlBase));

    switch (reg) {
      case PicCtrlReg::DrainQuery: return handleDrainQuery(pkt);
      case PicCtrlReg::WayQuery:   return handleQueryWayState(pkt);
      case PicCtrlReg::WayMode:    return handleWayMode(pkt);
      default:                     panic("unreachable");
    }
}

} // namespace gem5
