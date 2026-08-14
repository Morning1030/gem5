

#include "mem/cache/tags/pic_llc_tags.hh"

#include <cassert>

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/CacheTags.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/tags/indexing_policies/base.hh"

namespace gem5
{

PICLLCTags::PICLLCTags(const Params &p)
    : BaseSetAssoc(p),
      numBanks(p.num_banks),
      normalCacheWay(p.normal_cache_way),
      numMatsPerBank(p.num_mats_per_bank),
      numSubArraysPerMat(p.num_sub_arrays_per_mat),
      matSliceBytes(p.mat_slice_bytes),
      wayPICModeBitmap(0),
      blkShift(floorLog2(blkSize)),
      bankBits(floorLog2(numBanks))
      /*picStats(*this)*/
{
    fatal_if(numBanks == 0 || !isPowerOf2(numBanks),
             "PICLLCTags: num_banks (%u) must be a non-zero power of 2",
             numBanks);
    fatal_if(normalCacheWay >= allocAssoc,
             "PICLLCTags: normal_cache_way (%u) must be < assoc (%u)",
             normalCacheWay, allocAssoc);
    fatal_if(allocAssoc > 32,
             "PICLLCTags: associativity %u exceeds the 32-way bitmap limit",
             allocAssoc);

    // Sanity-check Mat geometry against the block size.
    // Paper: 4 Mats × 16 bytes = 64 bytes = one cache line per way.
    // We only warn; the user can intentionally choose different parameters.
    const unsigned matCoverage =
        (numMatsPerBank / allocAssoc) * matSliceBytes;
    if (matCoverage != blkSize) {
        warn("PICLLCTags: Mat geometry implies %u bytes/line; "
             "block_size=%u bytes. Adjust num_mats_per_bank or "
             "mat_slice_bytes if inaccurate.",
             matCoverage, blkSize);
    }

    DPRINTF(CacheTags,
            "PICLLCTags: %u banks, %u ways (%u PIC + 1 normal), "
            "%u Mats/bank x %u sub-arrays, %u-byte Mat slice\n",
            numBanks, allocAssoc, allocAssoc - 1,
            numMatsPerBank, numSubArraysPerMat, matSliceBytes);
}

// ---------------------------------------------------------------------------
// accessBlock — tag lookup with PIC-mode arbiter 
// ---------------------------------------------------------------------------

CacheBlk*
PICLLCTags::accessBlock(const PacketPtr pkt, Cycles &lat)
{
    // check cache hit and miss
    // if miss -> return nullptr, if hit -> check the way is in PIC mode or not
    CacheBlk *blk = findBlock({pkt->getAddr(), pkt->isSecure()});

    // All ways are read in parallel for tag comparison (one access per way).
    stats.tagAccesses += allocAssoc;
    if (sequentialAccess) {
        if (blk != nullptr) {
            stats.dataAccesses += 1;
        }
    } else {
        stats.dataAccesses += allocAssoc;
    }

    if (blk != nullptr) {
        const unsigned way = blk->getWay();

        // Arbiter: if the matched way is in PIC/compute mode, deny access.
        // The requestor will be retried after the PIC computation completes
        // and the way is returned to cache mode via setWayPICMode().
        if (isWayInPICMode(way)) {
            //picStats.modeBlockedAccesses++;
            DPRINTF(CacheTags,
                    "PICLLCTags: addr %#x blocked — way %u is in PIC mode "
                    "(bank %u)\n",
                    pkt->getAddr(), way, getBankIndex(pkt->getAddr()));
            lat = lookupLatency;
            return nullptr; //all in PIC mode
        }

        // Normal cache hit.
        blk->increaseRefCount();
        replacementPolicy->touch(blk->replacementData, pkt);
        if (way == normalCacheWay) {
            //picStats.normalCacheHits++;
        }
    }

    lat = lookupLatency;
    return blk; // if miss-> return nullptr
}

// ---------------------------------------------------------------------------
// findVictim — replacement policy (cache miss)
// ---------------------------------------------------------------------------

CacheBlk*
PICLLCTags::findVictim(const CacheBlk::KeyType& key,
                        const std::size_t size,
                        std::vector<CacheBlk*>& evict_blks,
                        const uint64_t partition_id)
{
    // step 1: get all candidate ways for the given key
    std::vector<ReplaceableEntry*> entries =
        indexingPolicy->getPossibleEntries(key); 

    //step 2
    // Remove any way currently in PIC/compute mode.
    // Those ways hold live compute data and must not be evicted.
    entries.erase(
        std::remove_if(entries.begin(), entries.end(),
            [this](ReplaceableEntry *e) {
                return isWayInPICMode(
                    static_cast<CacheBlk*>(e)->getWay());
            }),
        entries.end());
    
    //step 3
    // Apply the partitioning policy on top of the PIC-mode filter.
    if (partitionManager) {
        partitionManager->filterByPartition(entries, partition_id);
    }

    //step 4: if no ways are available for replacement, return nullptr and record the failure.
    if (entries.empty()) {
        //picStats.victimSearchFails++;
        DPRINTF(CacheTags,
                "PICLLCTags: no cache-mode victim for addr %#x\n",
                key.address);
        evict_blks.push_back(nullptr);
        return nullptr;
    }

    //step 5: choose a victim from the remaining candidates using the replacement policy.
    CacheBlk *victim =
        static_cast<CacheBlk*>(replacementPolicy->getVictim(entries));
    evict_blks.push_back(victim);
    return victim;
}

// ---------------------------------------------------------------------------
// setWayPICMode — runtime mode switch (called by Switch Controller after
//                 flush is fully complete)
// ---------------------------------------------------------------------------

void
PICLLCTags::setWayPICMode(unsigned way, bool pic)
{
    fatal_if(way >= allocAssoc,
             "PICLLCTags::setWayPICMode: way %u out of range (assoc=%u)",
             way, allocAssoc);
    fatal_if(way == normalCacheWay,
             "PICLLCTags::setWayPICMode: way %u is the normal cache way "
             "and cannot be switched to PIC mode", way);

    // Safety check: when switching TO PIC mode, the way must already be
    // fully flushed (no valid blocks remain).  The Cache Controller is
    // responsible for calling invalidateWay() before calling this function.
    if (pic) {
        auto remaining = getDirtyBlocksInWay(way);
        panic_if(!remaining.empty(),
                 "PICLLCTags::setWayPICMode: way %u still has %zu dirty "
                 "block(s) — Cache Controller must flush before switching "
                 "to PIC mode", way, remaining.size());
    }

    const bool wasSet = static_cast<bool>((wayPICModeBitmap >> way) & 1u);
    if (wasSet == pic) {
        DPRINTF(CacheTags,
                "PICLLCTags: way %u already in %s mode — no change\n",
                way, pic ? "PIC" : "cache");
        return;
    }

    if (pic) {
        wayPICModeBitmap |= (1u << way);
    } else {
        wayPICModeBitmap &= ~(1u << way);
    }

    //picStats.modeSwitches++;
    DPRINTF(CacheTags,
            "PICLLCTags: way %u -> %s mode (bitmap=0x%08x, active=%u)\n",
            way, pic ? "PIC" : "cache",
            wayPICModeBitmap, activePICWays());
}

// ---------------------------------------------------------------------------
// Flush support — cache bank side of the flush sequence
// ---------------------------------------------------------------------------

std::vector<CacheBlk*>
PICLLCTags::getDirtyBlocksInWay(unsigned way) const
{
    std::vector<CacheBlk*> dirtyBlks;
    for (const CacheBlk &blk : blks) {
        if (blk.getWay() == way &&
            blk.isValid() &&
            blk.isSet(CacheBlk::DirtyBit)) {
            dirtyBlks.push_back(const_cast<CacheBlk*>(&blk));
        }
    }
    DPRINTF(CacheTags,
            "PICLLCTags: getDirtyBlocksInWay(%u): %zu dirty block(s) found\n",
            way, dirtyBlks.size());
    return dirtyBlks;
}

CacheBlk*
PICLLCTags::getBlockByWaySet(unsigned way, unsigned set) const
{
    for (const CacheBlk &blk : blks) {
        if (blk.getWay() == way && blk.getSet() == set)
            return const_cast<CacheBlk*>(&blk);
    }
    return nullptr;
}

void
PICLLCTags::invalidateWay(unsigned way)
{
    fatal_if(way == normalCacheWay,
             "PICLLCTags::invalidateWay: refusing to invalidate normal "
             "cache way %u", way);

    unsigned count = 0;
    for (CacheBlk &blk : blks) {
        if (blk.getWay() == way && blk.isValid()) {
            // Clears valid bit + updates replacement data.
            // Called only after Cache Controller has confirmed:
            //   1. dirty data written back to DRAM
            //   2. L1 invalidation acks received
            invalidate(&blk);
            ++count;
        }
    }
    DPRINTF(CacheTags,
            "PICLLCTags: invalidateWay(%u): %u block(s) invalidated — "
            "way is now clean and safe for PIC compute\n",
            way, count);
    //picStats.wayFlushesCompleted++;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

bool
PICLLCTags::isWayInPICMode(unsigned way) const
{
    if (way == normalCacheWay)
        return false;
    return static_cast<bool>((wayPICModeBitmap >> way) & 1u);
}

unsigned
PICLLCTags::getBankIndex(Addr addr) const
{
    // Bank-select bits sit immediately above the block-offset bits.
    // Example (blkSize=64, numBanks=4):
    //   addr[5:0]  = byte offset within cache line
    //   addr[7:6]  = bank index (0-3)
    //   addr[...] = set bits
    return static_cast<unsigned>((addr >> blkShift) & (numBanks - 1));
}

unsigned
PICLLCTags::activePICWays() const
{
    return static_cast<unsigned>(popCount(wayPICModeBitmap));
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

/*
PICLLCTags::PICLLCTagStats::PICLLCTagStats(PICLLCTags &parent)
    : statistics::Group(&parent),
      ADD_STAT(normalCacheHits, statistics::units::Count::get(),
               "Hits on the normal cache way (way 0)"),
      ADD_STAT(modeBlockedAccesses, statistics::units::Count::get(),
               "CPU accesses blocked because the matching way is in PIC mode"),
      ADD_STAT(modeSwitches, statistics::units::Count::get(),
               "Total way-mode switches (cache to PIC or PIC to cache)"),
      ADD_STAT(victimSearchFails, statistics::units::Count::get(),
               "findVictim() calls with no available cache-mode victim"),
      ADD_STAT(wayFlushesCompleted, statistics::units::Count::get(),
               "Number of way flushes completed (invalidateWay() calls)")
{
}
*/

} // namespace gem5
