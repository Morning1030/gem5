

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
      // initial_pic_ways seeds both states (tests that drive Mats directly)
      wayPICModeBitmap(p.initial_pic_ways),
      dirPICModeBitmap(p.initial_pic_ways),
      matBusyBitmap(p.num_banks, 0),
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
    fatal_if((p.initial_pic_ways >> normalCacheWay) & 1u,
             "PICLLCTags: initial_pic_ways has the normal cache way (%u) set",
             normalCacheWay);
    fatal_if(allocAssoc > 32,
             "PICLLCTags: associativity %u exceeds the 32-way bitmap limit",
             allocAssoc);

    // Sanity-check Mat geometry against the block size.
    // Paper: 4 banks × 16 bytes = 64 bytes = one cache line per way.
    // We only warn; the user can intentionally choose different parameters.
    const unsigned matCoverage = numBanks * matSliceBytes;
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
// accessBlock — tag lookup; a hit on a bank-PIC way is a bug
// ---------------------------------------------------------------------------

CacheBlk*
PICLLCTags::accessBlock(const PacketPtr pkt, Cycles &lat)
{
    // check cache hit and miss
    // if miss -> return nullptr, if hit -> the way must not be in PIC mode
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

        // RTL (BankSellPIC.scala:107-108) asserts a cache access never
        // lands on a PIC way; paper Sec. 5.2: PIC ways never hit. Every
        // block was flushed before its way entered PIC mode, so a hit here
        // means the switch/flush flow is broken. isMatBusy() is a
        // different, query-only signal and plays no part here.
        panic_if(isWayInPICMode(way),
                 "PICLLCTags: addr %#x hit way %u, which is in PIC mode",
                 pkt->getAddr(), way);

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
    // Remove any way whose directory state is PIC (RTL Directory picks
    // victims with picLevels, already set at the switch's pre_check).
    entries.erase(
        std::remove_if(entries.begin(), entries.end(),
            [this](ReplaceableEntry *e) {
                return isWayDirPIC(static_cast<CacheBlk*>(e)->getWay());
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
// setDirPicWays / commitBankPicState — the two PIC states of a switch
// ---------------------------------------------------------------------------

void
PICLLCTags::setDirPicWays(unsigned beginWay, unsigned endWay, bool pic)
{
    fatal_if(beginWay > endWay || endWay >= allocAssoc,
             "PICLLCTags::setDirPicWays: bad way range [%u, %u] "
             "(assoc=%u)", beginWay, endWay, allocAssoc);
    fatal_if(normalCacheWay >= beginWay && normalCacheWay <= endWay,
             "PICLLCTags::setDirPicWays: range [%u, %u] has the normal "
             "cache way %u", beginWay, endWay, normalCacheWay);

    const uint32_t ways =
        static_cast<uint32_t>(mask(endWay - beginWay + 1) << beginWay);
    if (pic) {
        dirPICModeBitmap |= ways;
    } else {
        dirPICModeBitmap &= ~ways;
    }

    DPRINTF(CacheTags,
            "PICLLCTags: dir ways [%u, %u] -> %s (dir=0x%08x)\n",
            beginWay, endWay, pic ? "PIC" : "cache", dirPICModeBitmap);
}

void
PICLLCTags::commitBankPicState()
{
    // Model-only check (the RTL has no such assert): every way entering
    // PIC mode must have been fully flushed. Catches flush bugs before a
    // Mat overwrites live cache data.
    const uint32_t entering = dirPICModeBitmap & ~wayPICModeBitmap;
    if (entering) {
        for (const CacheBlk &blk : blks) {
            panic_if(blk.isValid() && ((entering >> blk.getWay()) & 1u),
                     "PICLLCTags::commitBankPicState: way %u set %u is "
                     "still valid while entering PIC mode",
                     blk.getWay(), blk.getSet());
        }
    }

    //picStats.modeSwitches++;
    wayPICModeBitmap = dirPICModeBitmap;
    DPRINTF(CacheTags,
            "PICLLCTags: bank state committed (bitmap=0x%08x, active=%u)\n",
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

bool
PICLLCTags::isWayDirPIC(unsigned way) const
{
    if (way == normalCacheWay)
        return false;
    return static_cast<bool>((dirPICModeBitmap >> way) & 1u);
}

void
PICLLCTags::setMatBusy(unsigned bank, unsigned way, bool busy)
{
    fatal_if(bank >= numBanks, "PICLLCTags::setMatBusy: bank %u out of "
             "range (%u banks)", bank, numBanks);
    fatal_if(way >= allocAssoc, "PICLLCTags::setMatBusy: way %u out of "
             "range (assoc=%u)", way, allocAssoc);
    fatal_if(way == normalCacheWay, "PICLLCTags::setMatBusy: way %u is "
             "the normal cache way and has no Mat", way);

    if (busy) {
        matBusyBitmap[bank] |= (1u << way);
    } else {
        matBusyBitmap[bank] &= ~(1u << way);
    }
    DPRINTF(CacheTags, "PICLLCTags: bank %u way %u busy=%d\n",
            bank, way, busy);
}

bool
PICLLCTags::isMatBusy(unsigned bank, unsigned way) const
{
    if (way == normalCacheWay)
        return false;
    fatal_if(bank >= numBanks, "PICLLCTags::isMatBusy: bank %u out of "
             "range (%u banks)", bank, numBanks);
    return static_cast<bool>((matBusyBitmap[bank] >> way) & 1u);
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
