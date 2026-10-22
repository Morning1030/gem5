
/**
 * @file
 * PIC-LLC tag store (PolymorPIC paper, section 4.1.3).
 *
 * Cache: 4 banks x 16 ways, one Mat per (bank, way). A Mat holds a 16-byte
 * slice of every line of its way (4 sub-arrays x 512 rows x 64 bits).
 * Way 0 is always a normal cache way; ways 1-15 can switch between cache
 * and PIC mode, one way across all four banks at a time.
 *
 * This class keeps the CPU away from PIC-mode ways: a tag hit in a PIC way
 * is treated as a miss, and PIC ways are never chosen as victims. Which
 * PIC client gets the bank port is decided by AccessBankArb
 * (mem/cache/pic/access_bank_arb.hh), not here.
 */

#ifndef __MEM_CACHE_TAGS_PIC_LLC_TAGS_HH__
#define __MEM_CACHE_TAGS_PIC_LLC_TAGS_HH__

#include <algorithm>
#include <cstdint>
#include <vector>

//#include "base/statistics.hh"
#include "mem/cache/tags/base_set_assoc.hh"
#include "params/PICLLCTags.hh"

namespace gem5
{

/**
 * PIC-LLC tag store that models 4 cache banks with per-way mode switching.
 *
 * Extends BaseSetAssoc so the rest of the cache hierarchy (MSHR, coherence,
 * replacement) remains unchanged.  Only accessBlock() and findVictim() are
 * overridden to enforce the PIC-mode arbiter.
 *
 * Compared with the old PICTags,
 * which only had findVictim(), the PIC-mode flag and (set, way) lookups,
 * this class adds the parts marked "NEW vs PICTags" below:
 *   - accessBlock(): the CPU cannot hit in a PIC-mode way
 *   - setWayPICMode() checks: never way 0, never a way with dirty lines
 *   - flush support: getDirtyBlocksInWay(), invalidateWay()
 *   - Mat busy bits: setMatBusy(), isMatBusy()
 *   - bank info: getBankIndex(), bankCount(), sliceBytes(), blockSize()
 *   - activePICWays()
 *   - its own Python params (Tags.py), incl. initial_pic_ways for tests
 */
class PICLLCTags : public BaseSetAssoc
{
  public:
    typedef PICLLCTagsParams Params;

    PICLLCTags(const Params &p);
    ~PICLLCTags() = default;

    /**
     * Tag lookup with PIC-mode arbiter.
     * NEW vs PICTags: PICTags did not override this, so the CPU could hit
     * in a PIC-mode way.
     *
     * If the matching block lives in a way that is currently in PIC mode,
     * the arbiter denies CPU access and returns nullptr (the hit is invisible
     * to the CPU until the way is returned to cache mode).
     */
    CacheBlk* accessBlock(const PacketPtr pkt, Cycles &lat) override;

    /**
     * Victim selection restricted to cache-mode ways (same as PICTags).
     *
     * PIC-mode ways hold live compute data and must not be evicted by
     * ordinary CPU misses.
     */
    CacheBlk* findVictim(const CacheBlk::KeyType& key,
                         const std::size_t size,
                         std::vector<CacheBlk*>& evict_blks,
                         const uint64_t partition_id=0) override;

    /**
     * Switch one PIC way between cache mode and PIC/compute mode.
     *
     * The switch applies across all four banks simultaneously, matching
     * the "Level" granularity defined in Fig. 6(c).  Way 0 (normalCacheWay)
     * cannot be switched.
     *
     * @param way  Way index to switch (1 … assoc-1)
     * @param pic  true → PIC mode; false → cache mode
     *
     * NEW vs PICTags: refuses way 0 and a way that still has dirty lines.
     */
    void setWayPICMode(unsigned way, bool pic);

    /** Return true if @p way is currently in PIC/compute mode.
     *  (PICTags had this as isWayPICMode().) */
    bool isWayInPICMode(unsigned way) const;

    /**
     * Per-Mat job-busy bit: true from when that Mat's FSM leaves MainIdle
     * until it returns. Query-only (RTL: Controller.io.busy) -- does NOT
     * gate CPU access. accessBlock()/findVictim() check wayPICModeBitmap
     * (via isWayInPICMode()) instead: RTL's actual CPU-blocking signal
     * (BankSellPIC.scala:107-127, cacheLevelEnd/picActivated) is driven
     * only by an explicit SWITCH command, never by a Mat's own busy
     * state -- a way stays blocked through any idle gap between jobs
     * until SWITCH says otherwise. Driven externally (Mat), not by this
     * class.
     *
     * NEW vs PICTags.
     */
    void setMatBusy(unsigned bank, unsigned way, bool busy);
    bool isMatBusy(unsigned bank, unsigned way) const;

    /**
     * Return the bank index (0–3) that owns @p addr.
     *
     * Bank selection uses log2(numBanks) bits immediately above the block
     * offset, interleaving cache lines across banks at 64-byte granularity.
     *
     * NEW vs PICTags (PICTags had no notion of banks), as are bankCount(),
     * sliceBytes(), blockSize() and activePICWays() below.
     */
    unsigned getBankIndex(Addr addr) const;

    /**
     * Return number of PIC ways currently in compute mode.
     * Useful for debugging and statistics collection.
     */
    unsigned activePICWays() const;

    /** Cache line size in bytes. Exposed so DPM can bound a single-block
     *  transfer without reaching into BaseTags' protected members. */
    unsigned blockSize() const { return blkSize; }

    /** Number of banks a line is striped across (paper default: 4). */
    unsigned bankCount() const { return numBanks; }

    /** Bytes per Mat/bank slice within one line (paper default: 16). */
    unsigned sliceBytes() const { return matSliceBytes; }

    // -----------------------------------------------------------------------
    // Flush support — called by Cache Controller on behalf of Switch
    // Controller when switching a way from cache mode to PIC mode.
    // NEW vs PICTags: getDirtyBlocksInWay() and invalidateWay().
    //
    // Sequence (Cache Controller is responsible for steps 2-4):
    //   1. getDirtyBlocksInWay(way)  → cache bank provides dirty block list
    //   2. For each dirty block: send writeback packet to DRAM
    //   3. For each block: send snoop/invalidation to L1 (cache coherence)
    //   4. Wait for all acks
    //   5. invalidateWay(way)        → cache bank clears all entries
    //   6. setWayPICMode(way, true)  → cache bank marks way as PIC mode
    // -----------------------------------------------------------------------

    /**
     * Scan way @p way across all sets and return every valid dirty block.
     *
     * The Cache Controller uses this list to issue writeback packets to DRAM.
     * Must be called before invalidateWay() and setWayPICMode().
     *
     * @param way  PIC way to flush (must not be normalCacheWay)
     * @return     All valid CacheBlks in that way that have DirtyBit set
     */
    std::vector<CacheBlk*> getDirtyBlocksInWay(unsigned way) const;

    /**
     * Return the block at a specific (way, set) coordinate.
     *
     * Used by the Switch Controller (DCF, Section 5.1) to directly query
     * the Directory by position rather than by address.
     * Replaces PICTags::getSetWayValid()/getSetWayAddr(), which scanned
     * every block on each call.
     *
     * @param way  Way index
     * @param set  Set index
     * @return     Pointer to the matching CacheBlk, or nullptr if not found
     */
    CacheBlk* getBlockByWaySet(unsigned way, unsigned set) const;

    /**
     * Invalidate every valid block in way @p way across all sets.
     *
     * Called by Cache Controller after:
     *   - all dirty blocks have been written back to DRAM, AND
     *   - L1 caches have been notified (snoop/invalidation acks received).
     *
     * Clears valid bits and updates replacement data so PIC compute
     * engine can safely overwrite the SRAM.
     *
     * @param way  Way to invalidate (must not be normalCacheWay)
     */
    void invalidateWay(unsigned way);

  protected:
    /** Number of cache banks (must be a power of 2; paper default: 4). */
    const unsigned numBanks;

    /**
     * Way that is permanently reserved for CPU use (paper default: way 0).
     * This way is never switched to PIC mode.
     */
    const unsigned normalCacheWay;

    /** Mats per bank (paper: 16). */
    const unsigned numMatsPerBank;

    /** SRAM sub-arrays per Mat (paper: 4). */
    const unsigned numSubArraysPerMat;

    /** Byte width of each Mat's data output (paper: 16 bytes = 128 bits). */
    const unsigned matSliceBytes;

    /**
     * Per-way mode bitmap.  Bit i set → way i is in PIC/compute mode.
     * Bit normalCacheWay is always 0 and is protected by setWayPICMode().
     * Up to 32 ways are supported (uint32_t).
     */
    uint32_t wayPICModeBitmap;

    /** matBusyBitmap[bank] bit i = way i busy (real-time access gate). */
    std::vector<uint32_t> matBusyBitmap;

    /** log2(blkSize) — used to locate bank-select bits in an address. */
    const unsigned blkShift;

    /** log2(numBanks) — number of address bits consumed by bank selection. */
    const unsigned bankBits;

  /*private:
    struct PICLLCTagStats : public statistics::Group
    {
        explicit PICLLCTagStats(PICLLCTags &parent);

        // Hits that land on the normal cache way (way 0).
        statistics::Scalar normalCacheHits;

        // CPU accesses blocked because the matching way is in PIC mode.
        statistics::Scalar modeBlockedAccesses;

        // Total calls to setWayPICMode (cache→PIC or PIC→cache).
        statistics::Scalar modeSwitches;

        // findVictim() calls that returned nullptr (no cache-mode victim).
        statistics::Scalar victimSearchFails;

        // invalidateWay() calls completed (one per successful way flush).
        statistics::Scalar wayFlushesCompleted;
    } picStats;*/
};

} // namespace gem5

#endif // __MEM_CACHE_TAGS_PIC_LLC_TAGS_HH__
