

/**
 * @file
 * PIC-LLC (Processing-In-Cache Last Level Cache) tag store.
 *
 * Implements the cache bank architecture from the Polymorphic PIC paper
 * (Section 4.1.3):
 *
 *   Physical organization per bank:
 *     - 16 Mats per bank, each Mat holding 4 SRAM sub-arrays
 *     - Each Mat stores a 16-byte slice of one way across all sets
 *     - SRAM: 64-bit I/O width, 512-wordline configuration
 *     - Total banks: 4 (Bank 0-3)
 *
 *   Way partitioning:
 *     - Way 0  : Normal Cache way — always available for CPU use
 *     - Ways 1–15 : PIC ways — can be in cache mode or PIC/compute mode
 *     - Mode switching granularity: one way across all four banks
 *       (called a "Level" in the paper, Fig. 6(c))
 *
 *   Arbiter:
 *     Per the PolymorPIC paper, the Arbiter is the path selector that
 *     decides, per cache bank, whether the CPU or the PIC/compute path may
 *     touch a way's data -- and enforces isolation between the two (the
 *     PAI mechanism) so ordinary CPU/OS traffic can't land on a way that's
 *     been allocated to AI compute. That's the half implemented here:
 *     - On every tag lookup the arbiter checks wayPICModeBitmap
 *     - If the matched way is currently in PIC mode, CPU access is blocked
 *       (access returns nullptr); the cache controller must retry later
 *     - PIC-mode ways are also excluded from replacement (findVictim)
 *     The *other* half of the paper's Arbiter -- routing DRAM data toward
 *     a p2s engine and routing p2s results into the right Mat, i.e. who
 *     gets the shared physical bank port this cycle -- needs no tag/way
 *     state at all, so it does not live here: see AccessBankArb in
 *     mem/cache/pic/access_bank_arb.hh.
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
 */
class PICLLCTags : public BaseSetAssoc
{
  public:
    typedef PICLLCTagsParams Params;

    PICLLCTags(const Params &p);
    ~PICLLCTags() = default;

    /**
     * Tag lookup with PIC-mode arbiter.
     *
     * If the matching block lives in a way that is currently in PIC mode,
     * the arbiter denies CPU access and returns nullptr (the hit is invisible
     * to the CPU until the way is returned to cache mode).
     */
    CacheBlk* accessBlock(const PacketPtr pkt, Cycles &lat) override;

    /**
     * Victim selection restricted to cache-mode ways.
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
     */
    void setWayPICMode(unsigned way, bool pic);

    /** Return true if @p way is currently in PIC/compute mode. */
    bool isWayInPICMode(unsigned way) const;

    /**
     * Return the bank index (0–3) that owns @p addr.
     *
     * Bank selection uses log2(numBanks) bits immediately above the block
     * offset, interleaving cache lines across banks at 64-byte granularity.
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
