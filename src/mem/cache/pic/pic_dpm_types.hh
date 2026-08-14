#ifndef __MEM_CACHE_PIC_PIC_DPM_TYPES_HH__
#define __MEM_CACHE_PIC_PIC_DPM_TYPES_HH__

#include <cstdint>
#include <deque>
#include <vector>

#include "base/types.hh"
#include "mem/packet.hh"

namespace gem5
{

/**
 * Destination buffer tag.  Mirrors DataDir in the RTL.
 *
 * DMA-scheduling state: P2SOpState::dest feeds DMAEngine::issue()'s call to
 * Host::bufRows(op.dest), which sizes the reassembly credit window.
 */
enum class P2SDest : uint8_t
{
    L  = 0,
    R  = 1,
    RT = 2,
};

/**
 * Two-dimensional DMA descriptor: @a numRows runs of @a bytesPerRow bytes,
 * each run starting @a rowStrideBytes after the previous one.
 *
 * This single shape serves both the transposed and the non-transposed
 * path; only the parameterisation differs.  For a 64-bitline sub-array
 * holding @a elemBytes wide elements:
 *
 *   t=false : bytesPerRow = 64 * elemBytes, numRows = N_R_col
 *   t=true  : bytesPerRow = elemBytes,      numRows = 64  (strided gather)
 *
 * The DMA engine itself is deliberately unaware of bitWidth and
 * transpose; it only ever sees this descriptor.  That mirrors the RTL,
 * where the DMA engine does not know the precision either.
 */
struct DMADescriptor
{
    Addr     dramVaddr      = 0;
    Addr     onChipAddr     = 0;
    uint32_t bytesPerRow    = 0;
    uint32_t numRows        = 0;
    uint64_t rowStrideBytes = 0;

    bool     rowsContiguous() const
    {
        return rowStrideBytes == bytesPerRow;
    }

    uint64_t totalBytes() const
    {
        return uint64_t(bytesPerRow) * numRows;
    }
};

/**
 * One physically contiguous piece of one row, after page splitting.
 *
 * Carries a virtual address, not physical: splitting is pure address
 * arithmetic (page boundaries fall out of the low bits of the vaddr) and
 * has no need to translate. Translation happens later, per chunk, right
 * before it's sent -- see DMAEngine::issue().
 */
struct AddrChunk
{
    Addr     vaddr      = 0;
    uint32_t size       = 0;
    uint32_t rowIdx     = 0;   //!< which descriptor row this belongs to
    uint32_t rowOffset  = 0;   //!< byte offset within that row
};

/**
 * Per-operation state.  Metadata lives here for the whole lifetime of the
 * op and never travels with the data: DMA response packets carry only an
 * opId and a buffer offset, and the consumer looks the rest back up.
 *
 * DMAEngine itself reads/writes opId, desc, pending, buf, rowRemaining,
 * rowsPerGroup, groupRemaining, outstanding, and dest (via
 * Host::bufRows(op.dest)). `bitWidth`, `transpose`, `groupsCompleted`,
 * `pendingWrites`, and `finished()` are consumed downstream of the DMA
 * engine, once a completed group is handed off.
 */
struct P2SOpState
{
    uint32_t opId       = 0;
    bool     transpose  = false;
    uint8_t  bitWidth   = 8;
    P2SDest  dest       = P2SDest::R;

    DMADescriptor desc;

    /** Chunks still waiting to be issued. */
    std::deque<AddrChunk> pending;

    /** Reassembly storage, indexed [rowIdx * bytesPerRow + rowOffset]. */
    std::vector<uint8_t> buf;

    /** Bytes still missing per row; a row is ready when this hits zero. */
    std::vector<uint32_t> rowRemaining;

    /**
     * How many descriptor rows make up one array wordline group.
     *
     *   t=false : 1   (one contiguous run already spans the 64 bitlines)
     *   t=true  : 64  (strided gather; each descriptor row is one element)
     *
     * Getting this into the structure now is what keeps P2S_R_T a
     * parameterisation rather than a rewrite.
     */
    uint32_t rowsPerGroup = 1;

    /** Descriptor rows still missing per group. */
    std::vector<uint32_t> groupRemaining;

    /** Groups handed to the P2S unit so far.  Drives the credit window. */
    uint32_t groupsCompleted = 0;

    uint32_t numGroups() const
    {
        return desc.numRows / rowsPerGroup;
    }

    /** DMA reads issued but not yet returned. */
    uint32_t outstanding = 0;

    /** Wordline writes still queued in the bank arbiter. */
    uint32_t pendingWrites = 0;

    bool dmaDone() const
    {
        return pending.empty() && outstanding == 0;
    }

    bool finished() const
    {
        return dmaDone() && groupsCompleted == numGroups() &&
               pendingWrites == 0;
    }
};

/**
 * Sender state attached to every DMA read packet.  Intentionally tiny:
 * enough to find the op and the slot, nothing more.
 */
class PICDMASenderState : public Packet::SenderState
{
  public:
    PICDMASenderState(uint32_t _opId, uint32_t _rowIdx, uint32_t _rowOffset,
                      uint32_t _size)
        : opId(_opId), rowIdx(_rowIdx), rowOffset(_rowOffset), size(_size)
    {}

    const uint32_t opId;
    const uint32_t rowIdx;
    const uint32_t rowOffset;
    const uint32_t size;
};

}  // namespace gem5

#endif  // __MEM_CACHE_PIC_PIC_DPM_TYPES_HH__
