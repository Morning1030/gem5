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
 * Where ReadPostProcess sends loaded data. Mirrors DataDir in
 * ReadPostProcess.scala.
 */
enum class DataDir : uint8_t
{
    Bank  = 0,  //!< write into the cache banks through AccessBankArb
    P2SL  = 1,  //!< P2S_L input buffer
    P2SR  = 2,  //!< P2S_R input buffer
    P2SRT = 3,  //!< P2S_R_T input buffer
};

/**
 * One load request into Load_ctl. Mirrors Load_req in Load_ctl.scala.
 * Padding (PadInfo) is carried but not implemented downstream yet.
 */
struct LoadReq
{
    uint32_t bytePerRow = 0;  //!< bytes to fetch per row
    uint32_t row = 0;         //!< number of rows
    uint32_t offset = 0;      //!< DRAM stride between rows, in bytes
    Addr baseAddrDram = 0;    //!< virtual address of the first row
    Addr baseAddrArray = 0;   //!< array address (64-bit words) to write to
    bool padding = false;
    uint32_t padSize = 0;
    DataDir dataDir = DataDir::Bank;
};

/**
 * One row read, Load_ctl -> DMA_read_page_split. Mirrors DMA_read_req in
 * DMA_read_page_split.scala. lastRow is the separate io.lastRow wire in
 * the RTL, carried with the request here.
 */
struct DMAReadReq
{
    uint64_t nByte;
    uint64_t baseAddrDram;
    uint64_t lastRow;
};

/**
 * Load_ctl -> ReadPostProcess setup. Mirrors ReadPostProcess_req in
 * ReadPostProcess.scala.
 */
struct ReadPostProcessReq
{
    uint64_t bankBaseAddr;
    uint64_t bankEndAddr;
    uint32_t padding;
    uint32_t padSize;
    uint32_t dataDir;
    uint32_t reserved;
};

/**
 * One 64-bit beat of the DMA_read_page_split -> ReadPostProcess stream.
 * Mirrors StreamChannel(64): keep has one bit per valid byte; last marks
 * the final beat of the whole load.
 */
struct StreamBeat
{
    uint64_t data;
    uint8_t keep;
    uint8_t last;
    uint8_t pad[6];
};

/**
 * mod: p2s_arbiter
 * Wire format for one bit-plane WRITE, carried as the data payload of the
 * MemCmd::WriteReq packet a P2S engine sends on its cb_port. Shared here
 * (rather than duplicated per module) because both sides of that link --
 * learning_gem5/PIC/p2s.hh's P2S_L/P2S_R/P2S_R_T and
 * mem/cache/pic/access_bank_arb.hh's AccessBankArb -- need the identical
 * layout: P2S builds it, AccessBankArb::handleRequest() parses it into a
 * ReqPackage. Mirrors bankAccessScheduler.scala's ReqPackage minus optype
 * (P2S ever only sends WRITE, so it isn't carried on the wire).
 */
struct P2SWritePayload
{
    uint64_t arrayAddr;   //!< 17-bit array address, widened for alignment.
    uint64_t bitSlice;    //!< 64-bit bit-plane payload.
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
 * before it's sent -- see DMA_read_page_split::issue().
 */
struct AddrChunk
{
    Addr     vaddr      = 0;
    uint32_t size       = 0;
    uint32_t rowIdx     = 0;   //!< which descriptor row this belongs to
    uint32_t rowOffset  = 0;   //!< byte offset within that row
};

/**
 * Sender state on a DMA read: which chunk of the current page segment
 * it is, so responses can be put back in order.
 */
class PICDMASenderState : public Packet::SenderState
{
  public:
    explicit PICDMASenderState(uint32_t _chunkIdx) : chunkIdx(_chunkIdx) {}

    const uint32_t chunkIdx;
};

}  // namespace gem5

#endif  // __MEM_CACHE_PIC_PIC_DPM_TYPES_HH__
