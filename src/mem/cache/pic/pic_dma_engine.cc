#include "mem/cache/pic/pic_dma_engine.hh"

#include <algorithm> // std::min
#include <cstring> // std::memcpy
#include <utility> // std::move

#include "base/logging.hh" // gem5 handle panic, fatal, warn
#include "base/trace.hh" // gem5 handle DPRINTF
#include "debug/PICDMA.hh" //come up after compile with debug flags

namespace gem5
{

namespace
{
constexpr Addr PageBytes = 4096; //default RISC-V page size, used for page splitting
}  // namespace, only visible in this file, wont conflict with other outer files

std::deque<AddrChunk>
splitRequests(const DMADescriptor &desc, uint32_t maxChunkBytes,
              uint32_t blockBytes)
{
    std::deque<AddrChunk> out;

    if (desc.bytesPerRow == 0 || desc.numRows == 0)
        return out;

    if (desc.rowStrideBytes != 0 &&
        desc.rowStrideBytes < desc.bytesPerRow) {
        warn("splitRequests: rowStride %llu < bytesPerRow %u",
             (unsigned long long)desc.rowStrideBytes, desc.bytesPerRow);
        return out;
    }

    for (uint32_t row = 0; row < desc.numRows; ++row) // outer loop : every row
    {
        const Addr rowStart = desc.dramVaddr + row * desc.rowStrideBytes; // the starting addr of this row (why we need stride)
        uint32_t done = 0;

        while (done < desc.bytesPerRow) // inner loop : every chunk in a row
        {
            const Addr v = rowStart + done;

            // Three independent ceilings.  Whichever bites first wins.
            const Addr pageEnd  = (v & ~(PageBytes - 1)) + PageBytes;
            const Addr blockEnd = (v & ~Addr(blockBytes - 1)) + blockBytes;

            uint32_t n = desc.bytesPerRow - done;
            n = std::min<uint64_t>(n, pageEnd - v);
            n = std::min<uint64_t>(n, blockEnd - v);
            n = std::min(n, maxChunkBytes);

            out.push_back(AddrChunk{v, n, row, done}); // store the chunk info into the deque, including vaddr, size, rowIdx, rowOffset
            done += n;
        }
    }

    return out;
}

DMAEngine::DMAEngine(Host _host) : host(std::move(_host)) {}

uint32_t
DMAEngine::startOp(P2SOpState op)
{
    op.pending = splitRequests(op.desc, host.chunkBytes, host.blockBytes);
    if (op.pending.empty())
        return 0;

    const uint32_t opId = nextOpId++;
    op.opId = opId;

    op.buf.assign(op.desc.totalBytes(), 0);
    op.rowRemaining.assign(op.desc.numRows, op.desc.bytesPerRow);
    op.groupRemaining.assign(op.numGroups(), op.rowsPerGroup);

    DPRINTF(PICDMA, "op %u: %u rows x %u B -> %zu chunks%s\n",
            opId, op.desc.numRows, op.desc.bytesPerRow, op.pending.size(),
            op.desc.rowsContiguous() ? " (contiguous)" : " (strided)");

    opTable.emplace(opId, std::move(op));
    return opId;
}

Tick
DMAEngine::issue()
{
    if (blocked)
        return MaxTick;

    Tick nextWake = MaxTick;

    for (auto &entry : opTable) {
        P2SOpState &op = entry.second;

        while (!op.pending.empty()) {
            if (op.outstanding >= host.maxOutstanding) {
                nextWake = std::min(nextWake, host.nextCycleTick());
                break;
            }

            const AddrChunk &c = op.pending.front();

            // Backpressure.  The sliding window is exactly the modelled
            // buffer depth: with a shallow FIFO (P2S_R_T) the engine
            // stalls long before it can saturate the memory system, which
            // is the whole point of keeping the depth parameterised.
            const uint32_t group = c.rowIdx / op.rowsPerGroup;
            const uint32_t windowEnd = op.groupsCompleted + host.bufRows(op.dest);
            if (group >= windowEnd) {
                DPRINTF(PICDMA, "op %u: stalled on buffer credits "
                        "(group %u, window ends %u)\n",
                        op.opId, group, windowEnd);
                break;
            }

            // Serialised TLB port.  SE mode would otherwise translate for
            // free, hiding the cost that page splitting actually incurs.
            if (!host.tlbPipelined && curTick() < tlbFreeTick) {
                nextWake = std::min(nextWake, tlbFreeTick);
                break;
            }
            tlbFreeTick = curTick() + host.tlbLatencyTicks;

            // Translated here, per chunk, at the moment it's actually
            // sent -- not up front in splitRequests() -- so tlbFreeTick
            // above lands where the RTL pays the TLB round trip, and a
            // future TLB-miss model has somewhere to hook in.
            Addr paddr = 0;
            panic_if(!host.translate(c.vaddr, paddr),
                     "DMAEngine::issue: op %u translation failed for "
                     "vaddr %#x", op.opId, c.vaddr);

            PacketPtr pkt = host.buildReadPacket(paddr, c); // build packet
            pkt->pushSenderState(new PICDMASenderState(
                op.opId, c.rowIdx, c.rowOffset, c.size)); // add lable for this packet (the position for data coming back)

            DPRINTF(PICDMA, "op %u: issue paddr %#x size %u "
                    "(row %u off %u)\n",
                    op.opId, paddr, c.size, c.rowIdx, c.rowOffset);

            if (!host.sendDMAReq(pkt)) {
                // Port refused: hold the packet, wait for a retry.
                retryPkt = pkt;
                blocked = true;
                op.pending.pop_front();
                op.outstanding++;
                return MaxTick;
            }

            op.pending.pop_front();
            op.outstanding++;
        }
    }

    return nextWake;
}

void
DMAEngine::retry()
{
    if (retryPkt) {
        if (!host.sendDMAReq(retryPkt)) {
            // Still refused; stay blocked and wait for the next
            // recvReqRetry().
            DPRINTF(PICDMA, "retry: port refused retryPkt again, "
                    "still blocked\n");
            return;
        }
        DPRINTF(PICDMA, "retry: resent held packet\n");
        retryPkt = nullptr;
    }
    blocked = false;
}

bool
DMAEngine::handleResponse(PacketPtr pkt)
{
    auto *ss = safe_cast<PICDMASenderState *>(pkt->popSenderState());
    P2SOpState *op = findOp(ss->opId);
    panic_if(!op, "DMA response for unknown op %u", ss->opId);

    const uint32_t slot = ss->rowIdx * op->desc.bytesPerRow + ss->rowOffset;
    panic_if(slot + ss->size > op->buf.size(),
             "op %u: reassembly slot %u+%u exceeds buffer %zu",
             op->opId, slot, ss->size, op->buf.size());

    std::memcpy(op->buf.data() + slot, pkt->getConstPtr<uint8_t>(), ss->size);

    op->outstanding--;
    panic_if(op->rowRemaining[ss->rowIdx] < ss->size,
             "op %u: row %u over-filled", op->opId, ss->rowIdx);
    op->rowRemaining[ss->rowIdx] -= ss->size;

    const bool rowDone = (op->rowRemaining[ss->rowIdx] == 0);
    const uint32_t rowIdx = ss->rowIdx;
    const uint32_t group  = rowIdx / op->rowsPerGroup;

    bool groupDone = false;
    if (rowDone) {
        panic_if(op->groupRemaining[group] == 0,
                 "op %u: group %u over-filled", op->opId, group);
        groupDone = (--op->groupRemaining[group] == 0);
    }

    DPRINTF(PICDMA, "op %u: resp row %u off %u size %u%s%s\n",
            op->opId, rowIdx, ss->rowOffset, ss->size,
            rowDone ? " ROW-DONE" : "", groupDone ? " GROUP-DONE" : "");

    delete ss;
    delete pkt;

    if (groupDone)
        host.groupReady(*op, group);

    return groupDone;
}

P2SOpState *
DMAEngine::findOp(uint32_t opId)
{
    auto it = opTable.find(opId);
    return it == opTable.end() ? nullptr : &it->second;
}

}  // namespace gem5
