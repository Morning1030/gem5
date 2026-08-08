/*
 * Copyright (c) 2017 Jason Lowe-Power
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "base/trace.hh"
#include "debug/CacheController.hh"

#include "learning_gem5/PIC/cache_controller.hh"
#include "learning_gem5/PIC/scheduler.hh"

namespace gem5
{

CacheBlk*
PICTags::findVictim(const CacheBlk::KeyType& key,
                const std::size_t size,
                std::vector<CacheBlk*>& evict_blks,
                const uint64_t partition_id=0) override
{
    // Get possible entries to be victimized
    std::vector<ReplaceableEntry*> entries =
        indexingPolicy->getPossibleEntries(key);

    // !! Filter entries based on pic mode
    auto it = entries.begin();
    while (it != entries.end()) {
        CacheBlk* blk = static_cast<CacheBlk*>(*it);
        if (blk && isWayPICMode(blk->getWay())) {
            it = entries.erase(it);
        } else {
            ++it;
        }
    }

    // Filter entries based on PartitionID
    if (partitionManager) {
        partitionManager->filterByPartition(entries, partition_id);
    }

    // Choose replacement victim from replacement candidates
    CacheBlk* victim = entries.empty() ? nullptr :
        static_cast<CacheBlk*>(replacementPolicy->getVictim(entries));

    // There is only one eviction for this replacement
    evict_blks.push_back(victim);

    return victim;
}

bool
PICTags::getSetWayValid(const uint32_t setID, const uint32_t wayID) {
    unsigned index = setID * indexingPolicy->assoc + wayID;
    return blks[index]->isValid();
}
Addr
PICTags::getSetWayAddr(const uint32_t setID, const uint32_t wayID) {
    unsigned index = setID * indexingPolicy->assoc + wayID;
    return blk[index]->getAddr();
}


CacheController::CacheController(CacheControllerParams *params) :
    ClockedObject(params),
    instPort(params.name + ".cpu_port", this),
    memPort(params.name + ".mem_port", this)
{
}
bool
CacheController::CPUSidePort::recvTimingReq(PacketPtr pkt) {
    // classify by flag then forward to other functions
    // TODO need to go to request.hh to add in new request type
    if (pkt->req->isQueryWay) return owner->handleQueryWayState(pkt);
    else if (pkt->req->isFlushWay) return owner->handleFlushReq(pkt);
    else if (pkt->req->isCache2PIC) return owner->handleCache2PIC(pkt);
    else if (pkt->req->isPIC2Cache) return owner->handlePIC2Cache(pkt);
}

bool
CacheController::handleQueryWayState(PacketPtr pkt) {
    // decode setID and wayID
    QueryPayload qPayload;

    pkt->writeData(reinterpret_cast<uint8_t*>(&qPayload));
    uint32_t setID = qPayload.setID;
    uint32_t wayID = qPayload.wayID;

    // pack valid and addr into payload
    bool valid = getSetWayValid(setID, wayID);
    Addr addr = getSetWayAddr(setID, wayID);

    RespPayload respPayload;
    respPayload.state = valid;
    respPayload.addr = addr;

    pkt->makeResponse();

    pkt->setData(reinterpret_cast<const uint8_t*>(&respPayload));

    bool success = cpuSidePort.sendTimingResp(pkt);
    if (!success) {
        // send it to the retry queue
    }

    return true;
}

bool
CacheController::handleFlushReq(PacketPtr pkt) {

    // decode flush address from packet
    Addr flushAddr = pkt->getLE<Addr>;

    // flush the (setID, wayID) cache block / cache line
    CacheBlk *blk = tags->findBlock(flushAddr, false);

    if (blk && blk->isValid()) {

        if (blk->isDirty()) {
            // writeback to DRAM
            PacketPtr wb_pkt = writebackBlk(blk);
            allocateWriteBuffer(wb_pkt, curTick());
        }

        invalidateBlock(blk);
        // invalidateSharers(blk);
    }
    if (pkt->needsResponse()) {
        pkt->makeResponse();
        bool success = cpuSidePort.sendTimingResp(pkt);
        if (!success) {
            // responseRetryQueue.push_back(pkt);
        }
    }
    else delete pkt;
    return true;
}

bool
CacheController::handleCache2PIC(pkt) {
    // switch PIC_model[wayID] from cache mode to PIC mode
    uint32_t wayID = pkt->getLE<uint32_t>;
    tags->PIC_mode[wayID] = true;

    if (pkt->needsResponse()) {
        pkt->makeResponse();
        bool success = cpuSidePort.sendTimingResp(pkt);
        if (!success) {
            // responseRetryQueue.push_back(pkt);
        }
    }
    else delete pkt;
    return true;

}

bool
CacheController::handlePIC2Cache(pkt) {
    // switch PIC_model[wayID] from PIC mode to cache mode
    uint32_t wayID = pkt->getLE<uint32_t>;
    tags->PIC_mode[wayID] = true;

    if (pkt->needsResponse()) {
        pkt->makeResponse();
        bool success = cpuSidePort.sendTimingResp(pkt);
        if (!success) {
            // responseRetryQueue.push_back(pkt);
        }
    }
    else delete pkt;
    return true;
}
}
