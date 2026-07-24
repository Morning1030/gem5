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

#ifndef __LEARNING_GEM5_SCHEDULER_HH__
#define __LEARNING_GEM5_SCHEDULER_HH__

#include <deque>
#include <string>

#include "mem/mshr_queue.hh"
#include "mem/port.hh"
#include "params/cache_controller.hh"
#include "sim/clocked_object.hh"

namespace gem5
{
    class PICTags : public BaseSetAssoc
    {
        public:
            CacheBlk* findVictim(const CacheBlk::KeyType& key,
                const std::size_t size,
                std::vector<CacheBlk*>& evict_blks,
                const uint64_t partition_id=0) override;

            bool getSetWayValid(const uint32_t setID, const uint32_t wayID);
            Addr PICTags::getSetWayAddr(const uint32_t setID, const uint32_t wayID);

        private:
            std::vector<bool> PIC_mode; // each element indicates one way

        protected:
    };

    class CacheController : public BaseCache
    {
        private:
            class CPUSidePort : BaseCache::CpuSidePort
            {
                private:
                    CacheController* owner;
                public:
                    CPUSidePort(const std::string& name, CacheController *owner);
                protected:
                    Tick recvAtomic(PacketPtr pkt) override {panic("recvAtomic unimplemented.");}
                    void recvFunctional(PacketPtr pkt) override;
                    bool recvTimingReq(PacketPtr pkt) override;
                    void recvRespRetry() override;
            };
            class MemSidePort : public BaseCache::MemSidePort
            {
                private:
                    CacheController *owner;
                public:
                    MemSidePort(const std::string& name, CacheController *owner);
                    void sendPacket(PacketPtr pkt);
                protected:
                    bool recvTimingResp(PacketPtr pkt) override;
                    void recvReqRetry() override;
                    void recvRangeChange() override;
            };

            CPUSidePort cpuSidePort;
            MemSidePort memSidePort;

            PICTags *tags;

        public:
            CacheController(CacheControllerParams *params);
            bool handleQueryWayState(PacketPtr pkt);
            bool handleFlushReq(PacketPtr pkt);
            void handleCache2PIC(PacketPtr pkt);
            void handlePIC2Cache(PacketPtr pkt);


        protected:
    };


}
