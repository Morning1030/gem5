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

#include "mem/port.hh"
#include "params/Scheduler.hh"
#include "sim/clocked_object.hh"

namespace gem5
{
// datapayload for switch
struct QueryPayload
{
    uint32_t setID;
    uint32_t wayID;
};

struct RespPayload
{
    uint64_t addr;
    uint32_t state;
};
struct P2S_L_Payload
{
    uint64_t base_dramAddr_to_load;
    uint64_t base_picAddr_to_store;
    uint32_t next_row_offset_elem;  // the low 15 bits
    uint8_t _L_block_row;           // 8 bits
    uint8_t precision;              // 3 bits
};
// datapayload for p2s
struct P2S_R_Payload
{
    uint64_t dramAddr;
    uint64_t base_arrayID_to_store; // Which subarray to put the first selected bit map
    uint32_t next_row_offset_bytes;                                 // 15bits
    uint32_t nRows                          ;                        // Read how many rows
    uint32_t nCols;                                                 // Number of columns to read, max 1024
    uint8_t precision;    
    uint8_t bufNum;                                                 // 2 bits
};


class Scheduler : public ClockedObject
{
    private:
    /*
    val LOAD_ID=0
        val P2SL_ID=1
        val P2SR_ID=2
        val P2SRT_ID=3
        val IM2COL_ID=4
        val ACC_ID=5
        val EXE_ID=6
        val STORE_ID=7
        val SWITCH_ID=8
        val QUERY_ID=9

        val totalModule=10
    */
        enum class FuncID {LOAD, STORE, P2S_L, P2S_R, P2S_R_T, ACC, CAL, SWITCH, QUERY};
        struct Task
        {
            FuncID funcID;
            PacketPtr pkt;
        };
        // to interact with MMIO request
        class CPUSidePort : public ResponsePort
        {
            private:
                Scheduler *owner;

            public:
                CPUSidePort(const std::string& name, Scheduler *owner);
                void sendPacket(PacketPtr pkt);
                AddrRangeList getAddrRanges() const override;

            // there are three modes of the port
            protected:
                Tick recvAtomic(PacketPtr pkt) override {panic("recvAtomic unimplemented.");}
                void recvFunctional(PacketPtr pkt) override;
                bool recvTimingReq(PacketPtr pkt) override;
                void recvRespRetry() override;
        };
        // to interact with Cache controller
        class MemSidePort : public RequestPort
        {
            private:
                // corresponds to each direct port
                enum class PortID {CC, DPM, CB};
                Scheduler *owner;
                PortID portID;
            public:
                MemSidePort(const std::string& name, Scheduler *owner);
                void sendPacket(PacketPtr pkt);

            protected:
                bool recvTimingResp(PacketPtr pkt) override;
                void recvReqRetry() override;
                void recvRangeChange() override;
        };

        class TaskScheduler
        {
            private:
                enum class TaskState
                {
                    IDLE,
                    SWITCHING,
                    P2SING,
                    CALING,
                    ACCING
                };
                Scheduler *owner;
                std::deque<Task> nextTask;
                std::deque<Task> nextImmTask;   // switch is immTask
                TaskState currState;
                
                EventFunctionWrapper p2sLEvent;
                EventFunctionWrapper p2sREvent;
                EventFunctionWrapper p2sRTEvent;


                void prepareTask(PacketPtr paramPkt, uint64_t src, uint64_t dst, uint16_t row, uint16_t byte_per_row, uint16_t offset);
                void triggerTS(Task t);
                void processP2SLEvent();
                void processP2SREvent();
                void processP2SRTEvent();

            public:
                TaskScheduler(Scheduler *owner);
               
            protected:
        };

        class SwitchController
        {
            private:
                enum class SwitchType {PIC2Cache, Cache2PIC};
                Scheduler *owner;
                uint32_t setID;
                uint32_t wayID;
                uint32_t wayStateValid;
                Addr flushAddr;
                SwitchType currSwitchType;

                EventFunctionWrapper switchEvent;
                EventFunctionWrapper queryEvent;
                EventFunctionWrapper requestFlushEvent;
                EventFunctionWrapper switch2PICEvent;
                EventFunctionWrapper switch2CacheEvent;
                void processSwitchEvent();
                void processQueryEvent();
                void processRequestFlushEvent();
                void processSwitch2PICEvent();
                void processSwitch2CacheEvent();

            public:
                SwitchController(Scheduler *owner);
            protected:

        };

        CPUSidePort instPort;
        MemSidePort cacheControllerPort; // cache controller direct port
        MemSidePort DPMPort;             // DPM direct port
        MemSidePort cacheBankPort;       // cache bank direct port
        TaskScheduler taskScheduler;
        SwitchController switchController;

        std::deque<PacketPtr> instQueue;
        // register file data to store the params
        uint64_t src;                   // SET_SRC
        uint64_t dst;                   // SET_DST
        uint16_t row;                   // SET_SIZE
        uint16_t byte_per_row;          // SET_SIZE
        uint16_t offset;                // SET_SIZE                 

        size_t maxInstQueueSize = 1000; // temporarily set to 1000
        EventFunctionWrapper decodeEvent;
        
        void processDecodeEvent();

    public:
        Scheduler(SchedulerParams *params);

        Port &getPort(const std::string &if_name, PortID idx=InvalidPortID) override;
        AddrRangeList getAddrRanges() const;
        void sendRangeChange();
        void handleFunctional(PacketPtr pkt);
        bool handleRequest(PacketPtr pkt);
        bool handleResponse(PortID portID, PacketPtr pkt);
        void startup() override;

    /**
     * Part of a SimObject's initilaization. Startup is called after all
     * SimObjects have been constructed. It is called after the user calls
     * simulate() for the first time.
     */
    // void startup() override;
};


} // namespace gem5

#endif // __LEARNING_GEM5_SCHEDULER_HH__
