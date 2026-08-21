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

#include "learning_gem5/PIC/p2s.hh"
#include "mem/port.hh"
#include "params/p2s.hh"
#include "sim/clocked_object.hh"

namespace gem5
{
    struct DMALPayload{
        // row = 1;
        // byte_per_row = 512;
        // baseAddr_Array = 0;
        uint32_t offset;
        uint64_t baseAddr_DRAM;
    };
    struct DMARPayload{
        uint32_t byte_per_row;  // cols
        uint32_t offset;
        uint64_t baseAddr_DRAM;
    };
    struct DMARTPayload{
        // row = 1;
        uint32_t byte_per_row;  // cols
        uint64_t baseAddr_DRAM;
    };
    struct P2SWritePayload {
        uint64_t arrayAddr;
        uint64_t bitSlice;
    };

    class P2S_L : public ClockedObject {
        private:
            // to interact with scheduler
            class CPUSidePort : public ResponsePort
            {
                private:
                    P2S_L *owner;

                public:
                    CPUSidePort(const std::string& name, Scheduler *owner);
                    void sendPacket(PacketPtr pkt);

                // there are three modes: Atomic, Functional and Timing
                protected:
                    Tick recvAtomic(PacketPtr pkt) override {panic("recvAtomic unimplemented.");}
                    void recvFunctional(PacketPtr pkt) override;
                    bool recvTimingReq(PacketPtr pkt) override;
                    void recvRespRetry() override;
            };
            // to interact with DMA and Cache Bank
            class MemSidePort : public RequestPort
            {
                private:
                    // corresponds to each direct port
                    enum class PortID {DMA, CB};
                    P2S_L *owner;
                    PortID portID;
                public:
                    MemSidePort(const std::string& name, Scheduler *owner);
                    void sendPacket(PacketPtr pkt);

                protected:
                    bool recvTimingResp(PacketPtr pkt) override;
            };
            // direct port
            CPUSidePort instPort;
            MemSidePort DMAPort;
            MemSidePort CacheBankPort;

            // variables
            uint64_t base_dram_addr;
            uint64_t base_picAddr;
            uint64_t pic_write_ptr;
            uint32_t next_row_offset_elem;
            uint32_t next_row_offset_dram;
            uint8_t _L_block_row;
            uint8_t _L_block_row_ptr;
            uint8_t next_slice_offset_pic;
            uint8_t precision;
            uint8_t bit_ptr;

            uint8_t dmaRow;

            // buffer to store data accessed from DMA
            std::vector<std::vector<uint8_t>> regArray(8, std::vector<uint8_t>(8, 0)); // 8 * 8

            // write bank queue
            std::vector<PacketPtr> bitSliceQueue;

            // events
            EventFunctionWrapper dmaReadEvent;
            EventFunctionWrapper bitSliceEvent;
            EventFunctionWrapper writeEvent;
            
        protected:
        public:
            P2S_L(P2S_LParams *params);
            bool handleP2SRequest(PacketPtr pkt);
            uint64_t extractBits_L(const std::vector<std::vector<uint8_t>> &arr, uint8_t bit);
            void processWriteEvent();
    };

    class P2S_R : public ClockedObject {
        private:
            class CPUSidePort : public ResponsePort
            {
                private:
                    P2S_R *owner;

                public:
                    CPUSidePort(const std::string& name, Scheduler *owner);
                    void sendPacket(PacketPtr pkt);

                // there are three modes: Atomic, Functional and Timing
                protected:
                    Tick recvAtomic(PacketPtr pkt) override {panic("recvAtomic unimplemented.");}
                    void recvFunctional(PacketPtr pkt) override;
                    bool recvTimingReq(PacketPtr pkt) override;
                    void recvRespRetry() override;
            };
            // to interact with DMA and Cache Bank
            class MemSidePort : public RequestPort
            {
                private:
                    // corresponds to each direct port
                    enum class PortID {DMA, CB};
                    P2S_R *owner;
                    PortID portID;
                public:
                    MemSidePort(const std::string& name, Scheduler *owner);
                    void sendPacket(PacketPtr pkt);

                protected:
                    bool recvTimingResp(PacketPtr pkt) override;
            };
            // direct port
            CPUSidePort instPort;
            MemSidePort DMAPort;
            MemSidePort CacheBankPort;

            // request params
            uint64_t dramAddr;
            uint64_t base_arrayID_to_store; // Which subarray to put the first selected bit map
            uint32_t next_row_offset_bytes;                                 // 15bits
            uint32_t nRows                          ;                        // Read how many rows
            uint32_t nCols;                                                 // Number of columns to read, max 1024
            uint8_t precision;    
            uint8_t bufNum;

            // variables for bitslice
            uint32_t curBlockColPtrGlobal;
            uint32_t blockNColInMem;
            uint32_t curBufColPtrInBlock;
            uint32_t curBufNCols;
            uint32_t curEnqBlockInBufColPtr;
            uint32_t bit_ptr;

            // variables for dma request (each block)
            uint32_t curBlockDramBaseAddrPtr;
            uint32_t curRowDramAddrOffset;
            uint32_t curBlockNRows;
            uint32_t curBlockNCols;
            uint32_t curBlockColPtr;
            uint32_t curBlockRowPtr;
            uint32_t writeBufRowPtr;
            uint32_t readMemAddr;
            uint32_t writeMemAddr;

            // since each element is 8 bit, arrayID_offset has 8 elements corresponding to each bit
            std::vector<uint8_t> relative_offset_buf(7);
            std::vector<uint8_t> arrayID_offset(8);
            std::vector<std::vector<uint8_t>> mem0(64, std::vector<uint8_t>(128, 0));      // SRAM block is 64 * 128
            std::vector<std::vector<uint8_t>> regArray(64, std::vector<uint8_t>(8, 0));    // p2s_R it's 64 * 8
            // write bank queue
            std::vector<PacketPtr> bitSliceQueue;
            // events
            EventFunctionWrapper writeEvent;
        protected:
        public:
            P2S_R(P2S_RParams *params);
            bool handleP2SRequest(PacketPtr pkt);
            void extractBits_R(const std::vector<std::vector<uint8_t>> &arr, uint32_t row, uint8_t bit, uint32_t dim);
            void processWriteEvent();
            
    };

    class P2S_R_T : public ClockedObject{
        private:
            class CPUSidePort : public ResponsePort
            {
                private:
                    P2S_R_T *owner;

                public:
                    CPUSidePort(const std::string& name, Scheduler *owner);
                    void sendPacket(PacketPtr pkt);

                // there are three modes: Atomic, Functional and Timing
                protected:
                    Tick recvAtomic(PacketPtr pkt) override {panic("recvAtomic unimplemented.");}
                    void recvFunctional(PacketPtr pkt) override;
                    bool recvTimingReq(PacketPtr pkt) override;
                    void recvRespRetry() override;
            };
            // to interact with DMA and Cache Bank
            class MemSidePort : public RequestPort
            {
                private:
                    // corresponds to each direct port
                    enum class PortID {DMA, CB};
                    P2S_R_T *owner;
                    PortID portID;
                public:
                    MemSidePort(const std::string& name, Scheduler *owner);
                    void sendPacket(PacketPtr pkt);

                protected:
                    bool recvTimingResp(PacketPtr pkt) override;
            };
            // direct port
            CPUSidePort instPort;
            MemSidePort DMAPort;
            MemSidePort CacheBankPort;

            // request params
            uint64_t dramAddr;
            uint64_t base_arrayID_to_store; // Which subarray to put the first selected bit map
            uint32_t next_row_offset_bytes;                                 // 15bits
            uint32_t nRows                          ;                        // Read how many rows
            uint32_t nCols;                                                 // Number of columns to read, max 1024
            uint8_t precision;    
            uint8_t bufNum;  
            // variables
            bit_ptr;
            row_store_ptr;
            // since each element is 8 bit, arrayID_offset has 8 elements corresponding to each bit
            std::vector<uint8_t> relative_offset_buf(7);
            std::vector<uint8_t> arrayID_offset(8);

            // Buffer Array 
            std::vector<uint64_t> bufArray(8);               // nBuf = 8, busWidth=64 bit
            std::vector<uint8_t> bufArrayOutReFormat(64);    // bitline=64
            // write bank queue
            std::vector<PacketPtr> bitSliceQueue;
            // events
            EventFunctionWrapper writeEvent;
        protected:
        public:
            P2S_R_T(P2S_R_TParams *params);
            bool handleP2SRequest(PacketPtr pkt);
            void extractBits_R_T(std::vector<uint8_t> buf, uint8_t bit);
            void processWriteEvent();
    };
}