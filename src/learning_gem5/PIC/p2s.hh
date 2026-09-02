#ifndef __LEARNING_GEM5_P2S_HH__
#define __LEARNING_GEM5_P2S_HH__

#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include "mem/port.hh"
#include "params/P2S_L.hh"
#include "params/P2S_R.hh"
#include "params/P2S_R_T.hh"
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
                    PacketPtr blockedPacket;
                public:
                    CPUSidePort(const std::string& name, P2S_L *owner);
                    // TODO fix sendPacket, sendTimingReq, sendTimingResp, recvReqRetry, and RecvRespRetry
                    void sendPacket(PacketPtr pkt);

                // there are three modes: Atomic, Functional and Timing
                protected:
                    Tick recvAtomic(PacketPtr pkt) override {panic("recvAtomic unimplemented.");}
                    void recvFunctional(PacketPtr pkt) override {panic("recvFunctional unimplemented.");}
                    bool recvTimingReq(PacketPtr pkt) override;
                    void recvRespRetry() override;
                    AddrRangeList getAddrRanges() const override {return {};}
            };
            // to interact with DMA and Cache Bank
            class MemSidePort : public RequestPort
            {
                private:
                    // corresponds to each direct port
                    P2S_L *owner;
                    PICPortID portID;               
                public:
                    enum class PICPortID {DMA, CB};
                    MemSidePort(const std::string& name, P2S_L *owner, PICPortID picPortID);
                    PacketPtr blockedPacket;
                    void sendPacket(PacketPtr pkt) {}

                protected:
                    bool recvTimingResp(PacketPtr pkt) override;
                    void recvReqRetry() override {};
            };
            // direct port
            CPUSidePort instPort;
            MemSidePort DMAPort;
            MemSidePort CacheBankPort;

            // request and control
            RequestorID requestorId;
            PacketPtr pendingReqPkt;
            bool p2sDone;

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
            std::deque<PacketPtr> bitSliceQueue;

            // events
            EventFunctionWrapper dmaReadEvent;
            EventFunctionWrapper bitSliceEvent;
            EventFunctionWrapper writeEvent;

        protected:
        public:
            Port &getPort(const std::string &if_name, PortID idx = InvalidPortID) override;
            P2S_L(const P2S_LParams &params);
            bool handleRequest(PacketPtr pkt);
            uint64_t extractBits(const std::vector<std::vector<uint8_t>> &arr, uint8_t bit);
            void processDMAReadEvent();
            void processBitSliceEvent();
            void processWriteEvent();
    };

    class P2S_R : public ClockedObject {
        private:
            class CPUSidePort : public ResponsePort
            {
                private:
                    P2S_R *owner;
                    PacketPtr blockedPacket;
                public:
                    CPUSidePort(const std::string& name, P2S_R *owner);
                    void sendPacket(PacketPtr pkt);                     // TBD whether sendPacket is bool or void

                // there are three modes: Atomic, Functional and Timing
                protected:
                    Tick recvAtomic(PacketPtr pkt) override {panic("recvAtomic unimplemented.");}
                    void recvFunctional(PacketPtr pkt) override {panic("recvFunctional unimplemented.");}
                    bool recvTimingReq(PacketPtr pkt) override;
                    void recvRespRetry() override;
                    AddrRangeList getAddrRanges() const override {return {};}
            };
            // to interact with DMA and Cache Bank
            class MemSidePort : public RequestPort
            {
                private:
                    P2S_R *owner;
                    PICPortID portID;
                public:
                    enum class PICPortID {DMA, CB};
                    MemSidePort(const std::string& name, P2S_L *owner, PICPortID picPortID);
                    PacketPtr blockedPacket;
                    void sendPacket(PacketPtr pkt) {}
                protected:
                    bool recvTimingResp(PacketPtr pkt) override;
                    void recvReqRetry() override;
            };
            // direct port
            CPUSidePort instPort;
            MemSidePort DMAPort;
            MemSidePort CacheBankPort;

            // request and control
            RequestorID requestorId;
            PacketPtr pendingReqPkt;
            bool p2sDone;

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
            std::deque<PacketPtr> bitSliceQueue;

            // Set when no more output slices will be produced
            void completeControlRequest();
            void retryControlResponse();
            PacketPtr blockedDmaPkt = nullptr;
            bool handleDMAResponse(PacketPtr pkt);
            void retryDMARequest();

            // events
            EventFunctionWrapper dmaReadEvent;
            EventFunctionWrapper loadBufferEvent;
            EventFunctionWrapper bitSliceEvent;
            EventFunctionWrapper writeEvent;
        protected:
        public:
            P2S_R(const P2S_RParams &params);
            Port &getPort(const std::string &if_name, PortID idx = InvalidPortID) override;
            bool handleRequest(PacketPtr pkt);
            void get_array_relatice_offset(std::vector<uint8_t> &offset, uint8_t numBuf);
            void extractBits(const std::vector<std::vector<uint8_t>> &arr, uint32_t row, uint8_t bit, uint32_t dim);
            void processDMAReadEvent();
            void processLoadBufferEvent();
            void processBitSliceEvent();
            void processWriteEvent();

    };

    class P2S_R_T : public ClockedObject{
        private:
            class CPUSidePort : public ResponsePort
            {
                private:
                    P2S_R_T *owner;

                public:
                    CPUSidePort(const std::string& name, P2S_R_T *owner);
                    void sendPacket(PacketPtr pkt);

                // there are three modes: Atomic, Functional and Timing
                protected:
                    Tick recvAtomic(PacketPtr pkt) override {panic("recvAtomic unimplemented.");}
                    void recvFunctional(PacketPtr pkt) override {panic("recvFunctional unimplemented.");}
                    bool recvTimingReq(PacketPtr pkt) override;
                    void recvRespRetry() override;
                    AddrRangeList getAddrRanges() const override {return {};}
            };
            // to interact with DMA and Cache Bank
            class MemSidePort : public RequestPort
            {
                private:
                    P2S_R_T *owner;
                    PICPortID portID;
                public:
                    enum class PICPortID {DMA, CB};
                    MemSidePort(const std::string& name, P2S_L *owner, PICPortID picPortID);
                    PacketPtr blockedPacket;
                    void sendPacket(PacketPtr pkt) {}
                protected:
                    bool recvTimingResp(PacketPtr pkt) override;
                    void recvReqRetry() override;
            };
            // direct port
            CPUSidePort instPort;
            MemSidePort DMAPort;
            MemSidePort CacheBankPort;

            // request and control
            RequestorID requestorId;
            PacketPtr pendingReqPkt;
            bool p2sDone;

            // request params
            uint64_t dramAddr;
            uint64_t base_arrayID_to_store; // Which subarray to put the first selected bit map
            uint32_t next_row_offset_bytes;                                 // 15bits
            uint32_t nRows                          ;                        // Read how many rows
            uint32_t nCols;                                                 // Number of columns to read, max 1024
            uint8_t precision;
            uint8_t bufNum;

            // variables
            uint8_t bit_ptr;
            uint32_t row_store_ptr;

            uint8_t dmaRow;
            // since each element is 8 bit, arrayID_offset has 8 elements corresponding to each bit
            std::vector<uint8_t> relative_offset_buf(7);
            std::vector<uint8_t> arrayID_offset(8);

            // Buffer Array
            std::vector<uint8_t> bufArray(64);
            // write bank queue
            std::deque<PacketPtr> bitSliceQueue;

            // events
            EventFunctionWrapper dmaReadEvent;
            EventFunctionWrapper bitSliceEvent;
            EventFunctionWrapper writeEvent;
        protected:
        public:
            P2S_R_T(P2S_R_TParams &params);
            Port &getPort(const std::string &if_name, PortID idx = InvalidPortID) override;
            bool handleRequest(PacketPtr pkt);
            void get_array_relative_offset(std::vector<uint8_t> &offset,uint8_t numBuf);
            void extractBits(std::vector<uint8_t> buf, uint8_t bit);
            void processDMAReadEvent();
            void processBitSliceEvent();
            void processWriteEvent();

    };
}
