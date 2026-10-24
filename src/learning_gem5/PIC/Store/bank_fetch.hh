#ifndef __LEARNING_GEM5_BANK_FETCH_HH__
#define __LEARNING_GEM5_BANK_FETCH_HH__

#include <deque>
#include <string>
#include <vector>

#include "mem/port.hh"
#include "params/BankFetch.hh"
#include "sim/clocked_object.hh"
namespace gem5
{
    class BankFetch : public ClockedObject {
        public:
            enum class PICPortID {CB, DMA};
        private:
            class CPUSidePort : public ResponsePort{
                private:
                    BankFetch* owner;
                protected:
                    Tick recvAtomic(PacketPtr pkt) override {panic("recvAtomic unimplemented.");}
                    void recvFunctional(PacketPtr pkt) override {panic("recvFunctional unimplemented.");}
                    bool recvTimingReq(PacketPtr pkt) override;
                    void recvRespRetry() override;
                    AddrRangeList getAddrRanges() const override {return {};}
                public:
                    CPUSidePort(const std::string &name, BankFetch* owner);
                    PacketPtr blockedPacket;
                    void sendPacket(PacketPtr pkt);
            };
            class MemSidePort : public RequestPort{
                private:
                    BankFetch* owner;
                    PICPortID picPortID;
                protected:
                    bool recvTimingResp(PacketPtr pkt) override;
                    void recvReqRetry() override;
                public:
                    MemSidePort(const std::string &name, BankFetch* owner, PICPortID picPortID);
                    PacketPtr blockedPacket;
                    void sendPacket(PacketPtr pkt);
            };
            CPUSidePort instPort;
            MemSidePort cacheBankPort;
            MemSidePort dmaWritePort;
            RequestorID requestorId;
            PacketPtr pendingReqPkt;

            // bankFetch request variables
            uint64_t src;
            uint64_t dst;
            uint32_t nBytes;

            // pageFetch variables
            uint64_t v_addr_ptr;
            uint64_t final_v_addr;
            uint64_t next_page_begin_addr;

            uint32_t validNBytes;
            uint32_t startRow;
            uint32_t endRow;

            // atomic row variables
            uint64_t nextBankAddr;
            uint32_t bankRowPtr;
            uint32_t rowsReceived;
            uint8_t offset;

            std::deque<PacketPtr> bankFetchAtomicQueue;
            std::vector<uint8_t> cacheBankData;

            EventFunctionWrapper pageFetchEvent;
            EventFunctionWrapper atomicBankFetchEvent;
            EventFunctionWrapper alignEvent;

        protected:
        public:
            BankFetch(const BankFetchParams &params);
            Port& getPort(const std::string &if_name, PortID idx = InvalidPortID) override;
            bool handleRequest(PacketPtr pkt);
            bool handleResponse(PICPortID picPortID, PacketPtr pkt);
            void processPageFetchEvent();
            void processAtomicBankFetchEvent();
            void processAlignEvent();
    };
}
#endif