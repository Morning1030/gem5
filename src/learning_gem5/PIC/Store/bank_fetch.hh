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
    struct AtomicBankFetchSenderState : public Packet::SenderState {
        // when align, start from offset
        uint8_t offset;
        bool lastCacheBankRow;
        explicit BankFetchSenderState(const uint8_t& offset, const bool& lcb) : offset(offset), lastCacheBankRow(lcb)}
    };
    class BankFetch : public ClockedObject {
        private:
            class CPUSidePort : public ResponsePort{
                private:
                protected:
                public:
                    CPUSidePort(const std::string &name, BankFetch* owner);
            };
            class MemSidePort : public RequestPort{
                private:
                protected:
                public:
                    MemSidePort(const std::string &name, BankFetch* owner);
            };
            CPUSidePort instPort;
            MemSidePort cacheBankPort;
            MemSidePort dmaWritePort;

            uint64_t src;
            uint64_t dst;
            uint32_t nBytes;

            uint64_t v_addr_ptr;
            uint64_t final_v_addr;
            uint64_t next_page_begin_addr;

            uint32_t validNBytes;
            std::deque<PacketPtr> bankFetchAtomicReqs;
            std::vector<uint8_t> cacheBankData;

            EventFunctionWrapper pageFetchEvent;
            EventFunctionWrapper atomicBankFetchEvent;
            EventFunctionWrapper alignDMAEvent;

        protected:
        public:
            BankFetch(const BankFetchParams &params);
            Port& getPort(const std::string &if_name, PortID idx = InvalidPortID) override;
    };
}
#endif