#ifndef __LEARNING_GEM5_P2S_HH__
#define __LEARNING_GEM5_P2S_HH__

#include <string>
#include "mem/port.hh"
#include "params/StoreCtrl.hh"
#include "sim/clocked_object.hh"

namespace gem5
{
    class StoreCtrl : public ClockedObject
    {
        public:
            enum class PICPortID {DMA, BANKFETCH};
        private:
            class CPUSidePort : public ResponsePort
            {
                private:
                    StoreCtrl *owner;
                protected:
                    Tick recvAtomic(PacketPtr pkt) override {panic("recvAtomic unimplemented.");}
                    void recvFunctional(PacketPtr pkt) override {panic("recvFunctional unimplemented.");}
                    bool recvTimingReq(PacketPtr pkt) override;
                    void recvRespRetry() override;
                    AddrRangeList getAddrRanges() const override {return {};}
                public:
                    CPUSidePort(const std::string &name, StoreCtrl *owner);
                    PacketPtr blockedPacket;
                    void sendPacket(PacketPtr pkt);
            };
            class MemSidePort : public RequestPort
            {
                private:
                    StoreCtrl *owner;
                protected:
                    bool recvTimingResp(PacketPtr pkt) override;
                    void recvReqRetry() override;
                public:
                    MemSidePort(const std::string &name, StoreCtrl *owner, PICPortID picPortID);
                    PacketPtr blockedPacket;
                    bool needRetry;
                    void sendPacket(PacketPtr pkt);
            };
            CPUSidePort instPort;
            MemSidePort dmaPort;
            MemSidePort bankFetchPort;

            RequestorID requestorId;
            PacketPtr pendingReqPkt;

            uint64_t src;
            uint64_t dst;
            uint16_t row;                   
            uint16_t byte_per_row;
            uint16_t offset;

            bool one_shot;
            uint16_t row_store_ptr;
            uint16_t fetchSize;

            const size_t dataSize;
            std::vector<uint8_t> bankFetchPayload;

            EventFunctionWrapper bankFetchEvent;
            EventFunctionWrapper dmaWriteEvent;

        protected:
        public:
            StoreCtrl(const StoreCtrlParams &params);
            Port& getPort(const std::string &if_name, PortID idx = InvalidPortID) override;
            bool handleRequest(PacketPtr pkt);
            bool handleResponse(PacketPtr pkt);
            void processBankFetchEvent();
            void processDMAReadEvent();
    };
}
#endif