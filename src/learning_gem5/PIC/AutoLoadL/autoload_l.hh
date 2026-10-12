#ifndef __LEARNING_GEM5_AUTOLOAD_L_HH__
#define __LEARNING_GEM5_AUTOLOAD_L_HH__

#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include "mem/port.hh"
#include "params/autoload_l.hh"
#include "sim/clocked_object.hh"

namespace gem5
{
    struct AutoLoadLPayload {
        uint64_t source;
        uint64_t data;
    };
    class AutoLoadL : public ClockedObject {
        private:
            class CPUSidePort : public ResponsePort
            {
                private:
                    AutoLoadL* owner;
                    PacketPtr blockedPacket;
                protected:
                    Tick recvAtomic(PacketPtr pkt) override {panic("recvAtomic unimplemented.");}
                    void recvFunctional(PacketPtr pkt) override {panic("recvFunctional unimplemented.");}
                    bool recvTimingReq(PacketPtr pkt) override;
                    void recvRespRetry() override;
                    AddrRangeList getAddrRanges() const override {return {};}
                public:
                    CPUSidePort(const std::string& name, AutoLoadL* owner);
                    void sendPacket(PacketPtr pkt);

            };
            class MemSidePort : public RequestPort
            {
                private:
                    AutoLoadL* owner;
                    PacketPtr blockedPacket;
                protected:
                    bool recvTimingResp(PacketPtr pkt) override;
                    void recvReqRetry() override;
                public:
                    MemSidePort(const std::string& name, AutoLoadL* owner);
                    void sendPacket(PacketPtr pkt);
            };

            CPUSidePort instPort;
            MemSidePort cacheBankPort;
            RequestorID requestorId;
            PacketPtr pendingReqPkt;

            EventFunctionWrapper loadReqEvent;
            EventFunctionWrapper recvRespEvent;

            uint64_t dataReadFromBank;
            bool respBusy;
        protected:
        public:
            AutoLoadL(const AutoLoadLParams &params);
            Port &getPort(const std::string &if_name, PortID idx = InvalidPortID) override;
            bool handleRequest(PacketPtr pkt);
            bool handleResponse(PacketPtr pkt);
            void processLoadReqEvent();
            void processRecvRespEvent();
    };
}