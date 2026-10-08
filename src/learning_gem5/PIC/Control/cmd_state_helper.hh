#ifndef __LEARNING_GEM5_CMD_STATE_HELPER_HH__
#define __LEARNING_GEM5_CMD_STATE_HELPER_HH__

#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include "mem/port.hh"
#include "params/CmdStateHelper.hh"
#include "sim/clocked_object.hh"


namespace gem5
{
    constexpr uint16_t CmdIdMax = 256;
    enum class QryTabClient : uint8_t
    {
        MAIN_SETTER,
        READER,
        LD_L,
        LD_R,
        EXE,
        LD_P,
        ACC,    // 6
        ST_P,
        P2S_L,
        P2S_R,
        P2S_R_T,
        total_client
    };

    struct CmdTableEntry
    {
        // Not in queue
        // val VALID=Bool()
        // is finished?
        // val FINISH=Bool()
        bool valid;
        bool finish;
    };

    class CmdStateHelper : public ClockedObject {
        private:
            class CPUSidePort : public ResponsePort
            {
                private:
                    CmdStateHelper *owner;
                    PacketPtr blockedPacket;
                public:
                    CPUSidePort(const std::string& name, CmdStateHelper *owner);
                    void sendPacket(PacketPtr pkt);

                // there are three modes: Atomic, Functional and Timing
                protected:
                    Tick recvAtomic(PacketPtr pkt) override {panic("recvAtomic unimplemented.");}
                    void recvFunctional(PacketPtr pkt) override {panic("recvFunctional unimplemented.");}
                    bool recvTimingReq(PacketPtr pkt) override;
                    void recvRespRetry() override;
                    AddrRangeList getAddrRanges() const override {return {};}
            };
            CPUSidePort instPort;
            uint8_t client_num;

            std::vector<CmdTableEntry>cmd_state_table;
            uint8_t req_cmdID;

            // Round Robin Arbiter functions
            std::vector<std::deque<PacketPtr>>query_req;
            size_t maxQueryReqSize = 1000;
            PacketPtr pendingReqPkt;
            uint8_t RRArbiterLastChoose;
            
            EventFunctionWrapper arbiterEvent;
            EventFunctionWrapper initEvent;
            EventFunctionWrapper setFinishEvent;
            EventFunctionWrapper checkFinishEvent;
            EventFunctionWrapper setInvalidEvent;
        protected:
        public:
            CmdStateHelper(const CmdStateHelperParams &params);
            Port &getPort(const std::string &if_name, PortID idx = InvalidPortID) override;
            bool handleRequest(PacketPtr pkt);
            void processArbiterEvent();
            void processInitEvent();
            void processSetFinishEvent();
            void processCheckFinishEvent();
            void processSetInvalidEvent();
    };
    
}
#endif // __LEARNING_GEM5_CMD_STATE_HELPER_HH__