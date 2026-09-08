#ifndef __LEARNING_GEM5_CMD_STATE_HELPER_HH__
#define __LEARNING_GEM5_CMD_STATE_HELPER_HH__

#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include "mem/port.hh"
#include "params/cmd_state_helper.hh"
#include "sim/clocked_object.hh"

namespace gem5
{
    enum class QryTabClient
    {
        MAIN_SETTER,
        READER,
        LD_L,
        LD_R,
        EXE,
        LD_P,
        ACC,
        ST_P,
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
            QryTabClient client_num;

            std::vector<CmdTableEntry>cmd_state_table;
            uint8_t req_cmdID;

            // Round Robin Arbiter functions
            std::vector<std::queue<PacketPtr>query_req>(QryTabClient::client_num);
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
            void processSetFinishEvent();
            void processCheckFinishEvent();
            void processSetInvalidEvent();
    }
    
}