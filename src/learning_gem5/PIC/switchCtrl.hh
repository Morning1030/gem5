class SwitchController {
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
        void processNextSet();

      public:
        SwitchController(Scheduler *owner);
        bool handleResponse(PacketPtr pkt);
};