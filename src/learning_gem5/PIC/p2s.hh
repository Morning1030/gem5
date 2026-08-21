// mod: p2s_arbiter
// Forward-declared only (no include) so this header doesn't drag in the
// arbiter's definition just to hold a pointer. See
// mem/cache/pic/access_bank_arb.hh for AccessBankArb::submitP2SWrite() and
// the kClientP2S_L/R/R_T IDs the .cc file uses. (This used to be
// gem5::PICLLCTags -- moved to a standalone module since the arbiter needs
// no tag/way state, only numBanks.)
namespace gem5 { class AccessBankArb; }

class P2S_L {
    private:
        uint8_t precision;
        uint64_t base_dram_addr = base_dramAddr_to_load;
        uint64_t base_picAddr=RegInit(0.U((sysCfg.accessCacheFullAddrLen).W))
        uint64_t pic_write_ptr = base_picAddr_to_store;
        next_row_offset_elem=next_row_offset_elem
        next_row_offset_dram=(next_row_offset_elem*bits_per_ele)>>log2Ceil(8)
        _L_block_row =_L_block_row
        _L_block_row_ptr=RegInit(0.U(sysCfg._L_nRow_sigLen.W))
        next_slice_offset_pic=_L_block_row
        std::vector<std::vector<uint8_t>> &regArray; // 8 * 8

        // mod: p2s_arbiter
        // Shared cache-bank arbiter this engine's WRITEs are issued
        // through (accessArb.io.request(kClientP2S_L) <> io.accessArray
        // in the RTL). Owned by whoever wires up the DPM; not this class.
        gem5::AccessBankArb *bankArb = nullptr;
    protected:
    public:
        P2S_L();
        bool handleP2SRequest(PacketPtr pkt);
        uint64_t extractBits_L(const std::vector<std::vector<uint8_t>> &arr, uint8_t bit);
};

class P2S_R {
    private:
        // request params
        uint64_t base_arrayID_to_store;
        uint32_t nCols;
        uint32_t nRows;
        uint8_t precision;
        uint8_t bufNum;

        next_row_offset_bytes=UInt(sysCfg.offset_signLen.W)
        dramAddr=UInt(sysCfg.virtualAddrLen.W)

        // since each element is 8 bit, arrayID_offset has 8 elements corresponding to each bit
        std::vector<uint8_t> relative_offset_buf(7);
        std::vector<uint8_t> arrayID_offset(8);
        std::vector<std::vector<uint8_t>> &regArray;    // p2s_R it's 64 * 8

        // mod: p2s_arbiter
        gem5::AccessBankArb *bankArb = nullptr;  // kClientP2S_R
    protected:
    public:
        p2s();
        bool handleP2SRequest(PacketPtr pkt);
        void extractBits_R(const std::vector<std::vector<uint8_t>> &arr, uint32_t row, uint8_t bit, uint32_t dim);
        void extractBits_R_T(std::vector<uint_8> buf, uint8_t bit);

};

class P2S_R_T {
    // request params
    uint64_t base_arrayID_to_store;
    uint32_t nCols;
    uint32_t nRows;
    uint8_t precision;
    uint8_t bufNum;
    next_row_offset_bytes=UInt(sysCfg.offset_signLen.W)
    dramAddr=UInt(sysCfg.virtualAddrLen.W)

    // since each element is 8 bit, arrayID_offset has 8 elements corresponding to each bit
    std::vector<uint8_t> relative_offset_buf(7);
    std::vector<uint8_t> arrayID_offset(8);

    // Buffer Array
    std::vector<uint64_t> bufArray(8);               // nBuf = 8, busWidth=64 bit
    std::vector<uint8_t> bufArrayOutReFormat(64);    // bitline=64

    // mod: p2s_arbiter
    gem5::AccessBankArb *bankArb = nullptr;  // kClientP2S_R_T
};