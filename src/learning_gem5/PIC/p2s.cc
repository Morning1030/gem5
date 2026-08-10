#include <vector>
#include <deque>
/*
DPM P2S DATA MEMBERS


precision=precision
uint64_t base_dram_addr = base_dramAddr_to_load;
uint64_t base_picAddr=RegInit(0.U((sysCfg.accessCacheFullAddrLen).W))
uint64_t pic_write_ptr = base_picAddr_to_store;
next_row_offset_elem=next_row_offset_elem
next_row_offset_dram=(next_row_offset_elem*bits_per_ele)>>log2Ceil(8)
_L_block_row =_L_block_row
_L_block_row_ptr=RegInit(0.U(sysCfg._L_nRow_sigLen.W))
next_slice_offset_pic=_L_block_row
*/

/*
P2S_L REQUEST PARAMETERS

base_dramAddr_to_load
base_picAddr_to_store
_L_block_row
next_row_offset_elem
precision
*/

/*
P2S_R REQUEST PARAMETERS
nCols=UInt(log2Ceil(sysCfg.core_config.wordlineNums+1).W)    // Number of columns to read, max 1024
nRows=UInt(log2Ceil(sysCfg.core_config.wordlineNums+1).W)    // Read how many rows
precision=UInt(3.W)     
next_row_offset_bytes=UInt(sysCfg.offset_signLen.W)          // 1byte
base_arrayID_to_store=UInt(log2Ceil(sysCfg.numArraysTotal).W) // Which subarray to put the first selected bit map
bufNum=UInt(2.W)
dramAddr=UInt(sysCfg.virtualAddrLen.W)
*/

struct P2SWritePayload {
    uint64_t arrayAddr;
    uint64_t bitSlice;
};

void
DPM::processWriteEvent() {
    if (!bitSliceQueue.empty()) {
        PacketPtr pkt = bitSliceQueue.front();
        bool success = memSidePort.sendTimingReq(pkt);
        if (success) {
            bitSliceQueue.pop_front();
            schedule(writeEvent, curTick() + cycles(1));
        }
        else {
            // p2s is stalled, need to wait for cache bank notify to retry
        }
    }
}

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
    protected:
    public:
        P2S_L();
        bool handleP2SRequest(PacketPtr pkt);
        uint64_t extractBits_L(const std::vector<std::vector<uint8_t>> &arr, uint8_t bit);
};

bool
P2S_L::handleP2SRequest(PacketPtr pkt) {
    // fill the packet field into data members of p2s
    base_dram_addr = base_dramAddr_to_load;
    pic_write_ptr = base_picAddr_to_store;
    next_row_offset_elem=next_row_offset_elem;
    precision=precision;
    next_row_offset_dram=(next_row_offset_elem*bits_per_ele)>>log2Ceil(8)
    _L_block_row =_L_block_row;
    _L_block_row_ptr=RegInit(0.U(sysCfg._L_nRow_sigLen.W))
    next_slice_offset_pic=_L_block_row;

    // ask DMA to get data by cache controller
    // fill into regArray
    for (int i = 0; i < precision; i++) {   // each element is uint_8
        // extract bits from raw data
        uint64_t bitSlice = extractBits(regArray, i);

        // determine the address and pack into packets
        RequestorID requestorId = system.getRequestorId(this, "DPM");

        RequestPtr request = std::make_shared<Request>(
            pioAddr + offset,    // the target MMIO address of cache bank
            p2sWritePayload,     // store address + bitSlice
            Request::,           // TODO
            requestorId
        );

        PacketPtr bitSlicePkt = new Packet(request, MemCmd::WriteReq);
        bitSlicePkt->allocate();

        P2SWritePayload p2sWritePayload = {pic_write_ptr, bitSlice};
        
        // enqueue into write queue
        bitSliceQueue.push_back(bitSlicePkt);
        schedule(ProcessWriteEvent, curTick() + cycles(1));

        // update the next address
        pic_write_ptr += next_slice_offset_pic; // next_slice_offset_pic = _L_block_row(?)
    }
    // write to cache bank

    // send p2s_done to scheduler
    cpuSidePort.sendTimingResp(pkt);
}

uint64_t
P2S_L::extractBits_L(const std::vector<std::vector<uint8_t>> &arr, uint8_t bit) {
    uint64_t extractedBit 0;
    uint64_t bitSlice = 0;

    // for each element in the array
    for (int i = 0; i < 64; i++) {
        // extract the bit for element i
        extractedBit = (arr[i / 8][i % 8] >> bit) & 0x1;

        // shift it to bit and OR to bitSlice
        bitSlice |= (extractedBit << i);
    }
    // element 63, 62, 61, 60 .... 0
    return bitSlice;
}

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

        // wordline offset
        uint32_t curBlockColPtrGlobal;
        uint32_t blockNColInMem;
        uint32_t curBufColPtrInBlock;
        uint32_t curBufNCols;
        uint32_t curEnqBlockInBufColPtr;

        // since each element is 8 bit, arrayID_offset has 8 elements corresponding to each bit
        std::vector<uint8_t> relative_offset_buf(7);
        std::vector<uint8_t> arrayID_offset(8);
        std::vector<std::vector<uint8_t>> &regArray;    // p2s_R it's 64 * 8
    protected:
    public:
        p2s();
        bool handleP2SRequest(PacketPtr pkt);
        void extractBits_R(const std::vector<std::vector<uint8_t>> &arr, uint32_t row, uint8_t bit, uint32_t dim);
        void extractBits_R_T(std::vector<uint_8> buf, uint8_t bit);

};

P2S::P2S_R() {
    // ##########################################################
    // ################# Array offset Part ######################
    // ##########################################################
    get_array_relatice_offset(relative_offset_buf, bufNum);
    arrayID_offset[0] = 0;
    for (int i = 1; i < 8; i++) arrayID_offset[i] = relative_offset_buf[i-1] + arrayID_offset[i - 1];
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
}
bool
P2S_R::handleP2SRequest(PacketPtr pkt) {
    // fill the packet field into data members of p2s
    // ask DMA to get data by cache controller
    // fill into regArray

    // per block
    for (curBlockColPtrGlobal = 0; curBlockColPtrGlobal < nCols; curBlockColPtrGlobal += blockNColInMem) {
        blockNColInMem = std::min(128, nCols - curBlockColPtrGlobal);

        // per buffer
        for (curBufColPtrInBlock = 0; curBufColPtrInBlock < blockNColInMem; curBufColPtrInBlock += curBufNCols) {

            curBufNCols = std::min(8, blockNColInMem - curBufColPtrInBlock);   // usually 8, only the last one could be less

            // per col
            for (curEnqBlockInBufColPtr = 0; curEnqBlockInBufColPtr < curBufNCols; curEnqBlockInBufColPtr++) {

                // per bit, each element is uint_8
                for (int i = 0; i < precision; i++) {
                    // extract bits from raw data
                    uint64_t bitSlice = extractBits_R(regArray, curEnqBlockInBufColPtr, i)

                    // determine the address and pack into packets
                    RequestorID requestorId = system.getRequestorId(this, "DPM");

                    RequestPtr request = std::make_shared<Request>(
                        pioAddr + offset,    // the target MMIO address of cache bank
                        p2sWritePayload,     // store address + bitSlice
                        0,                   // TODO request flag?
                        requestorId
                    );

                    PacketPtr bitSlicePkt = new Packet(request, MemCmd::WriteReq);
                    bitSlicePkt->allocate();

                    uint64_t arrayAddrEnq = currArrayID << log2Ceil(coreCfg.wordlineNums) + curBlockColPtrGlobal + curBufColPtrInBlock + curEnqBlockInBufColPtr;
                    P2SWritePayload p2sWritePayload = {arrayAddrEnq, bitSlice};
                    
                    // enqueue into write queue
                    bitSliceQueue.push_back(bitSlicePkt);
                    schedule(ProcessWriteEvent, curTick() + cycles(1));

                }
            }
        }
    }
    // write to cache bank

    // send p2s_done to scheduler
    cpuSidePort.sendTimingResp(pkt);
}

uint64_t
P2S_R::extractBits_R(const std::vector<std::vector<uint8_t>> &arr, uint32_t row, uint8_t bit, uint32_t dim=8) {
    uint64_t extractedBit 0;
    uint64_t bitSlice = 0;

    assert(row < dim);

    // for each element in the array
    for (j = 0; j < 64; j++) {
        extractBits = (arr[row][j] >> bit) & 0x1;
        bitSlice |= (extractedBit << j);
    }
    return bitSlice;
}
uint64_t
PS2_R::extractBits_R_T(std::vector<uint_8> buf, uint8_t bit) {
    uint64_t extractedBit 0;
    uint64_t bitSlice = 0;

    for (int i = 0; i < 64; i++) {
        // extract the bit for element i
        extractedBit = (buf[i] >> bit) & 0x1;

        // shift it to bit and OR to bitSlice
        bitSlice |= (extractedBit << i);
    }
    // element 63, 62, 61, 60 .... 0
    return bitSlice;
}
void
P2S_R::get_array_relatice_offset(std::vector<uint8_t> &offset, uint8_t numBuf) { // numBuf is 2 bit in fact
    if (numBuf == 1) offset = [4, 4, 4, 4, 4, 4, 4];        // therefore later arrayID_offset could be [0, 4, 8, 12, 16, 20, 24, 28]
    else if (numBuf == 2) offset = [1, 3, 1, 3, 1, 3, 1];   // therefore later arrayID_offset could be [0, 1, 4, 5, 8, 9, 12, 13]
    else if (numBuf == 3) offset = [1, 1, 2, 1, 1, 2, 1];   // therefore later arrayID_offset could be [0, 1, 2, 4, 5, 6, 8, 9]
}