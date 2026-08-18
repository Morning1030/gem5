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

P2S_R::P2S_R(P2S_RParams *params) :
    ClockedObject(params),
    instPort(params.name + ".cpu_port", this),
    DMAPort(params.name + ".dma_port", this),
    CacheBankPort(params.name + ".cb_port", this),
    writeEvent([this]{this->processWriteEvent();}, "writeBankEvent")
{
    // ##########################################################
    // ################# Array offset Part ######################
    // ##########################################################
    get_array_relatice_offset(relative_offset_buf, bufNum);
    arrayID_offset[0] = 0;
    for (int i = 1; i < 8; i++) arrayID_offset[i] = arrayID_offset[i - 1] + relative_offset_buf[i - 1];
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
}
void
P2S_R::get_array_relatice_offset(std::vector<uint8_t> &offset, uint8_t numBuf) { // numBuf is 2 bit in fact
    if (numBuf == 1) offset = [4, 4, 4, 4, 4, 4, 4];        // therefore later arrayID_offset could be [0, 4, 8, 12, 16, 20, 24, 28]
    else if (numBuf == 2) offset = [1, 3, 1, 3, 1, 3, 1];   // therefore later arrayID_offset could be [0, 1, 4, 5, 8, 9, 12, 13]
    else if (numBuf == 3) offset = [1, 1, 2, 1, 1, 2, 1];   // therefore later arrayID_offset could be [0, 1, 2, 4, 5, 6, 8, 9]
}

bool
P2S_R::handleRequest(PacketPtr pkt) {
    // fill the packet field into data members of p2s
    // ask DMA to get data by cache controller
    // fill into regArray
    
    // bitSlice
    uint64_t bitSlice;

    // arrayID
    uint64_t currArrayID;
    uint64_t arrayAddrEnq;

    // wordline offset
    uint32_t curBlockColPtrGlobal;
    uint32_t blockNColInMem;
    uint32_t curBufColPtrInBlock;
    uint32_t curBufNCols;
    uint32_t curEnqBlockInBufColPtr;

    // per block
    for (curBlockColPtrGlobal = 0; curBlockColPtrGlobal < nCols; curBlockColPtrGlobal += blockNColInMem) {
        blockNColInMem = std::min(128, nCols - curBlockColPtrGlobal);

        // per buffer
        for (curBufColPtrInBlock = 0; curBufColPtrInBlock < blockNColInMem; curBufColPtrInBlock += curBufNCols) {

            curBufNCols = std::min(8, blockNColInMem - curBufColPtrInBlock);   // usually 8, only the last one could be less

            // per col
            for (curEnqBlockInBufColPtr = 0; curEnqBlockInBufColPtr < curBufNCols; curEnqBlockInBufColPtr++) {

                // per bit, each element is uint_8
                for (int bit = 0; bit < precision; bit++) {
                    // extract bits from raw data
                    bitSlice = extractBits_R(regArray, curEnqBlockInBufColPtr, bit)

                    // determine the address
                    curArrayID = base_arrayID_to_store + arrayID_offset[bit];
                    arrayAddrEnq = currArrayID << log2Ceil(coreCfg.wordlineNums) + curBlockColPtrGlobal + curBufColPtrInBlock + curEnqBlockInBufColPtr;
                    P2SWritePayload p2sWritePayload = {arrayAddrEnq, bitSlice};

                    // pack into packets
                    RequestorID requestorId = system.getRequestorId(this, "DPM");

                    RequestPtr request = std::make_shared<Request>(
                        pioAddr + offset,    // the target MMIO address of cache bank
                        p2sWritePayload,     // store address + bitSlice
                        0,                   // TODO request flag?
                        requestorId
                    );

                    PacketPtr bitSlicePkt = new Packet(request, MemCmd::WriteReq);
                    bitSlicePkt->allocate();
                    
                    // enqueue into write queue
                    bitSliceQueue.push_back(bitSlicePkt);
                    if (i == 0) schedule(writeEvent, curTick() + cycles(1));

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

void
P2S_R::processWriteEvent() {
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


P2S_R_T::P2S_R_T(P2S_R_TParams *params) :
    ClockedObject(params),
    instPort(params.name + ".cpu_port", this),
    DMAPort(params.name + ".dma_port", this),
    CacheBankPort(params.name + ".cb_port", this),
    writeEvent([this]{this->processWriteEvent();}, "writeBankEvent")
{
    // ##########################################################
    // ################# Array offset Part ######################
    // ##########################################################
    get_array_relatice_offset(relative_offset_buf, bufNum);
    arrayID_offset[0] = 0;
    for (int i = 1; i < 8; i++) arrayID_offset[i] = arrayID_offset[i - 1] + relative_offset_buf[i - 1];
}

P2S_R::get_array_relatice_offset(std::vector<uint8_t> &offset, uint8_t numBuf) { // numBuf is 2 bit in fact
    if (numBuf == 1) offset = [4, 4, 4, 4, 4, 4, 4];        // therefore later arrayID_offset could be [0, 4, 8, 12, 16, 20, 24, 28]
    else if (numBuf == 2) offset = [1, 3, 1, 3, 1, 3, 1];   // therefore later arrayID_offset could be [0, 1, 4, 5, 8, 9, 12, 13]
    else if (numBuf == 3) offset = [1, 1, 2, 1, 1, 2, 1];   // therefore later arrayID_offset could be [0, 1, 2, 4, 5, 6, 8, 9]
}

P2S_R_T::handleRequest(PacketPtr pkt) {
    // bitSlice
    uint64_t bitSlice;

    // arrayID
    uint64_t currArrayID;
    uint64_t arrayAddrEnq;

    for (int rowPtrStore = 0; rowPtrStore < nRows; rowPtrStore++) {

        // reformat the buffer from dma data to extract bit format
        for(int i = 0; i < 64; i++) {
            uint8_t bitShift = (i % 8) * 8;
            bufArrayOutReFormat[i] = (bufArray[i / 8] >> bitShift) & 0xFF;
        }

        // per bit, each element is uint8_t
        for (int bit; bit < precision; bit++) {
            // extract bits from raw data
            bitSlice = extractBits_R_T(bufArrayOutReFormat, bit);

            // determine the address
            curArrayID = base_arrayID_to_store + arrayID_offset[bit];
            arrayAddrEnq = currArrayID << log2Ceil(coreCfg.wordlineNums) + rowPtrStore;

            // pack into packets
            RequestorID requestorId = system.getRequestorId(this, "DPM");

            RequestPtr request = std::make_shared<Request>(
                pioAddr + offset,    // the target MMIO address of cache bank
                p2sWritePayload,     // store address + bitSlice
                0,                   // TODO request flag?
                requestorId
            );

            PacketPtr bitSlicePkt = new Packet(request, MemCmd::WriteReq);
            bitSlicePkt->allocate();

            P2SWritePayload p2sWritePayload = {arrayAddrEnq, bitSlice};
                        
            // enqueue into write queue
            bitSliceQueue.push_back(bitSlicePkt);
            if (i == 0) schedule(writeEvent, curTick() + cycles(1));
        }
    }
    // write to cache bank

    // send p2s_done to scheduler
    cpuSidePort.sendTimingResp(pkt);
}
uint64_t
P2S_R_T::extractBits_R_T(const std::vector<uint_8> &buf, uint8_t bit) {
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
P2S_R_T::processWriteEvent() {
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