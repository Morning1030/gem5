/*
 * Copyright (c) 2017 Jason Lowe-Power
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "learning_gem5/PIC/p2s.hh"
#include "debug/P2S_L.hh"
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
namespace gem5
{
P2S_R::P2S_R(P2S_RParams *params) :
    ClockedObject(params),
    instPort(params.name + ".cpu_port", this),
    DMAPort(params.name + ".dma_port", this),
    CacheBankPort(params.name + ".cb_port", this),
    dmaReadEvent([this]{this->processDMAReadEvent();}, "dmaReadEvent"),
    bitSliceEvent([this]{this->processBitSliceEvent();}, "bitSliceEvent"),
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

}

