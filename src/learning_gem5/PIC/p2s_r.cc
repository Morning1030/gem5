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
    loadBufferEvent([this]{this->processLoadBufferEvent();}, "loadBufferEvent"),
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
bool
P2S_R::CPUSidePort::recvTimingReq(PacketPtr pkt) {
    // Just forward to the memobj.
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    } else {
        return true;
    }
}
bool
P2S_R::MemSidePort::recvTimingResp(PacketPtr pkt) {
    // fill the response to sram block
    // TODO need sender state row to deal with out of order receiving
    uint8_t *dmaData = pkt->getConstPtr<uint8_t>(); // dma send a uint64_t variable but can be expressed as 8 * 8 uint8_t
    size_t pktSize = pkt->getSize();
    size_t writeOffset = writeMemAddr * 8;

    // std::vector<uint8_t> mem0 is 64 * 128, therefore each resp from dma will only take 8 elements in each row
    if (curBlockRowPtr < mem0.size() && (pktSize + writeOffset) <= mem0[curBlockRowPtr].size()) {
        std::memcpy(mem0[curBlockRowPtr].data() + writeOffset, dmaData, pktSize);
    } else {
        panic("P2S_R: mem0 buffer overflow! curBlockRowPtr=%u\n", curBlockRowPtr);
    }

    delete pkt;
    
    // the whole row finishes
    if ((pktSize + writeOffset) == curBlockNCols) {
        
        writeMemAddr = 0;
        curBlockRowPtr++;
        curRowDramAddrOffset += next_row_offset_bytes;
        
        // the whole block finishes
        if (curBlockRowPtr == curBlockNRows) {
            curBlockRowPtr = 0;
            curBlockColPtr += curBlockNCols;
            curRowDramAddrOffset = 0;
            curBlockDramBaseAddrPtr += curBlockNCols;
            readMemAddr = 0;
            // fill dma from SRAM block into buffer
            schedule(loadBufferEvent, curTick() + cycles(1));
        
        // schedule for the next row DMA
        } else {
            schedule(dmaReadEvent, curTick() + cycles(1));
        }
    }
    else {
        // on the same row
        writeMemAddr++;
    }
        
}

bool
P2S_R::handleRequest(PacketPtr pkt) {
    // fill the packet field into data members of p2s
    // ask DMA to get data by cache controller
    // fill into regArray

    // fill the packet field into data members of p2s
    P2S_R_Payload *p2s_R_Payload = pkt->getConstPtr<P2S_R_Payload>();
    dramAddr = p2s_R_Payload->dramAddr;
    base_arrayID_to_store = p2s_R_Payload->base_arrayID_to_store; // Which subarray to put the first selected bit map
    next_row_offset_bytes = p2s_R_Payload->next_row_offset_bytes;                                 // 15bits
    nRows = p2s_R_Payload->nRows;                                                 // Read how many rows
    nCols = p2s_R_Payload->nCols;                                                 // Number of columns to read, max 1024
    precision = p2s_R_Payload->precision;    
    bufNum = p2s_R_Payload->bufNum;  

    // initialize data memebers
    // bitSlice variables
    bit_ptr = 0;
    curEnqBlockInBufColPtr = 0;
    curBufColPtrInBlock = 0;
    curBlockColPtrGlobal = 0;
    blockNColInMem = std::min(128, nCols - curBlockColPtrGlobal);
    curBufNCols = std::min(8, blockNColInMem - curBufColPtrInBlock);

    // dma and buffer variables
    curBlockRowPtr = 0;
    curBlockColPtr = 0;
    curBlockNCols = std::min(128, nCols - curBlockColPtr);          // equal to blockNColInMem
    curBlockNRows = nRows;
    writeBufRowPtr = 0;
    readMemAddr= 0;
    writeMemAddr = 0;
    curBlockDramBaseAddrPtr = p2s_R_Payload->dramAddr;
    curRowDramAddrOffset = 0;
    schedule(dmaReadEvent, curTick() + cycles(1));

    // send p2s_done to scheduler
    cpuSidePort.sendTimingResp(pkt);
}
void 
P2S_R::processDMAReadEvent() {
    // read 128 col into SRAM block
    RequestorID requestorId = system.getRequestorId(this, "P2S_R");

    RequestPtr request = std::make_shared<Request>(
        pioAddr + offset                    // the target MMIO address of cache controller
        sizeof(DMARPayload),                // next_row_offset_elem, base_dram_addr
        0,                                  // TODO
        requestorId
    )
    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);
        
    // ask DMA to get data by cache controller
    curBlockNCols = std::min(128, nCols - curBlockColPtr);
    DMARPayload* dmaRPayload = new DMARPayload{curBlockNCols, next_row_offset_bytes, curBlockDramBaseAddrPtr + curRowDramAddrOffset};
    
    pkt->dataDynamic(reinterpret_cast<uint8_t*>(&dmaRPayload));
    bool success = DMAPort.sendTimingReq(pkt);
    if (success) {

    }
    else {} // need to retry

}
void
P2S_R::processLoadBufferEvent() {
    // fill the block into the buffer
    uint32_t readOffset = readMemAddr * 8;
    std::memcpy(regArray[writeBufRowPtr], mem0[writeBufRowPtr].data() + readOffset, 8 * sizeof(uint8_t));
    writeBufRowPtr++;

    if (writeBufRowPtr == curBlockNRows) {
        writeBufRowPtr = 0;
        readMemAddr++;
        schedule(bitSliceEvent, curTick() + cycles(1));
    }
    else {
        schedule(loadBufferEvent, curTick() + cycles(1));
    }
    
}
void
P2S_R::processBitSliceEvent() {
    // extract bits from raw data
    uint64_t bitSlice = extractBits_R(regArray, curEnqBlockInBufColPtr, bit_ptr)

    // determine the address
    uint64_t curArrayID = base_arrayID_to_store + arrayID_offset[bit_ptr];
    uint64_t arrayAddrEnq = (currArrayID << log2Ceil(coreCfg.wordlineNums)) + curBlockColPtrGlobal + curBufColPtrInBlock + curEnqBlockInBufColPtr;
    P2SWritePayload p2sWritePayload = {arrayAddrEnq, bitSlice};

    // pack into packets
    RequestorID requestorId = system.getRequestorId(this, "P2S_R_T");

    RequestPtr request = std::make_shared<Request>(
        pioAddr + offset,            // the target MMIO address of cache bank
        sizeof(p2sWritePayload),     // store address + bitSlice
        0,                           // TODO request flag?
        requestorId
    );
    
    // TODO how to couple p2sWritePayload with Packet?
    PacketPtr bitSlicePkt = new Packet(request, MemCmd::WriteReq);
    bitSlicePkt->allocate();
                    
    // enqueue into write queue
    bitSliceQueue.push_back(bitSlicePkt);
    // write to cache bank
    schedule(writeEvent, curTick() + cycles(1));

    // per bit
    bit_ptr++;
    if (bit_ptr == precision) {
        // per row in the buffer
        bit_ptr = 0;
        curEnqBlockInBufColPtr++;
        if (curEnqBlockInBufColPtr == curBufNCols) {
            // per buffer
            curEnqBlockInBufColPtr = 0;
            if ((curBufColPtrInBlock + curBufNCols) == blockNColInMem) {
                curBufColPtrInBlock = 0;
                // per block
                if ((curBlockColPtrGlobal + blockNColInMem) == nCols) {
                    // TODO send p2s_done
                }
                else {
                    curBlockColPtrGlobal += blockNColInMem;
                    blockNColInMem = std::min(128, nCols - curBlockColPtrGlobal);
                    // (TO BE CHECKED)
                    schedule(dmaReadEvent, curTick() + cycles(1));
                }
            }
            else {
                curBufColPtrInBlock += curBufNCols;
                curBufNCols = std::min(8, blockNColInMem - curBufColPtrInBlock);
                // (TO BE CHECKED)
                schedule(loadBufferEvent, curTick() + cycles(1));
            }
            
        }
        else schedule(bitSliceEvent, curTick() + cycles(1));
    }
    else schedule(bitSliceEvent, curTick() + cycles(1));
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

void
P2S_R::get_array_relatice_offset(std::vector<uint8_t> &offset, uint8_t numBuf) { // numBuf is 2 bit in fact
    if (numBuf == 1) offset = [4, 4, 4, 4, 4, 4, 4];        // therefore later arrayID_offset could be [0, 4, 8, 12, 16, 20, 24, 28]
    else if (numBuf == 2) offset = [1, 3, 1, 3, 1, 3, 1];   // therefore later arrayID_offset could be [0, 1, 4, 5, 8, 9, 12, 13]
    else if (numBuf == 3) offset = [1, 1, 2, 1, 1, 2, 1];   // therefore later arrayID_offset could be [0, 1, 2, 4, 5, 6, 8, 9]
}

uint64_t
P2S_R::extractBits_R(const std::vector<std::vector<uint8_t>> &arr, uint32_t row, uint8_t bit, uint32_t dim=8) {
    uint64_t extractedBit 0;
    uint64_t bitSlice = 0;

    assert(row < dim);

    // for each element in the array
    for (j = 0; j < 64; j++) {
        // here take regArray as regArray.transpose
        extractBits = (arr[j][row] >> bit) & 0x1;
        bitSlice |= (extractedBit << j);
    }
    return bitSlice;
}



}

