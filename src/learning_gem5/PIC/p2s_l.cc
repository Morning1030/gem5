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

/* DATA MEMBERS
precision=precision
uint64_t base_dram_addr = base_dramAddr_to_load
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
namespace gem5
{
P2S_L::P2S_L(P2S_LParams *params) :
    ClockedObject(params),
    instPort(params.name + ".cpu_port", this),
    DMAPort(params.name + ".dma_port", this),
    CacheBankPort(params.name + ".cb_port", this),
    dmaReadEvent([this]{this->processDMAReadEvent();}, "dmaReadEvent"),
    bitSliceEvent([this]{this->processBitSliceEvent();}, "bitSliceEvent"),
    writeEvent([this]{this->processWriteEvent();}, "writeBankEvent")
{}

bool
P2S_L::CPUSidePort::recvTimingReq(PacketPtr pkt){
    // Just forward to the memobj.
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    } else {
        return true;
    }
}

void
P2S_L::MemSidePort::sendPacket(PacketPtr pkt){}

// TODO take care of out of order responses
bool
P2S_L::MemSidePort::recvTimingResp(PacketPtr pkt) {
    // fill the response to buffer
    // TODO need sender state row to deal with out of order receiving
    uint8_t *dmaData = pkt->getConstPtr<uint8_t>();
    size_t pktSize = pkt->getSize();

    if (dmaRow < regArray.size() && pktSize <= regArray[dmaRow].size()) {
        std::memcpy(regArray[dmaRow].data(), dmaData, pktSize);
    } else {
        panic("P2S_L: regArray buffer overflow! dmaRow=%u\n", dmaRow);
    }

    delete pkt;
    dmaRow++;

    if (dmaRow == regArray.size()) {
        dmaRow = 0;
        bit_ptr = 0;

        pic_write_ptr = base_picAddr + _L_block_row_ptr;
        schedule(bitSliceEvent, curTick() + cycles(1));

    } else {
        // need to wait for other dmaRows to finish
    }
}

bool
P2S_L::handleRequest(PacketPtr pkt) {
    // fill the packet field into data members of p2s
    P2S_L_Payload *p2s_L_Payload = pkt->getConstPtr<P2S_L_Payload>();

    base_dram_addr = p2s_L_Payload->base_dramAddr_to_load;
    base_picAddr = p2s_L_Payload->base_picAddr_to_store;
    pic_write_ptr = p2s_L_Payload->base_picAddr_to_store;
    next_row_offset_elem = p2s_L_Payload->next_row_offset_elem;
    next_row_offset_dram = (p2s_L_Payload->next_row_offset_elem * 8) >> log2Ceil(8)
    _L_block_row = p2s_L_Payload->_L_block_row;
    _L_block_row_ptr = 0;
    next_slice_offset_pic = p2s_L_Payload->_L_block_row;
    precision = p2s_L_Payload->precision;
    bit_ptr = 0;
    dmaRow = 0;

    schedule(dmaReadEvent, curTick + cyckes(1));
}

void
P2S_L::processDMAReadEvent() {
    // read one row in L Tile
    RequestorID requestorId = system.getRequestorId(this, "P2S_L");

    RequestPtr request = std::make_shared<Request>(
        pioAddr + offset                    // the target MMIO address of cache controller
        sizeof(DMALPayload),                // next_row_offset_elem, base_dram_addr
        0,                          // TODO
        requestorId
    )
    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);
        
    // ask DMA to get data by cache controller
    DMALPayload* dmaLPayload = new DMALPayload{next_row_offset_elem, base_dram_addr};
    
    pkt->dataDynamic(reinterpret_cast<uint8_t*>(&dmaLPayload));
    bool success = DMAPort.sendTimingReq(pkt);
    if (success) {
        base_dram_addr += next_row_offset_dram; // update base_dram_addr for the next round
    }

    
}
void
P2S_L::processBitSliceEvent() {
    // each bit of elements in the whole row

    // extract bits from raw data
    uint64_t bitSlice = extractBits(regArray, bit);

    // determine the address and pack into packets
    RequestorID requestorId = system.getRequestorId(this, "PS2L");

    RequestPtr request = std::make_shared<Request>(
        pioAddr + offset,    // the target MMIO address of cache bank
        sizeof(P2SWritePayload),     // store address + bitSlice
        0,                   // TODO
        requestorId
    );

    PacketPtr bitSlicePkt = new Packet(request, MemCmd::WriteReq);

    P2SWritePayload *p2sWritePayload = new P2SWritePayload{pic_write_ptr, bitSlice};
    bitSlicePkt->dataDynamic(reinterpret_cast<uint8_t*>(p2sWritePayload));

    // enqueue into write queue
    bitSliceQueue.push_back(bitSlicePkt);
    schedule(writeEvent, curTick() + cycles(1));

    // update the next address
    pic_write_ptr += next_slice_offset_pic; // next_slice_offset_pic = _L_block_row(?)

    // finish one bit slice, move on to the next bit
    bit_ptr++;
    if (bit_ptr < precision) {
        schedule(bitSliceEvent, curTick() + cycles(1));
    }
    // finish one row, move on to the next row
    else {
        bit_ptr = 0;
        _L_block_row_ptr++;
        if (_L_block_row_ptr < _L_block_row) {
            schedule(dmaReadEvent, curTick() + cycles(1));
        }
        else {
            // send p2s_done to scheduler
        }
    }
}

uint64_t
P2S_L::extractBits(const std::vector<std::vector<uint8_t>> &arr, uint8_t bit) {
    uint64_t extractedBit = 0;
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

void
P2S_L::processWriteEvent() {
    if (!bitSliceQueue.empty()) {
        PacketPtr pkt = bitSliceQueue.front();
        bool success = cacheBankPort.sendTimingReq(pkt);
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

