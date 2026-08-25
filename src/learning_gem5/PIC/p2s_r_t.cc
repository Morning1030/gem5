#include "learning_gem5/PIC/p2s.hh"
#include "learning_gem5/PIC/scheduler.hh"
#include "sim/system.hh"
#include <cstring>
#include "debug/P2S_R_T.hh"

namespace gem5
{
P2S_R_T::P2S_R_T(P2S_R_TParams *params) :
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
}
bool
P2S_R_T::CPUSidePort::recvTimingReq(PacketPtr pkt) {
    // Just forward to the memobj.
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    } else {
        return true;
    }
}
bool
P2S_R_T::MemSidePort::recvTimingResp(PacketPtr pkt) {
    // fill the response to buffer
    // TODO need sender state row to deal with out of order receiving
    uint8_t *dmaData = pkt->getConstPtr<uint8_t>();
    size_t pktSize = pkt->getSize();

    if (dmaRow < bufArray.size() && pktSize <= sizeof(uint64_t)) {
        std::memcpy(&bufArray[dmaRow], dmaData, pktSize);
    } else {
        panic("P2S_R_T: bufArray buffer overflow! dmaRow=%u\n", dmaRow);
    }

    delete pkt;
    dmaRow++;

    if (dmaRow == bufArray.size()) {
        dmaRow = 0;
        bit_ptr = 0;
        // finish filling dma into buffer
        schedule(bitSliceEvent, curTick() + cycles(1));

    } else {
        // need to wait for other dmaRows to finish
    }

    // reformat the buffer from dma data to extract bit format
    // for(int i = 0; i < 64; i++) {
    //     uint8_t bitShift = (i % 8) * 8;
    //     bufArrayOutReFormat[i] = (bufArray[i / 8] >> bitShift) & 0xFF;
    // }
}
void
P2S_R_T::handleRequest(PacketPtr pkt) {
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
    bit_ptr = 0;
    row_store_ptr = 0;
    schedule(dmaReadEvent, curTick() + cycles(1));
}
void
P2S_R_T::processDMAReadEvent() {
    // read one row in R Tile
    RequestorID requestorId = system.getRequestorId(this, "P2S_R_T");

    RequestPtr request = std::make_shared<Request>(
        pioAddr + offset,                    // the target MMIO address of cache controller
        sizeof(DMARTPayload),                // next_row_offset_elem, base_dram_addr
        0,                          // TODO
        requestorId
    )
    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);

    // ask DMA to get data by cache controller
    DMARTPayload* dmaRTPayload = new DMARTPayload{nCols, dramAddr};
    pkt->dataDynamic(reinterpret_cast<uint8_t*>(dmaRTPayload));
    bool success = DMAPort.sendTimingReq(pkt);
    if (success) {
        dramAddr += next_row_offset_bytes; // update base_dram_addr for the next round
    }
    else {
        // need to retry
    }
}
void
P2S_R_T::processBitSliceEvent() {

    // extract bits from raw data
    uint64_t bitSlice = extractBits(bufArrayOutReFormat, bit_ptr);

    // determine the address
    uint64_t curArrayID = base_arrayID_to_store + arrayID_offset[bit_ptr];
    uint64_t arrayAddrEnq = currArrayID << log2Ceil(coreCfg.wordlineNums) + row_store_ptr;
    P2SWritePayload *p2sWritePayload = new P2SWritePayload{arrayAddrEnq, bitSlice};

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
    bitSlicePkt.dataDynamic(reinterpret_cast<uint8_t*>(p2sWritePayload));

    // enqueue into write queue
    bitSliceQueue.push_back(bitSlicePkt);
    // write to cache bank
    schedule(writeEvent, curTick() + cycles(1));

    bit_ptr++;
    if (bit_ptr < precision) {
        schedule(bitSliceEvent, curTick() + cycles(1));
    }
    else {
        bit_ptr = 0;
        row_store_ptr++;
        if (row_store_ptr < nRows) {
            schedule(dmaReadEvent, curTick() + cycles(1));
        }
        else {
            // send p2s_done to scheduler
        }
    }

    // send p2s_done to scheduler

}
void
P2S_R_T::processWriteEvent() {
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

uint64_t
P2S_R_T::extractBits(const std::vector<uint_8> &buf, uint8_t bit) {
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
}
