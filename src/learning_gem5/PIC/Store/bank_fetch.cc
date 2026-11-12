#include "learning_gem5/PIC/Store/bank_fetch.hh"
#include "learning_gem5/PIC/Store/store_ctrl.hh"
#include "sim/system.hh"

#include <bit>
#include <algorithm>
#include <cmath>
#include <cstring>

#include "debug/BankFetch.hh"
namespace gem5
{
constexpr uint64_t pageSize = 4096;     // 4KB
constexpr uint64_t bitlineNum = 64;

BankFetch::BankFetch(const BankFetchParams &params) :
    ClockedObject(params),
    instPort(params.name + ".inst_port", this),
    cacheBankPort(params.name + ".cb_port", this, PICPortID::CB),
    dmaWritePort(params.name + ".dma_port", this, PICPortID::DMA),
    requestorId(params.system->getRequestorId(this, "BankFetch")),
    pendingReqPkt(nullptr),
    bankFetchDone(true),
    cacheBankData(pageSize + bitlineNum / 8),
    pageFetchEvent([this]{this->processPageFetchEvent();}, "pageFetchEvent"),
    atomicBankFetchEvent([this]{this->processAtomicBankFetchEvent();}, "atomicBankFetchEvent"),
    alignEvent([this]{this->processAlignEvent();}, "alignEvent"),
    alignBlocked(false)
{}
Port&
BankFetch::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port")
        return instPort;
    if (if_name == "cb_port")
        return cacheBankPort;
    if (if_name == "dma_port")
        return dmaWritePort;

    return ClockedObject::getPort(if_name, idx);
}
BankFetch::CPUSidePort::CPUSidePort(
    const std::string &name,
    BankFetch *owner) :
    ResponsePort(name),
    owner(owner),
    blockedPacket(nullptr)
{}
bool
BankFetch::CPUSidePort::recvTimingReq(PacketPtr pkt) {
    return owner->handleRequest(pkt);
}
void
BankFetch::CPUSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    if (sendTimingResp(pkt)) {
        owner->pendingReqPkt = nullptr;
        blockedPacket = nullptr;

        DPRINTF(BankFetch, "Store COMPLETE: send back to StoreCtrl\n");
    }
    else blockedPacket = pkt;
}
void
BankFetch::CPUSidePort::recvRespRetry()
{
    // retry to send resp to scheduler
    assert(blockedPacket != nullptr);

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
}

BankFetch::MemSidePort::MemSidePort(
    const std::string &name,
    BankFetch *owner,
    PICPortID picPortID) :
    RequestPort(name),
    owner(owner),
    picPortID(picPortID),
    blockedPacket(nullptr)
{}
bool
BankFetch::MemSidePort::recvTimingResp(PacketPtr pkt) {
    return owner->handleResponse(this->picPortID, pkt);
}
void
BankFetch::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");
    if (sendTimingReq(pkt)) {
        // to dma
        if (picPortID == PICPortID::DMA) {
            // if (owner->cacheBankPort.needRetry == true) {
            //     owner->cacheBankPort.sendRetryReq();
            // }
            // DPRINTF(BankFetch, "Successfully send request to DMA\n");
            if (owner->alignBlocked && !owner->alignEvent.scheduled()) {
                DPRINTF(BankFetch, "Wake up align event\n");
                owner->schedule(owner->alignEvent, owner->clockEdge(Cycles(1)));
                owner->alignBlocked = false;
            }

        }
        // to cache bank
        else if(pkt->isRead()) {
            // DPRINTF(BankFetch, "Successfully send request to bank fetch\n");
        }
        else {
            DPRINTF(BankFetch, "Cannot recognize port\n");
        }
    }
    else {
        DPRINTF(BankFetch, "dmaWritePort is rejected\n");
        blockedPacket = pkt;
    }
    // if (owner->pendingReqPkt == nullptr) DPRINTF(BankFetch, "pendingReqPkt is nullptr after dma send packet.\n");
}
void
BankFetch::MemSidePort::recvReqRetry()
{
    if (picPortID == PICPortID::CB) {
        if (!owner->atomicBankFetchEvent.scheduled()) {
            owner->schedule(owner->atomicBankFetchEvent, owner->clockEdge(Cycles(1)));
        }
    }
    else if (picPortID == PICPortID::DMA) {
        panic_if(blockedPacket == nullptr, "There's no blockedPacket to send");
        DPRINTF(BankFetch, "Retry DMA request\n");
        PacketPtr pkt = blockedPacket;
        blockedPacket = nullptr;
        if (owner->pendingReqPkt == nullptr) DPRINTF(BankFetch, "After recvReqRetry pending req is nullptr\n");
        sendPacket(pkt);
        // might fail again, wait for another req retry
    }
    else {
        DPRINTF(BankFetch, "Cannot recognize memsideport portID\n");
    }
}
bool
BankFetch::handleRequest(PacketPtr pkt)
{
    if (pendingReqPkt != nullptr) return false;
    pendingReqPkt = pkt;
    bankFetchDone = false;
    if (pendingReqPkt == nullptr) DPRINTF(BankFetch, "Recv nullptr request pkt");

    BankFetchSenderState* b = pkt->findNextSenderState<BankFetchSenderState>();
    panic_if(b == nullptr, "missing BankFetchSenderState");

    const BankFetchReq& bankFetchReq = b->req;

    src = bankFetchReq.src;
    dst = bankFetchReq.dst;
    nBytes = bankFetchReq.nBytes;
    // DMA variables
    v_addr_ptr = dst; // DRAM address ptr
    final_v_addr = dst + nBytes;
    next_page_begin_addr = ((v_addr_ptr >> __builtin_ctz(pageSize)) + 1U) << __builtin_ctz(pageSize);
    
    // bank fetch variables
    nextBankAddr = src;
    validNBytes = (final_v_addr > next_page_begin_addr) ? next_page_begin_addr - v_addr_ptr : final_v_addr - v_addr_ptr;
    startRow = nextBankAddr >> __builtin_ctz(bitlineNum / 8);
    endRow = (nextBankAddr + validNBytes - 1) >> __builtin_ctz(bitlineNum / 8);
    bankRowPtr = startRow;
    rowsReceived = 0;
    DmaReqCnt = ((final_v_addr - 1) >> __builtin_ctz(pageSize)) - (dst >> __builtin_ctz(pageSize)) + 1;
    DmaRecvCnt = 0;

    DPRINTF(BankFetch, "startRow = %d, endRow = %d\n", startRow, endRow);
        
    // isLastPage= next_page_begin_addr>=final_v_addr
    if (!pageFetchEvent.scheduled()) {
        schedule(pageFetchEvent, clockEdge(Cycles(1)));
    }
    return true;

}
bool
BankFetch::handleResponse(PICPortID picPortID, PacketPtr pkt)
{
    if (picPortID == PICPortID::DMA) {
        
        // means this page DMA write to DRAM is sent
        DmaRecvCnt++;
        DPRINTF(BankFetch, "DmaReqCnt:%d DmaRecvCnt:%d\n", DmaReqCnt, DmaRecvCnt);
        if (bankFetchDone && DmaRecvCnt == DmaReqCnt) {
            // if (pendingReqPkt == nullptr) DPRINTF(BankFetch, "pendingReqPkt is nullptr\n");
            pendingReqPkt->makeResponse();
            instPort.sendPacket(pendingReqPkt);
            // if (pendingReqPkt == nullptr) DPRINTF(BankFetch, "pendingReqPkt is nullptr after cpu sendPacket\n");
        }
    }
    else if (picPortID == PICPortID::CB) {
        panic_if(pkt->getSize() != bitlineNum / 8, "Receive more than one workline data!\n");

        std::memcpy(cacheBankData.data() + rowsReceived * (bitlineNum / 8),
            pkt->getConstPtr<uint8_t>(),
            pkt->getSize()
        );
        
        rowsReceived++;

        if (rowsReceived == (endRow - startRow + 1)) {
            // mask out some cacheBankData
            if (dmaWritePort.blockedPacket == nullptr && !alignEvent.scheduled()) 
                schedule(alignEvent, clockEdge(Cycles(1)));

            else {
                DPRINTF(BankFetch, "alignEvent is blocked\n");
                alignBlocked = true;
            }
        }
    }
    delete pkt;
    return true;
}
void
BankFetch::processPageFetchEvent()
{
    // TBD what format does arbiter accept
    RequestPtr request = std::make_shared<Request>(
        bankRowPtr,
        (bitlineNum / 8),
        0,
        requestorId
    );
    PacketPtr pkt = new Packet(request, MemCmd::ReadReq);
    pkt->allocate();

    bankFetchAtomicQueue.push_back(pkt);
    if (pendingReqPkt == nullptr) {
        // DPRINTF(BankFetch, "pendingReqPkt is nullptr in pageFetchEvent\n");
    }
    if (!atomicBankFetchEvent.scheduled()) {
        schedule(atomicBankFetchEvent, clockEdge(Cycles(1)));
    }
    if (bankRowPtr < endRow) {
        bankRowPtr++;
        schedule(pageFetchEvent, clockEdge(Cycles(1)));
    }
}
void
BankFetch::processAtomicBankFetchEvent()
{
    if (!bankFetchAtomicQueue.empty()) {
        PacketPtr pkt = bankFetchAtomicQueue.front();
        bool success = cacheBankPort.sendTimingReq(pkt);
        if (success) {
            uint64_t req = pkt->getAddr();
            uint16_t mat, bank, array, row;
            row = (req & 0b111111111);
            array = (req >> 9) & 0b11;
            bank = (req >> 11) & 0b11;
            mat = (req >> 13) & 0b1111;
            DPRINTF(BankFetch, "Read Request for %d/%d/%d/%d\n", mat, bank, array, row);   
            bankFetchAtomicQueue.pop_front();
            schedule(atomicBankFetchEvent, clockEdge(Cycles(1)));
        }
        else {
            // bankfetch is stalled, need to wait for cache bank notify to retry
            // do nothing and wait until notification
            // DPRINTF(BankFetch, "Cache bank busy, stalling bankFetchEvent.\n");
        }
    }
}
void
BankFetch::processAlignEvent()
{
    // algin the response from cache bank to a dma write requests
    offset = nextBankAddr & 0b111;

    RequestPtr request = std::make_shared<Request>(
        v_addr_ptr,
        validNBytes,
        0,
        requestorId
    );
    PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
    pkt->allocate();
    pkt->setData((cacheBankData.data() + offset));
    
    dmaWritePort.sendPacket(pkt);
    
    v_addr_ptr = next_page_begin_addr;
    nextBankAddr += validNBytes;
    // not only one page
    if (v_addr_ptr < final_v_addr) {
        // update address for next page 
        next_page_begin_addr = ((v_addr_ptr >> __builtin_ctz(pageSize)) + 1U) << __builtin_ctz(pageSize);

        // validNBytes, startRow, endRow for this page
        validNBytes = (final_v_addr > next_page_begin_addr) ? next_page_begin_addr - v_addr_ptr : final_v_addr - v_addr_ptr;
        startRow = nextBankAddr >> __builtin_ctz(bitlineNum / 8);
        endRow = (nextBankAddr + validNBytes - 1) >> __builtin_ctz(bitlineNum / 8);
        bankRowPtr = startRow;
        rowsReceived = 0;
            
        if (!pageFetchEvent.scheduled())
            schedule(pageFetchEvent, clockEdge(Cycles(1)));
    }
    else {
        bankFetchDone = true;
        // pendingReqPkt->makeResponse();
        // instPort.sendPacket(pendingReqPkt);
    }
    // else DPRINTF(BankFetch, "DMAWrite is stalled, wait for Request Retry\n");
}
}