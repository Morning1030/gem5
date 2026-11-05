#include "learning_gem5/PIC/mat/mat_setup_router.hh"

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/MatBank.hh"
#include "learning_gem5/PIC/Control/scheduler.hh"

namespace gem5
{

// Scala: BanksShell.scala L84-89 / Bank.scala L63-65 -- the matID each Mat
// answers to on set_up_addr. gem5 only: built once here instead of the
// RTL's per-bank / per-Mat comparators.
MatSetUpRouter::MatSetUpRouter(const MatSetUpRouterParams &params) :
    ClockedObject(params),
    cmdPort(params.name + ".cmd_side", this),
    numBanks(params.num_banks),
    respEvent([this] { sendResp(); }, name() + ".respEvent")
{
    fatal_if(numBanks == 0 || !isPowerOf2(numBanks),
             "%s: num_banks must be a power of 2, got %u", name(), numBanks);
    for (Mat *mat : params.mats) {
        const unsigned id = mat->wayIndex() * numBanks + mat->bank();
        fatal_if(!matById.emplace(id, mat).second,
                 "%s: two Mats have matID %u (bank %u way %u)", name(), id,
                 mat->bank(), mat->wayIndex());
    }
}

Port &
MatSetUpRouter::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "cmd_side")
        return cmdPort;
    return ClockedObject::getPort(if_name, idx);
}

// Scala: BanksShell.scala L84-99 + Bank.scala L63-76 -- decode set_up_addr
// into (bank, Mat), hand that Mat the set-up and pulse its exec. The
// CalPayload -> SetUpIO field mapping is CMD_resolver_new.enq_EXE's.
bool
MatSetUpRouter::handleCmd(PacketPtr pkt)
{
    // The Scheduler sends the next EXE only after this one's response.
    panic_if(pendingResp != nullptr, "%s: EXE while the last one's "
             "response is still pending", name());
    panic_if(!pkt->isWrite() || pkt->getSize() != sizeof(CalPayload),
             "%s: expected a %zu-byte CalPayload write, got %s", name(),
             sizeof(CalPayload), pkt->print());

    const CalPayload *exe = pkt->getConstPtr<CalPayload>();

    // SysConfig.get_bankID / get_in_bank_matID: matID = {way, bank}.
    const unsigned matId = static_cast<unsigned>(exe->set_up_addr);
    auto it = matById.find(matId);
    fatal_if(it == matById.end(), "%s: EXE to matID %u (bank %u way %u), "
             "no such Mat", name(), matId, matId % numBanks,
             matId / numBanks);

    mat_fsm::SetUpIO setUp;
    setUp.exec = true;
    setUp.nBuf = exe->nBuf;
    setUp.nCal = exe->nCal;
    // PolymorPIC_Config.scala L32-33: ACC_32BIT = true, ACC_16BIT = false.
    setUp.accWidth = exe->accWidth ? mat_fsm::AccWidth32Bit
                                   : mat_fsm::AccWidth16Bit;
    setUp.R_base_bit = exe->_R_base_bit;
    // Controller.scala L31: SetUpIO._R_block_row is log2(512) = 9 bits, so
    // the RTL keeps only the low 9 bits of R_Valid_nRols.
    setUp.R_block_row = exe->_R_block_row &
                        (mat_fsm::CArrayWordlineNums - 1);
    setUp.L_block_row = exe->_L_block_row;
    setUp.L_precision = exe->L_precision;
    setUp.L_vec_fetch_addr = exe->_L_vec_fetch_addr;
    setUp.signed_L = exe->signed_L;
    setUp.signed_R_last_exist = exe->signed_R_last_exist;

    DPRINTF(MatBank, "%s: EXE -> matID %u (bank %u way %u)\n", name(),
            matId, it->second->bank(), it->second->wayIndex());
    it->second->startCal(setUp);

    pkt->makeResponse();
    pendingResp = pkt;
    schedule(respEvent, nextCycle());
    return true;
}

// gem5 only: send the EXE's response to the Scheduler, or hold it until
// the Scheduler retries.
void
MatSetUpRouter::sendResp()
{
    if (cmdPort.sendTimingResp(pendingResp))
        pendingResp = nullptr;
    else
        respBlocked = true;
}

// gem5 only: the Scheduler can take the held response now.
void
MatSetUpRouter::handleRespRetry()
{
    panic_if(!respBlocked, "%s: unexpected response retry", name());
    respBlocked = false;
    sendResp();
}

} // namespace gem5
