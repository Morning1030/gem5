# 2 banks x 4 ways (way 0 = normal cache, no FSM/port) -> 6 PIC Mats, one
# SimObject each, all sharing ONE AutoLoadL, which now reads through
# AccessBankArb (round-robin, no real bank storage yet -- see
# access_bank_arb.cc, placeholder read data). Checks the L-vector load
# path end to end: Mat -> AutoLoadL -> AccessBankArb -> AutoLoadL ->
# Mat's vecBuf.
# Run with --debug-flags=MatBank,AutoLoadL,PICBankArb to see it all.

import m5
from m5.objects import *

NUM_BANKS = 2
NUM_WAYS = 4
FIRST_PIC_WAY = 1
L_ROWS = 2  # L_block_row = row count per Mat

system = System()
root = Root(full_system=False, system=system)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = 'timing'
system.mem_ranges = [AddrRange(start=0x0, size='1MiB')]

system.autoload_l = AutoLoadL(system=system)
system.bank_arb = AccessBankArb(num_banks=NUM_BANKS)
system.autoload_l.cb_port = system.bank_arb.autoload_side

# AccessBankArb requires exactly 3 p2s_side connections; not exercised by
# this L-vector-only test, just present to satisfy the port count.
system.p2s_l = P2S_L(system=system)
system.p2s_r = P2S_R(system=system)
system.p2s_r_t = P2S_R_T(system=system)
system.p2s_l.cb_port = system.bank_arb.p2s_side[0]
system.p2s_r.cb_port = system.bank_arb.p2s_side[1]
system.p2s_r_t.cb_port = system.bank_arb.p2s_side[2]

# One Mat SimObject per (bank, way). Connection order matches the RTL's
# matID*numBanks+bankID interleaving confirmed against SysConfig.scala:
# same in-bank way, bank varies fastest.
system.mats = [
    Mat(
        system=system,
        bank_index=b,
        way=w,
        n_buf=1, n_cal=3, acc_width=16,
        r_block_row=1, l_block_row=L_ROWS, l_precision=0,
        l_vec_fetch_addr=(w * NUM_BANKS + b) * L_ROWS,
    )
    for w in range(FIRST_PIC_WAY, NUM_WAYS)
    for b in range(NUM_BANKS)
]
for mat in system.mats:
    mat.load_l_port = system.autoload_l.inst_port

m5.instantiate()

# No real bank storage behind AccessBankArb yet -- every load returns the
# placeholder 0 (see access_bank_arb.cc's processTickEvent()). This run
# only checks the request/response plumbing, not data correctness.

finished = 0
while finished < len(system.mats):
    exit_event = m5.simulate()
    print('Exit @ tick {}: {}'.format(m5.curTick(), exit_event.getCause()))
    if exit_event.getCause() == 'Mat finished':
        finished += 1
