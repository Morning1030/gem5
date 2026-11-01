# add_switch_ctrl -- builds the SwitchController (RTL SwitchCtl, inside the
# InclusiveCache Scheduler) for a PICCache and wires it to the cache's
# CacheController. Kept out of PICCache so a testbench decides whether it
# wants the switch at all.
#
#   cache = PICCache(system, with_ctrl=True)
#   switch = add_switch_ctrl(system, cache)   # -> system.pic_switch
#
# Wired in here:
#   switch.cache_side[0] -> cache.ctrl.pic_ctrl_port   (dir query)
#   switch.cache_side[1] -> cache.ctrl.pic_flush_port  (flush)
# Connect from outside:
#   switch.inst_port <- PolymorPIC Scheduler.sc_port
#
# The cache must be built with pic_way_mask=0: PIC ways only come from
# ALLOC (RTL RegInit(0)).

from m5.objects import SwitchController

from pic_cache import FIRST_PIC_WAY, NUM_BANKS, NUM_SETS, NUM_WAYS


def add_switch_ctrl(system, cache):
    if cache.ctrl is None:
        raise ValueError(
            "add_switch_ctrl: PICCache needs with_ctrl=True")
    if cache.pic_way_mask != 0:
        # Same rule as SwitchController::init(), caught earlier
        raise ValueError(
            "add_switch_ctrl: PICCache needs pic_way_mask=0 "
            "(PIC ways only come from ALLOC)")

    system.pic_switch = SwitchController(
        num_sets=NUM_SETS,
        num_ways=NUM_WAYS,
        n_way_per_level=1,
        pic_avail_levels=NUM_WAYS - FIRST_PIC_WAY,
        total_mat_num=NUM_BANKS * NUM_WAYS,
        cache=cache.ctrl,
    )
    switch = system.pic_switch
    # Order is the idx: SwitchController::DirQuery = 0, Flush = 1
    switch.cache_side = [cache.ctrl.pic_ctrl_port, cache.ctrl.pic_flush_port]
    return switch
