#include "learning_gem5/PIC/pic_test_frontend.hh"

#include <sstream>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PicTestFrontend.hh"
#include "sim/sim_exit.hh"

namespace gem5
{
namespace pic
{

PicTestFrontend::PicTestFrontend(const PicTestFrontendParams &params)
    : ClockedObject(params),
      transport(params.transport),
      startDelay(params.start_delay),
      interCommandGap(params.inter_command_gap),
      exitOnFinish(params.exit_on_finish),
      trace(params.trace),
      loadCmds(params.load_cmds),
      calCmds(params.cal_cmds),
      accBitWidthCode(static_cast<uint8_t>(params.acc_bit_width_code)),
      sendEvent([this] { processSendEvent(); }, name() + ".send_event")
{
    panic_if(
        transport == nullptr,
        "%s requires a PicMmioTransport",
        name());

    panic_if(
        accBitWidthCode > 1,
        "%s invalid ACC bit-width code %u; "
        "official ACC uses 0=16-bit, 1=32-bit",
        name(),
        static_cast<unsigned>(accBitWidthCode));

    buildProtocolSmokeTrace();

    panic_if(
        requests.empty(),
        "%s built an empty test trace",
        name());
}

void
PicTestFrontend::startup()
{
    schedule(sendEvent, clockEdge(startDelay));
}

void
PicTestFrontend::append(
    const std::vector<PicSetRequest> &writes)
{
    requests.insert(
        requests.end(),
        writes.begin(),
        writes.end());
}

void
PicTestFrontend::buildProtocolSmokeTrace()
{
    // if (trace == "load") {
    //     // One LOAD (SET_SRC, SET_DST, SET_SIZE, SET_PARAM) per entry.
    //     uint8_t commandId = 1;
    //     for (const std::string &line : loadCmds) {
    //         std::istringstream in(line);
    //         std::string src, dst, row, bpr, off;
    //         in >> src >> dst >> row >> bpr >> off;
    //         panic_if(off.empty(), "%s bad load command '%s'", name(), line);

    //         SizeFields size{
    //             static_cast<uint16_t>(std::stoul(row, nullptr, 0)),
    //             static_cast<uint16_t>(std::stoul(bpr, nullptr, 0)),
    //             static_cast<uint16_t>(std::stoul(off, nullptr, 0))};
    //         panic_if(!validSize(size),
    //                  "%s load '%s' does not fit SET_SIZE (row and "
    //                  "bytePerRow 11 bits, offset 15 bits)", name(), line);

    //         append(PicFunctionBuilder::load(
    //             std::stoull(src, nullptr, 0), std::stoull(dst, nullptr, 0),
    //             size, commandId++));
    //     }
    //     return;
    // }
    
    
    if (trace == "store") {
        append(
            PicFunctionBuilder::store(
                0x00003000,
                0x00004000,
                512,
                64,
                128,
                8
            ));

        return;
    }

    if (trace == "p2s") {

        // SET_SIZE follows CMD_resolver_new.scala: row = rows, bytePerRow =
        // bytes read per row (nCols), offset = DRAM stride between rows.
        // Keep the values in sync with BLOCKS in p2s_test_data.py and CMDS
        // in check_p2s_mat.py.

        // P2S_L
        // append(
        //     PicFunctionBuilder::p2sl(
        //         0x80010000,
        //         0x00002000,    // mat 1 (mat 0 is a normal cache way)
        //         64,     // row
        //         64,     // byte per row (unused by P2S_L)
        //         128,    // offset (row stride in bytes)
        //         2,
        //         7));

        // P2S_R
        // append(
        //     PicFunctionBuilder::p2sr(
        //         0x80030000,
        //         0x00000016,    // base arrayID 22 -> mats 1~2
        //         64,     // row
        //         512,     // byte per row (nCols)
        //         2560,    // offset (row stride in bytes)
        //         3,
        //         2,
        //         7,
        //         false));

        // P2S_R_T
        append(
            PicFunctionBuilder::p2srt(
                0x80020000,
                0x00000014,    // base arrayID 20 -> mats 1~2
                128,    // row
                64,     // byte per row (nCols)
                64,     // offset (row stride in bytes)
                4,
                2,
                7,
                true));

        return;
    }

    if (trace == "cal") {
        // Scala: ISA.c EXE() -- one EXE (SET_SRC = baseLAddr, SET_DST =
        // targetMatID, SET_PARAM CAL) per entry, fields in EXE()'s order.
        uint8_t commandId = 1;
        for (const std::string &line : calCmds) {
            std::istringstream in(line);
            std::vector<uint64_t> f;
            std::string word;
            while (in >> word)
                f.push_back(std::stoull(word, nullptr, 0));
            panic_if(f.size() != 12, "%s bad cal command '%s' (12 fields)",
                     name(), line);
            // ISA_defines.scala ISA_EXE: field widths.
            panic_if(f[1] > mask(6) || f[2] > mask(10) || f[3] > mask(2) ||
                     f[4] > mask(2) || f[5] > mask(3) || f[6] > mask(3) ||
                     f[7] > mask(8) || f[8] > 1 || f[9] > 1 || f[10] > 1 ||
                     f[11] > 1,
                     "%s cal '%s' does not fit EXE's fields", name(), line);

            ExeParams exe{
                static_cast<uint16_t>(f[2]), static_cast<uint8_t>(f[3]),
                static_cast<uint8_t>(f[4]), static_cast<uint8_t>(f[5]),
                static_cast<uint8_t>(f[6]), static_cast<uint8_t>(f[7]),
                f[8] != 0, f[9] != 0, f[10] != 0, f[11] != 0};
            append(PicFunctionBuilder::exe(
                f[0], static_cast<uint8_t>(f[1]), commandId++, exe));
        }
        return;
    }

    if (trace == "acc") {
        inform(
            "%s building ACC frontend trace width=%s",
            name(),
            accBitWidthCode ? "32" : "16");

        append(
            PicFunctionBuilder::acc(
                0x00003000,
                0x00004000,
                4,
                2,
                64,
                accBitWidthCode));

        return;
    }
    
    if (trace == "switch") {
        append(
            PicFunctionBuilder::switchMode(
                1,  // alloc
                7,  // nLevels
                1 // commandID
            ));
        return;
    }

    panic(
        "%s unknown PicTestFrontend trace '%s'",
        name(),
        trace.c_str());
}

void
PicTestFrontend::processSendEvent()
{
    // panic_if(
    //     waitingForTransport,
    //     "%s attempted to submit while waiting for a response", name());

    if (requests.empty()) {

        if (exitOnFinish) {
            exitSimLoop(
                name() +
                " completed the PIC protocol smoke test");
        }

        return;
    }
    // waitingForTransport = true;

    transport->submit(
        requests.front(),
        // TBD 應該改成recvTimingResp再forward給handleResponse
        [this](const PicSetResponse &response) {
            handleResponse(response);
        });
    requests.pop_front();
    DPRINTF(PicTestFrontend, "submit req to picMMIOTransport");
    schedule(sendEvent, clockEdge(Cycles(1)));
}

void
PicTestFrontend::handleResponse(
    const PicSetResponse &response)
{
    // panic_if(
    //     !waitingForTransport ||
    //     requests.empty(),
    //     "%s received an unexpected transport completion",
    //     name());

    // waitingForTransport = false;

    const PicSetRequest &request = requests.front();
    bool consumeRequest = true;

    if (request.reg == SetRegister::Param) {

        const ParamFields param =
            unpackParam(request.value);


        if (param.module == ModuleID::QUERY) {

            const QueryResponse query =
                unpackQueryResponse(
                    response.data);


            DPRINTF(
                PicTestFrontend,
                "QUERY response: "
                "finish=%u busy=%u switch_ok=%u "
                "mats=%u..%u\n",
                query.commandFinished,
                query.schedulerBusy,
                query.switchSucceeded,
                query.beginPicMatId,
                query.endPicMatId);


            const bool immediate =
                (param.others & 1) != 0;


            if (immediate) {

                consumeRequest =
                    !query.schedulerBusy;

            } else if (
                !waitingForQueuedQueryResult ||
                queuedQueryCommandId !=
                    param.commandId) {

                waitingForQueuedQueryResult =
                    true;

                queuedQueryCommandId =
                    param.commandId;

                consumeRequest =
                    false;

            } else {

                waitingForQueuedQueryResult =
                    false;

                consumeRequest =
                    query.commandFinished;
            }
        }
    }

    if (consumeRequest) {
        requests.pop_front();
        ++completedRequests;
    }

    if (requests.empty()) {

        inform(
            "%s completed %llu PIC SET requests",
            name(),
            static_cast<unsigned long long>(
                completedRequests));

        if (exitOnFinish) {
            exitSimLoop(name() + " completed the PIC protocol smoke test");
        }

        return;
    }

    // schedule(sendEvent, clockEdge(interCommandGap));
}

} // namespace pic
} // namespace gem5
