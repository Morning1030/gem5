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
      programCmds(params.program_cmds),
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

namespace
{
// gem5 only: split a trace line into numbers (decimal or 0x hex).
std::vector<uint64_t>
parseFields(const std::string &line)
{
    std::istringstream in(line);
    std::vector<uint64_t> f;
    std::string word;
    while (in >> word)
        f.push_back(std::stoull(word, nullptr, 0));
    return f;
}
} // anonymous namespace

// Scala: ISA.c EXE() -- one EXE (SET_SRC = baseLAddr, SET_DST =
// targetMatID, SET_PARAM CAL). @p fields are EXE()'s arguments in order,
// without cmdID.
void
PicTestFrontend::appendExe(const std::string &fields, uint8_t commandId)
{
    const std::vector<uint64_t> f = parseFields(fields);
    panic_if(f.size() != 12, "%s bad cal command '%s' (12 fields)",
             name(), fields);
    // ISA_defines.scala ISA_EXE: field widths.
    panic_if(f[1] > mask(6) || f[2] > mask(10) || f[3] > mask(2) ||
             f[4] > mask(2) || f[5] > mask(3) || f[6] > mask(3) ||
             f[7] > mask(8) || f[8] > 1 || f[9] > 1 || f[10] > 1 ||
             f[11] > 1,
             "%s cal '%s' does not fit EXE's fields", name(), fields);

    ExeParams exe{
        static_cast<uint16_t>(f[2]), static_cast<uint8_t>(f[3]),
        static_cast<uint8_t>(f[4]), static_cast<uint8_t>(f[5]),
        static_cast<uint8_t>(f[6]), static_cast<uint8_t>(f[7]),
        f[8] != 0, f[9] != 0, f[10] != 0, f[11] != 0};
    append(PicFunctionBuilder::exe(
        f[0], static_cast<uint8_t>(f[1]), commandId, exe));
}

// Scala: ISA.c ACC() -- one ACC (SET_SRC, SET_DST, SET_PARAM ACC). @p
// fields are ACC()'s arguments in order, without cmdID: onChipSrcAddr,
// onChipDstAddr, srcNum, accRowNum, bitWidth (1 = 32-bit).
void
PicTestFrontend::appendAcc(const std::string &fields, uint8_t commandId)
{
    const std::vector<uint64_t> f = parseFields(fields);
    panic_if(f.size() != 5, "%s bad acc command '%s' (5 fields)",
             name(), fields);
    // gem5 packAccParams(): srcNum 3 bits, accRowNum 11 bits, bitWidth 1.
    panic_if(f[2] == 0 || f[2] > mask(3) || f[3] == 0 || f[3] > mask(11) ||
             f[4] > 1,
             "%s acc '%s' does not fit ACC's fields", name(), fields);
    append(PicFunctionBuilder::acc(
        f[0], f[1], commandId, static_cast<uint8_t>(f[2]),
        static_cast<uint16_t>(f[3]), static_cast<uint8_t>(f[4])));
}

void
PicTestFrontend::buildProtocolSmokeTrace()
{
    if (trace == "load") {

        SizeFields size{
            static_cast<uint16_t>(8),
            static_cast<uint16_t>(16),
            static_cast<uint16_t>(64)};

        append(PicFunctionBuilder::load(
            0x80001000,
            0x00020000,
            size,
            commandId++
        ));
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

    if (trace == "store") {

        append(
            PicFunctionBuilder::acc(
                0x00025000,
                0x00028000,
                1,
                2,
                64,
                accBitWidthCode));
        append(
            PicFunctionBuilder::store(
                0x00025000,
                0x80001F00,
                8,
                16,
                64,
                2
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
        for (const std::string &line : calCmds)
            appendExe(line, commandId++);
        return;
    }

    // gem5 only: a short program, every command sent right after the one
    // before it without waiting for it to finish. 'wait <cycles>' stands
    // in for ISA.c waitEnqCmdFree(): its QUERY cannot work yet, since
    // nothing in gem5 marks an EXE as finished in CmdStateHelper.
    if (trace == "program") {
        uint8_t commandId = 1;
        for (const std::string &line : programCmds) {
            std::istringstream in(line);
            std::string op;
            in >> op;
            std::string rest;
            std::getline(in, rest);

            if (op == "cal") {
                appendExe(rest, commandId++);
            } else if (op == "acc") {
                appendAcc(rest, commandId++);
            } else if (op == "wait") {
                const std::vector<uint64_t> f = parseFields(rest);
                panic_if(f.size() != 1, "%s bad wait '%s' (1 field)",
                         name(), line);
                waitBefore[requests.size()] += Cycles(f[0]);
            } else {
                panic("%s unknown program command '%s' (cal, acc, wait)",
                      name(), line);
            }
        }
        panic_if(waitBefore.count(requests.size()),
                 "%s program ends with a wait", name());
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

    // gem5 only: trace 'program' wait -- hold this SET, then send it.
    auto wait = waitBefore.find(sentRequests);
    if (wait != waitBefore.end()) {
        const Cycles cycles = wait->second;
        waitBefore.erase(wait);
        DPRINTF(PicTestFrontend, "wait %llu cycles before SET %llu\n",
                static_cast<unsigned long long>(cycles),
                static_cast<unsigned long long>(sentRequests));
        schedule(sendEvent, clockEdge(cycles));
        return;
    }
    // waitingForTransport = true;

    ++sentRequests;
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
