#include "learning_gem5/PIC/pic_test_frontend.hh"

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
      accBitWidthCode(
          static_cast<uint8_t>(params.acc_bit_width_code)),
      sendEvent(
          [this] { submitNext(); },
          name() + ".send_event")
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
    schedule(
        sendEvent,
        clockEdge(startDelay));
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
    if (trace == "p2s") {

        // P2S_L
        append(
            PicFunctionBuilder::p2sl(
                0x80010000,
                0x00001000,
                64,
                128,
                2,
                7));

        // P2S_R
        append(
            PicFunctionBuilder::p2sr(
                0x80030000,
                0x00000006,
                {64, 8, 32},
                3,
                2,
                7,
                false));

        // P2S_R_T
        append(
            PicFunctionBuilder::p2sr(
                0x80020000,
                0x00000004,
                {2, 64, 128},
                4,
                2,
                7,
                true));

        return;
    }


    if (trace == "acc") {

        /*
         * ACC command:
         *
         * src          = 0x3000
         * dst          = 0x4000
         * command ID   = 4
         * source count = 2
         * row count    = 64
         *
         * accBitWidthCode:
         *   0 -> four independent 16-bit lanes
         *   1 -> two independent 32-bit lanes
         */
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


    panic(
        "%s unknown PicTestFrontend trace '%s'",
        name(),
        trace.c_str());
}

void
PicTestFrontend::submitNext()
{
    panic_if(
        waitingForTransport,
        "%s attempted to submit while waiting for a response",
        name());


    if (requests.empty()) {

        if (exitOnFinish) {
            exitSimLoop(
                name() +
                " completed the PIC protocol smoke test");
        }

        return;
    }


    waitingForTransport = true;


    transport->submit(
        requests.front(),
        [this](const PicSetResponse &response) {
            handleResponse(response);
        });
}

void
PicTestFrontend::handleResponse(
    const PicSetResponse &response)
{
    panic_if(
        !waitingForTransport ||
        requests.empty(),
        "%s received an unexpected transport completion",
        name());


    waitingForTransport = false;


    const PicSetRequest &request =
        requests.front();


    bool consumeRequest = true;


    if (request.reg == SetRegister::Param) {

        const ParamFields param =
            unpackParam(request.value);


        if (param.module == ModuleId::Query) {

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

            exitSimLoop(
                name() +
                " completed the PIC protocol smoke test");
        }


        return;
    }


    schedule(
        sendEvent,
        clockEdge(interCommandGap));
}

} // namespace pic
} // namespace gem5
