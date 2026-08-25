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
      sendEvent([this] { submitNext(); }, name() + ".send_event")
{
    panic_if(transport == nullptr, "%s requires a PicMmioTransport", name());
    buildProtocolSmokeTrace();
    panic_if(requests.empty(), "%s built an empty test trace", name());
}

void
PicTestFrontend::startup()
{
    schedule(sendEvent, clockEdge(startDelay));
}

void
PicTestFrontend::append(const std::vector<PicSetRequest> &writes)
{
    requests.insert(requests.end(), writes.begin(), writes.end());
}

void
PicTestFrontend::buildProtocolSmokeTrace()
{
    // append(PicFunctionBuilder::switchMode(true, 15));
    // append(PicFunctionBuilder::query(255, true));
    // append(PicFunctionBuilder::load(
    //     0x80000000, 0x00000000, {64, 128, 128}, 1));

    // P2S_L
    append(PicFunctionBuilder::p2sl(
        0x80010000, 0x00001000, 64, 128, 2, 7));

    // P2S_R
    append(PicFunctionBuilder::p2sr(
        0x80030000, 0x00000006, {64, 8, 32}, 3, 2, 7, false));

    // P2S_R_T
    append(PicFunctionBuilder::p2sr(
        0x80020000, 0x00000004, {2, 64, 128}, 4, 2, 7, true));

    // append(PicFunctionBuilder::im2col(
    //     0x80030000, 0x81030000, 32, 3, 1, 1, true));
    // append(PicFunctionBuilder::acc(
    //     0x00003000, 0x00004000, 4, 2, 64, 1));
    // append(PicFunctionBuilder::exe(
    //     0x00001000, 1, 5,
    //     {64, 1, 1, 0, 7, 64, true, false, true, true}));
    // append(PicFunctionBuilder::store(
    //     0x00004000, 0x82000000, {64, 128, 128}, 6));
    // append(PicFunctionBuilder::query(6, false));
}

void
PicTestFrontend::submitNext()
{
    panic_if(waitingForTransport,
             "%s attempted to submit while waiting for a response", name());

    if (requests.empty()) {
        if (exitOnFinish) {
            exitSimLoop(name() + " completed the PIC protocol smoke test");
        }
        return;
    }

    waitingForTransport = true;
    transport->submit(
        requests.front(),
        [this](const PicSetResponse &response) { handleResponse(response); });
}

void
PicTestFrontend::handleResponse(const PicSetResponse &response)
{
    panic_if(!waitingForTransport || requests.empty(),
             "%s received an unexpected transport completion", name());
    waitingForTransport = false;

    const PicSetRequest &request = requests.front();
    bool consumeRequest = true;

    if (request.reg == SetRegister::Param) {
        const ParamFields param = unpackParam(request.value);
        if (param.module == ModuleId::Query) {
            const QueryResponse query = unpackQueryResponse(response.data);
            DPRINTF(PicTestFrontend,
                    "QUERY response: finish=%u busy=%u switch_ok=%u mats=%u..%u\n",
                    query.commandFinished, query.schedulerBusy,
                    query.switchSucceeded, query.beginPicMatId,
                    query.endPicMatId);

            const bool immediate = (param.others & 1) != 0;
            if (immediate) {
                consumeRequest = !query.schedulerBusy;
            }
            else if (!waitingForQueuedQueryResult ||
                       queuedQueryCommandId != param.commandId) {
                waitingForQueuedQueryResult = true;
                queuedQueryCommandId = param.commandId;
                consumeRequest = false;
            }
            else {
                waitingForQueuedQueryResult = false;
                consumeRequest = query.commandFinished;
            }
        }
    }

    if (consumeRequest) {
        requests.pop_front();
        ++completedRequests;
    }

    if (requests.empty()) {
        inform("%s completed %llu PIC SET requests",
               name(), static_cast<unsigned long long>(completedRequests));
        if (exitOnFinish) {
            exitSimLoop(name() + " completed the PIC protocol smoke test");
        }
        return;
    }

    schedule(sendEvent, clockEdge(interCommandGap));
}

} // namespace pic
} // namespace gem5
