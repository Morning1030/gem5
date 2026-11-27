#ifndef __LEARNING_GEM5_PIC_PIC_TEST_FRONTEND_HH__
#define __LEARNING_GEM5_PIC_PIC_TEST_FRONTEND_HH__

#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "learning_gem5/PIC/pic_function_builder.hh"
#include "learning_gem5/PIC/pic_mmio_transport.hh"
#include "params/PicTestFrontend.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

namespace gem5
{
namespace pic
{

class PicTestFrontend : public ClockedObject
{
  private:
    PicMmioTransport *const transport;
    const Cycles startDelay;
    const Cycles interCommandGap;
    const bool exitOnFinish;
    const std::string trace;
    const std::vector<std::string> loadCmds;
    const std::vector<std::string> calCmds;
    const std::vector<std::string> programCmds;
    const uint8_t accBitWidthCode;

    EventFunctionWrapper sendEvent;

    std::deque<PicSetRequest> requests;

    /** gem5 only: trace 'program' waits, keyed by the index (in send
     *  order) of the SET they hold back. */
    std::map<uint64_t, Cycles> waitBefore;
    /** SETs handed to the transport so far. */
    uint64_t sentRequests = 0;

    // bool waitingForTransport = false;

    // First queued QUERY starts the lookup and the second returns its result.
    bool waitingForQueuedQueryResult = false;
    uint8_t queuedQueryCommandId = 0;

    uint64_t completedRequests = 0;

    void append(const std::vector<PicSetRequest> &writes);
    void appendExe(const std::string &fields, uint8_t commandId);
    void appendAcc(const std::string &fields, uint8_t commandId);
    void buildProtocolSmokeTrace();
    void processSendEvent();
    void handleResponse(const PicSetResponse &response);

  public:
    PicTestFrontend(const PicTestFrontendParams &params);

    void startup() override;
};

} // namespace pic
} // namespace gem5

#endif // __LEARNING_GEM5_PIC_PIC_TEST_FRONTEND_HH__
