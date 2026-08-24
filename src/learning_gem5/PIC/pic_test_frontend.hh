#ifndef __LEARNING_GEM5_PIC_PIC_TEST_FRONTEND_HH__
#define __LEARNING_GEM5_PIC_PIC_TEST_FRONTEND_HH__

#include <cstdint>
#include <deque>
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
    EventFunctionWrapper sendEvent;

    std::deque<PicSetRequest> requests;
    bool waitingForTransport = false;
    // first query starts the table lookup and the second returns its result
    bool waitingForQueuedQueryResult = false;
    uint8_t queuedQueryCommandId = 0;
    uint64_t completedRequests = 0;

    void append(const std::vector<PicSetRequest> &writes);
    void buildProtocolSmokeTrace();
    void submitNext();
    void handleResponse(const PicSetResponse &response);

  public:
    PicTestFrontend(const PicTestFrontendParams &params);
    void startup() override;
};

} // namespace pic
} // namespace gem5

#endif // __LEARNING_GEM5_PIC_PIC_TEST_FRONTEND_HH__
