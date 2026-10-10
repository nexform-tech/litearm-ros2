// service_helpers.hpp — the two helpers every service handler is built from.
//
// They live in a header rather than in the node's translation unit because the handler
// implementations are split by area across several files, and a handler that reported its
// outcome differently from the next one would be worse than the duplication.

#ifndef LITEARM_DRIVER__SERVICE_HELPERS_HPP_
#define LITEARM_DRIVER__SERVICE_HELPERS_HPP_

#include <string>

#include "litearm/arm.hpp"
#include "litearm/clock.hpp"
#include "litearm_driver/safety_gates.hpp"

namespace litearm_driver
{

/// Copy a rejected gate into a service response.
template<class Response>
void fill_failure(Response & response, const GateResult & gate)
{
  response.success = false;
  response.message = gate.message;
}

/// Run one SDK call and fill the response from its outcome.
///
/// The SDK reports every refusal as an exception whose message names the reason: a clamped
/// register, a required reset, a mode the firmware does not implement. The driver passes
/// that text through unchanged, because a caller who sees the firmware's own words can act
/// on them and a generic "failed" cannot be acted on.
template<class Response, class Fn>
void run_command(litearm::Arm * arm, const char * what, Response & response, Fn && fn)
{
  const GateResult link = check_connected(arm != nullptr && arm->is_connected());
  if (!link.ok) {
    fill_failure(response, link);
    return;
  }
  try {
    fn(*arm);
    response.success = true;
    if (response.message.empty()) {
      response.message = std::string(what) + ": ok";
    }
  } catch (const litearm::LiteArmError & error) {
    response.success = false;
    response.message = std::string(what) + " failed: " + error.what();
  } catch (const std::exception & error) {
    response.success = false;
    response.message = std::string(what) + " failed: " + error.what();
  }
}

/// Age of the last status frame in seconds, or -1 when none arrived yet.
inline double frame_age(double stamp_s)
{
  if (stamp_s <= 0.0) {
    return -1.0;
  }
  return litearm::steady_clock_instance().now_s() - stamp_s;
}

}  // namespace litearm_driver

#endif  // LITEARM_DRIVER__SERVICE_HELPERS_HPP_
