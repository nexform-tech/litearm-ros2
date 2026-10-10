// litearm_driver_diagnostics.cpp — handlers for the link, host diagnostics and the
// firmware's control-tick log.
//
// Everything here reads. `reconnect` re-opens the session (which is what recovers from an
// unplugged cable), the diagnostics service reports what this process has seen, and the log
// services read the firmware's own 300 Hz recording of what the control loop did.

#include "litearm_driver/litearm_driver_node.hpp"
#include "litearm_driver/service_helpers.hpp"

#include <cstdlib>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "litearm/log.hpp"
#include "litearm/protocol.hpp"

namespace litearm_driver
{

namespace
{

/// The firmware response ids worth reporting a rate for: the status stream, the two
/// handshake replies, and the three variable-length replies whose rate says how busy the
/// session was.
const std::vector<std::pair<uint8_t, const char *>> & rate_ids()
{
  static const std::vector<std::pair<uint8_t, const char *>> ids{
    {litearm::proto::RSP_STATUS, "status"},
    {litearm::proto::RSP_ACK, "ack"},
    {litearm::proto::RSP_ERR, "err"},
    {litearm::proto::RSP_FIRMWARE, "firmware"},
    {litearm::proto::RSP_DETAIL, "detail"},
    {litearm::proto::RSP_MODEL_STATUS, "model_status"},
    {litearm::proto::RSP_LOG_DATA, "log_data"},
  };
  return ids;
}

}  // namespace

void LitearmDriverNode::handle_reconnect(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  try {
    if (arm_ == nullptr) {
      response->success = false;
      response->message = "the node is not configured: no SDK session to rebuild";
      return;
    }
    // close() + connect() in the SDK. The reader thread is restarted and the handshake is
    // replayed; the licence record is re-read because a different device could answer.
    arm_->reconnect();
    refresh_license_record();
    response->success = true;
    response->message = "link re-established on " + arm_->port_string() +
      " (firmware " + arm_->firmware() + ")";
  } catch (const std::exception & error) {
    response->success = false;
    response->message = std::string("reconnect failed: ") + error.what();
  }
}

void LitearmDriverNode::handle_get_tcp(
  const litearm_msgs::srv::GetTcp::Request::SharedPtr request,
  litearm_msgs::srv::GetTcp::Response::SharedPtr response)
{
  (void)request;
  const GateResult link = check_connected(connected());
  if (!link.ok) {
    fill_failure(*response, link);
    return;
  }
  try {
    const auto answer = arm_->get_tcp();
    if (!answer.value.has_value()) {
      response->success = false;
      response->message =
        "the firmware did not answer a tool pose (this is a firmware 1.8.0 command, and the "
        "arm has to have a solved pose)";
      return;
    }
    for (std::size_t i = 0; i < 6; ++i) {
      response->pose[i] = (*answer.value)[i];
    }
    response->success = true;
    response->message = "pose in [x, y, z, roll, pitch, yaw]";
  } catch (const std::exception & error) {
    response->success = false;
    response->message = std::string("get_tcp failed: ") + error.what();
  }
}

void LitearmDriverNode::handle_get_diagnostics(
  const litearm_msgs::srv::GetDiagnostics::Request::SharedPtr request,
  litearm_msgs::srv::GetDiagnostics::Response::SharedPtr response)
{
  (void)request;
  const GateResult link = check_connected(connected());
  if (!link.ok) {
    fill_failure(*response, link);
    return;
  }
  try {
    const litearm::HostStats stats = arm_->host_stats();
    const auto banner = arm_->banner_version();
    response->connected = arm_->is_connected();
    response->in_dfu = arm_->is_in_dfu();
    response->cart_supported = arm_->cart_supported();
    response->cart_probe_silent = stats.cart_probe_silent;
    response->joint_count = arm_->n();
    response->status_seq = arm_->status_seq();
    response->port = arm_->port_string();
    response->firmware = arm_->firmware();
    response->banner_version = banner.has_value() ? *banner : std::string();
    response->last_reset_reason = arm_->last_reset_reason();
    response->dropped = stats.dropped;
    response->bad_status_frames = stats.bad_status_frames;
    response->flush_failures = stats.flush_failures;
    response->cart_evicted_unclaimed = stats.cart_evicted_unclaimed;
    response->cart_extra_replies = stats.cart_extra_replies;
    for (const auto & id : rate_ids()) {
      response->msg_ids.push_back(id.first);
      response->msg_hz.push_back(arm_->msg_hz(id.first));
    }
    response->success = true;
    response->message = "host-side counters and the per-id message rates of this session";
  } catch (const std::exception & error) {
    response->success = false;
    response->message = std::string("get_diagnostics failed: ") + error.what();
  }
}

void LitearmDriverNode::handle_kin_bench(
  const litearm_msgs::srv::KinBench::Request::SharedPtr request,
  litearm_msgs::srv::KinBench::Response::SharedPtr response)
{
  const GateResult timeout = check_timeout(request->timeout, "timeout");
  if (!timeout.ok) {
    fill_failure(*response, timeout);
    return;
  }
  run_command(arm_.get(), "kin_bench", *response, [&request, &response](litearm::Arm & arm) {
    const litearm::KinBenchResult result = arm.diag().kin_bench(
      request->timeout > 0.0 ? request->timeout : litearm::KIN_BENCH_TIMEOUT).value;
    response->raw = result.raw;
    for (const auto & entry : result.timings) {
      std::string line = entry.first + "=";
      for (std::size_t i = 0; i < entry.second.size(); ++i) {
        line += (i == 0 ? "" : ",") + std::to_string(entry.second[i]);
      }
      response->timings.push_back(line);
    }
    response->crc_errors = result.crc_errors();
    response->reply_dropped = result.reply_dropped();
    response->can_tx_fail = result.can_tx_fail();
    response->loop_max_kcycle = result.loop_max_kcycle();
    response->loop_overruns = result.loop_overruns();
    response->rx_fifo_lost_motor = result.rx_fifo_lost_motor();
    response->rx_fifo_lost_bridge = result.rx_fifo_lost_bridge();
    response->gsusb_ring_drops = result.gsusb_ring_drops();
  });
  if (response->success) {
    response->message +=
      " (0 in a counter means \"the firmware did not report it\", not \"no errors\")";
  }
}

void LitearmDriverNode::handle_log_start(
  const litearm_msgs::srv::LogStart::Request::SharedPtr request,
  litearm_msgs::srv::LogStart::Response::SharedPtr response)
{
  if (request->ticks <= 0) {
    response->success = false;
    response->message = "ticks must be positive: the firmware records that many control "
      "cycles and stops on its own when its buffer is full";
    return;
  }
  run_command(arm_.get(), "log_start", *response, [&request](litearm::Arm & arm) {
    arm.log().start(static_cast<int>(request->ticks));
  });
  if (response->success) {
    response->message += " (recording at 300 Hz; read it back with log_dump)";
  }
}

void LitearmDriverNode::handle_log_stop(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  run_command(arm_.get(), "log_stop", *response, [](litearm::Arm & arm) {
    arm.log().stop();
  });
}

void LitearmDriverNode::handle_log_dump(
  const litearm_msgs::srv::LogDump::Request::SharedPtr request,
  litearm_msgs::srv::LogDump::Response::SharedPtr response)
{
  const GateResult name = check_log_filename(request->filename);
  if (!name.ok) {
    fill_failure(*response, name);
    return;
  }
  const GateResult timeout = check_timeout(request->timeout, "timeout");
  if (!timeout.ok) {
    fill_failure(*response, timeout);
    return;
  }
  const GateResult link = check_connected(connected());
  if (!link.ok) {
    fill_failure(*response, link);
    return;
  }
  try {
    std::filesystem::create_directories(log_dir_);
    const std::filesystem::path path = std::filesystem::path(log_dir_) / request->filename;
    const double timeout_s = request->timeout > 0.0 ? request->timeout : 1.0;
    const std::size_t bytes = arm_->log().dump(path.string(), request->wait, timeout_s, 3,
      request->timeout > 0.0 ? request->timeout : -1.0);
    response->path = path.string();
    response->bytes = static_cast<int32_t>(bytes);
    const int sample_size = litearm::sample_size(arm_->n());
    response->samples = sample_size > 0 ? static_cast<int32_t>(bytes / sample_size) : 0;
    response->success = true;
    response->message = "wrote " + std::to_string(response->bytes) + " bytes (" +
      std::to_string(response->samples) + " samples of " + std::to_string(sample_size) +
      " bytes: u32 tick + q_ref/dq/tau per axis as f32)";
  } catch (const std::exception & error) {
    response->success = false;
    response->message = std::string("log_dump failed: ") + error.what();
  }
}

}  // namespace litearm_driver
