// Tests for LitearmDriverNode: the whole lifecycle and every service handler, driven
// offline against the SDK's FakeTransport.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "litearm/arm.hpp"
#include "litearm/protocol.hpp"
#include "litearm/state.hpp"
#include "litearm/testing.hpp"
#include "litearm/transport.hpp"

#include "litearm_driver/litearm_driver_node.hpp"

namespace litearm_driver
{
namespace
{

using std_srvs::srv::SetBool;
using std_srvs::srv::Trigger;

/// Delegates to a shared FakeTransport so the test keeps a handle on the fake while the
/// Arm owns the Transport handed to it.
class SharedFake : public litearm::Transport
{
public:
  explicit SharedFake(std::shared_ptr<litearm::testing::FakeTransport> fake)
  : fake_(std::move(fake)) {}

  void write_frame(uint8_t cmd, const uint8_t * payload, size_t len) override
  {
    fake_->write_frame(cmd, payload, len);
  }
  std::optional<litearm::proto::Frame> read_frame(double timeout) override
  {
    return fake_->read_frame(timeout);
  }
  void close() override { fake_->close(); }
  bool is_open() const override { return fake_->is_open(); }
  std::string text_log() const override { return fake_->text_log(); }
  std::string port_name() const override { return fake_->port_name(); }
  uint64_t flush_failures() const override { return fake_->flush_failures(); }

private:
  std::shared_ptr<litearm::testing::FakeTransport> fake_;
};

/// Driver that builds its Arm against a FakeTransport instead of a serial port, and
/// exposes the protected handlers so a test can call them without a client or a spin.
class TestDriver : public LitearmDriverNode
{
public:
  TestDriver() = default;

  using LitearmDriverNode::build_status;
  using LitearmDriverNode::connected;
  using LitearmDriverNode::motors_enabled;

  using LitearmDriverNode::handle_clear_faults;
  using LitearmDriverNode::handle_disable;
  using LitearmDriverNode::handle_emergency_stop;
  using LitearmDriverNode::handle_enable;
  using LitearmDriverNode::handle_enter_dfu;
  using LitearmDriverNode::handle_get_feedforward_scalar;
  using LitearmDriverNode::handle_get_joint_params;
  using LitearmDriverNode::handle_get_status;
  using LitearmDriverNode::handle_park;
  using LitearmDriverNode::handle_reset;
  using LitearmDriverNode::handle_reset_factory_params;
  using LitearmDriverNode::handle_save_params;
  using LitearmDriverNode::handle_set_feedforward_mask;
  using LitearmDriverNode::handle_set_feedforward_preset;
  using LitearmDriverNode::handle_set_feedforward_scalar;
  using LitearmDriverNode::handle_set_feedforward_vector;
  using LitearmDriverNode::handle_set_joint_gains;
  using LitearmDriverNode::handle_set_joint_limits;
  using LitearmDriverNode::handle_set_motion_mode;
  using LitearmDriverNode::handle_set_payload;
  using LitearmDriverNode::handle_set_speed_scaling;
  using LitearmDriverNode::handle_zero_g;
  using LitearmDriverNode::handle_get_tcp;
  using LitearmDriverNode::handle_get_diagnostics;
  using LitearmDriverNode::handle_kin_bench;
  using LitearmDriverNode::handle_move_j;
  using LitearmDriverNode::handle_move_j_sync;
  using LitearmDriverNode::handle_move_p;
  using LitearmDriverNode::handle_move_js;
  using LitearmDriverNode::handle_send_mit;
  using LitearmDriverNode::handle_send_mit_all;
  using LitearmDriverNode::handle_move_l;
  using LitearmDriverNode::handle_move_c;
  using LitearmDriverNode::handle_move_path;
  using LitearmDriverNode::handle_poll_cart;
  using LitearmDriverNode::handle_inverse_kinematics;
  using LitearmDriverNode::handle_get_feedforward_vector;
  using LitearmDriverNode::handle_get_feedforward_mask;
  using LitearmDriverNode::handle_get_feedforward_catalog;
  using LitearmDriverNode::handle_set_gravity_scale;
  using LitearmDriverNode::handle_set_inertia_scale;
  using LitearmDriverNode::handle_set_gravity_vector;
  using LitearmDriverNode::handle_probe_model;
  using LitearmDriverNode::handle_get_model_body;
  using LitearmDriverNode::handle_set_model_body;
  using LitearmDriverNode::handle_get_model_jm;
  using LitearmDriverNode::handle_set_model_jm;
  using LitearmDriverNode::handle_commit_model;
  using LitearmDriverNode::handle_get_model_status;
  using LitearmDriverNode::handle_log_start;
  using LitearmDriverNode::handle_log_dump;
  using LitearmDriverNode::handle_reconnect;
  using LitearmDriverNode::handle_home;
  using LitearmDriverNode::handle_revert_model;
  using LitearmDriverNode::handle_log_stop;

  std::shared_ptr<litearm::testing::FakeTransport> fake() const { return fake_; }

protected:
  std::unique_ptr<litearm::Arm> create_arm(const litearm::ArmOptions & options) override
  {
    litearm::ArmOptions opts = options;
    opts.port = std::string("fake");
    auto shared = std::make_shared<litearm::testing::FakeTransport>(
      "fake", 0.2, "Litearm1.8.0-7J", 7);
    shared->auto_status = true;
    shared->set_q({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    fake_ = shared;
    opts.transport_factory =
      [shared](const std::string &) -> std::unique_ptr<litearm::Transport> {
        return std::make_unique<SharedFake>(shared);
      };
    return std::make_unique<litearm::Arm>(opts);
  }

private:
  std::shared_ptr<litearm::testing::FakeTransport> fake_;
};

class LitearmDriverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      // File logging needs a writable ~/.ros/log; tests read stdout instead.
      rclcpp::InitOptions options;
      options.auto_initialize_logging(false);
      rclcpp::init(0, nullptr, options);
    }
    driver_ = std::make_shared<TestDriver>();
  }

  void TearDown() override {driver_.reset();}

  /// Configure and activate; returns false when either transition fails, so the caller can
  /// ASSERT on it (an ASSERT_* inside this helper would only return from the helper).
  bool prepare()
  {
    const auto configured = driver_->on_configure(rclcpp_lifecycle::State());
    if (configured != LitearmDriverNode::CallbackReturn::SUCCESS) {
      return false;
    }
    return driver_->on_activate(rclcpp_lifecycle::State()) ==
           LitearmDriverNode::CallbackReturn::SUCCESS;
  }

  /// Call one of the handlers that uses std_srvs::srv::Trigger.
  std::shared_ptr<Trigger::Response> trigger(
    void (LitearmDriverNode::*handler)(
      const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr))
  {
    auto response = std::make_shared<Trigger::Response>();
    ((*driver_).*handler)(std::make_shared<Trigger::Request>(), response);
    return response;
  }

  /// Build a response, let the lambda fill it, and return it.
  template <class Response, class Call>
  std::shared_ptr<Response> invoke(Call && call)
  {
    auto response = std::make_shared<Response>();
    call(response);
    return response;
  }

  /// Poll a predicate until it holds or the timeout expires.
  bool wait_for(const std::function<bool()> & predicate, int timeout_ms = 2000)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return predicate();
  }

  std::shared_ptr<TestDriver> driver_;
};

// ────────────────────────────────────────────────────────────────────────────────
// lifecycle
// ────────────────────────────────────────────────────────────────────────────────

TEST_F(LitearmDriverTest, ConfigureConnectsAndDerivesJointNames)
{
  ASSERT_TRUE(prepare());
  const auto status = driver_->build_status(nullptr);
  EXPECT_TRUE(status.connected);
  EXPECT_EQ(status.firmware, "Litearm1.8.0-7J");
  ASSERT_EQ(status.joint_names.size(), 7u);
  EXPECT_EQ(status.joint_names.front(), "joint1");
  EXPECT_EQ(status.joint_names.back(), "joint7");
  EXPECT_TRUE(status.license_valid);
}

TEST_F(LitearmDriverTest, ConfigureRejectsAJointNameListOfTheWrongLength)
{
  driver_->set_parameter(
    rclcpp::Parameter("joint_names", std::vector<std::string>{"joint1", "joint2"}));
  EXPECT_EQ(
    driver_->on_configure(rclcpp_lifecycle::State()),
    LitearmDriverNode::CallbackReturn::ERROR);
}

TEST_F(LitearmDriverTest, ActivationAdvertisesTheWholeCommandSet)
{
  ASSERT_TRUE(prepare());
  const auto services = driver_->get_service_names_and_types();
  const std::vector<std::string> expected{
    "/enable", "/disable", "/reset", "/clear_faults", "/emergency_stop", "/park",
    "/save_params", "/reset_factory_params", "/enter_dfu", "/zero_g",
    "/get_status",
    "/set_speed_scaling", "/set_motion_mode", "/set_payload",
    "/set_feedforward_mask", "/set_feedforward_preset", "/set_feedforward_scalar",
    "/set_feedforward_vector", "/get_feedforward_scalar",
    "/get_joint_params", "/set_joint_gains", "/set_joint_limits",
    "/reconnect",
    "/home",
    "/get_tcp",
    "/get_diagnostics",
    "/kin_bench",
    "/move_j",
    "/move_j_sync",
    "/move_p",
    "/move_js",
    "/send_mit",
    "/send_mit_all",
    "/move_l",
    "/move_c",
    "/move_path",
    "/poll_cart",
    "/inverse_kinematics",
    "/get_feedforward_vector",
    "/get_feedforward_mask",
    "/get_feedforward_catalog",
    "/set_gravity_scale",
    "/set_inertia_scale",
    "/set_gravity_vector",
    "/probe_model",
    "/get_model_body",
    "/set_model_body",
    "/get_model_jm",
    "/set_model_jm",
    "/commit_model",
    "/revert_model",
    "/get_model_status",
    "/log_start",
    "/log_stop",
    "/log_dump"};
  for (const std::string & name : expected) {
    EXPECT_NE(services.find(name), services.end()) << name;
  }
  // The graph also carries the lifecycle node's own state-transition services and entries
  // left by earlier tests, so the command set is asserted by membership, not by count.
}

TEST_F(LitearmDriverTest, DeactivationRemovesEveryCommandService)
{
  ASSERT_TRUE(prepare());
  ASSERT_FALSE(driver_->get_service_names_and_types().empty());
  EXPECT_EQ(
    driver_->on_deactivate(rclcpp_lifecycle::State()),
    LitearmDriverNode::CallbackReturn::SUCCESS);

  const auto services = driver_->get_service_names_and_types();
  // The node's own lifecycle services stay; the command set must be gone.
  for (const char * name : {"/enable", "/clear_faults", "/get_status", "/set_speed_scaling"}) {
    EXPECT_EQ(services.find(name), services.end()) << name;
  }
}

// ────────────────────────────────────────────────────────────────────────────────
// state and safety
// ────────────────────────────────────────────────────────────────────────────────

TEST_F(LitearmDriverTest, EnableAndDisableReachTheFirmware)
{
  ASSERT_TRUE(prepare());
  ASSERT_TRUE(wait_for([this]() {return driver_->motors_enabled();}));

  const auto disabled = trigger(&TestDriver::handle_disable);
  ASSERT_TRUE(disabled->success) << disabled->message;
  EXPECT_TRUE(wait_for([this]() {return !driver_->motors_enabled();}));

  const auto enabled = trigger(&TestDriver::handle_enable);
  ASSERT_TRUE(enabled->success) << enabled->message;
  EXPECT_TRUE(wait_for([this]() {return driver_->motors_enabled();}));
}

TEST_F(LitearmDriverTest, RefusedArgumentsSendNoFrame)
{
  ASSERT_TRUE(prepare());
  const auto before = driver_->fake()->tx_count();

  auto too_fast = std::make_shared<litearm_msgs::srv::SetSpeedScaling::Request>();
  too_fast->percent = 150;
  const auto speed = invoke<litearm_msgs::srv::SetSpeedScaling::Response>(
    [&](auto response) {driver_->handle_set_speed_scaling(too_fast, response);});
  EXPECT_FALSE(speed->success);

  auto too_heavy = std::make_shared<litearm_msgs::srv::SetPayload::Request>();
  too_heavy->mass = 25.0;
  const auto payload = invoke<litearm_msgs::srv::SetPayload::Response>(
    [&](auto response) {driver_->handle_set_payload(too_heavy, response);});
  EXPECT_FALSE(payload->success);

  auto bad_preset = std::make_shared<litearm_msgs::srv::SetFeedforwardPreset::Request>();
  bad_preset->preset = 7;
  const auto preset = invoke<litearm_msgs::srv::SetFeedforwardPreset::Response>(
    [&](auto response) {driver_->handle_set_feedforward_preset(bad_preset, response);});
  EXPECT_FALSE(preset->success);

  auto bad_vector = std::make_shared<litearm_msgs::srv::SetFeedforwardVector::Request>();
  bad_vector->item = 4;
  bad_vector->values = {std::nan("")};
  const auto vector = invoke<litearm_msgs::srv::SetFeedforwardVector::Response>(
    [&](auto response) {driver_->handle_set_feedforward_vector(bad_vector, response);});
  EXPECT_FALSE(vector->success);

  // The one remaining opt-in service is off by default.
  EXPECT_FALSE(trigger(&TestDriver::handle_enter_dfu)->success);

  // A gate that refuses has to refuse before the frame, not after the firmware rejects it.
  EXPECT_EQ(driver_->fake()->tx_count(), before);
}

TEST_F(LitearmDriverTest, SaveParamsRequiresTheMotorsToBeDisabled)
{
  ASSERT_TRUE(prepare());
  ASSERT_TRUE(wait_for([this]() {return driver_->motors_enabled();}));

  const auto refused = trigger(&TestDriver::handle_save_params);
  EXPECT_FALSE(refused->success);
  EXPECT_NE(refused->message.find("disable"), std::string::npos);

  ASSERT_TRUE(trigger(&TestDriver::handle_disable)->success);
  ASSERT_TRUE(wait_for([this]() {return !driver_->motors_enabled();}));
  const auto saved = trigger(&TestDriver::handle_save_params);
  EXPECT_TRUE(saved->success) << saved->message;
}

TEST_F(LitearmDriverTest, EmergencyStopAndParkReachTheFirmware)
{
  ASSERT_TRUE(prepare());
  const auto stopped = trigger(&TestDriver::handle_emergency_stop);
  EXPECT_TRUE(stopped->success) << stopped->message;
  EXPECT_NE(stopped->message.find("hardware"), std::string::npos);

  const auto parked = trigger(&TestDriver::handle_park);
  EXPECT_TRUE(parked->success) << parked->message;
}

// ────────────────────────────────────────────────────────────────────────────────
// motion configuration
// ────────────────────────────────────────────────────────────────────────────────

TEST_F(LitearmDriverTest, SpeedScalingIsSentAndTracked)
{
  ASSERT_TRUE(prepare());
  auto request = std::make_shared<litearm_msgs::srv::SetSpeedScaling::Request>();
  request->percent = 40;
  const auto response = invoke<litearm_msgs::srv::SetSpeedScaling::Response>(
    [&](auto out) {driver_->handle_set_speed_scaling(request, out);});
  ASSERT_TRUE(response->success) << response->message;

  bool seen = false;
  for (const auto & entry : driver_->fake()->tx_snapshot()) {
    if (entry.first == litearm::proto::CMD_SET_SPEED_PERCENT) {
      seen = entry.second.size() == 1 && entry.second[0] == 40;
    }
  }
  EXPECT_TRUE(seen);
  EXPECT_EQ(driver_->build_status(nullptr).speed_scaling, 40);
}

TEST_F(LitearmDriverTest, MotionModeIsForwardedAndExplained)
{
  ASSERT_TRUE(prepare());
  auto request = std::make_shared<litearm_msgs::srv::SetMotionMode::Request>();
  request->mode = 0;
  const auto response = invoke<litearm_msgs::srv::SetMotionMode::Response>(
    [&](auto out) {driver_->handle_set_motion_mode(request, out);});
  ASSERT_TRUE(response->success) << response->message;
  EXPECT_NE(response->message.find("mode 0"), std::string::npos);
}

// ────────────────────────────────────────────────────────────────────────────────
// feedforward and joint parameters
// ────────────────────────────────────────────────────────────────────────────────

TEST_F(LitearmDriverTest, FeedforwardScalarRoundTripsThroughTheFirmware)
{
  ASSERT_TRUE(prepare());
  auto write = std::make_shared<litearm_msgs::srv::SetFeedforwardScalar::Request>();
  write->item = 4;
  write->sub = 0;
  write->value = 1.25;
  const auto written = invoke<litearm_msgs::srv::SetFeedforwardScalar::Response>(
    [&](auto out) {driver_->handle_set_feedforward_scalar(write, out);});
  ASSERT_TRUE(written->success) << written->message;

  auto read = std::make_shared<litearm_msgs::srv::GetFeedforwardScalar::Request>();
  read->item = 4;
  read->sub = 0;
  const auto value = invoke<litearm_msgs::srv::GetFeedforwardScalar::Response>(
    [&](auto out) {driver_->handle_get_feedforward_scalar(read, out);});
  ASSERT_TRUE(value->success) << value->message;
  EXPECT_DOUBLE_EQ(value->value, 1.25);
}

TEST_F(LitearmDriverTest, FeedforwardMaskIsWritten)
{
  ASSERT_TRUE(prepare());
  auto request = std::make_shared<litearm_msgs::srv::SetFeedforwardMask::Request>();
  request->mask = 0x1FF;
  const auto response = invoke<litearm_msgs::srv::SetFeedforwardMask::Response>(
    [&](auto out) {driver_->handle_set_feedforward_mask(request, out);});
  EXPECT_TRUE(response->success) << response->message;
}

TEST_F(LitearmDriverTest, JointParamsReadAndWrite)
{
  ASSERT_TRUE(prepare());

  auto all = std::make_shared<litearm_msgs::srv::GetJointParams::Request>();
  all->joint = -1;
  const auto table = invoke<litearm_msgs::srv::GetJointParams::Response>(
    [&](auto out) {driver_->handle_get_joint_params(all, out);});
  ASSERT_TRUE(table->success) << table->message;
  EXPECT_EQ(table->params.size(), 7u);

  auto gains = std::make_shared<litearm_msgs::srv::SetJointGains::Request>();
  gains->joint = 0;
  gains->kp = 12.0;
  gains->kd = 1.5;
  gains->tau_max = 9.0;
  EXPECT_TRUE(invoke<litearm_msgs::srv::SetJointGains::Response>(
    [&](auto out) {driver_->handle_set_joint_gains(gains, out);})->success);

  auto limits = std::make_shared<litearm_msgs::srv::SetJointLimits::Request>();
  limits->joint = 0;
  limits->q_min = -1.0;
  limits->q_max = 1.0;
  EXPECT_TRUE(invoke<litearm_msgs::srv::SetJointLimits::Response>(
    [&](auto out) {driver_->handle_set_joint_limits(limits, out);})->success);

  auto one = std::make_shared<litearm_msgs::srv::GetJointParams::Request>();
  one->joint = 0;
  const auto single = invoke<litearm_msgs::srv::GetJointParams::Response>(
    [&](auto out) {driver_->handle_get_joint_params(one, out);});
  ASSERT_TRUE(single->success) << single->message;
  ASSERT_EQ(single->params.size(), 1u);
  EXPECT_DOUBLE_EQ(single->params[0].kp, 12.0);
  EXPECT_DOUBLE_EQ(single->params[0].q_min, -1.0);
}

TEST_F(LitearmDriverTest, JointIndexOutOfRangeSendsNoFrame)
{
  ASSERT_TRUE(prepare());
  const auto before = driver_->fake()->tx_count();

  auto request = std::make_shared<litearm_msgs::srv::SetJointGains::Request>();
  request->joint = 7;  // the fake reports seven axes: 0..6
  request->kp = 1.0;
  request->kd = 1.0;
  request->tau_max = 1.0;
  const auto response = invoke<litearm_msgs::srv::SetJointGains::Response>(
    [&](auto out) {driver_->handle_set_joint_gains(request, out);});
  EXPECT_FALSE(response->success);
  EXPECT_EQ(driver_->fake()->tx_count(), before);
}

// ────────────────────────────────────────────────────────────────────────────────
// zero gravity
// ────────────────────────────────────────────────────────────────────────────────

TEST_F(LitearmDriverTest, ZeroGravityStartsAndStops)
{
  ASSERT_TRUE(prepare());

  auto start = std::make_shared<SetBool::Request>();
  start->data = true;
  const auto entered = invoke<SetBool::Response>(
    [&](auto out) {driver_->handle_zero_g(start, out);});
  ASSERT_TRUE(entered->success) << entered->message;
  EXPECT_TRUE(driver_->build_status(nullptr).zero_g_active);
  EXPECT_GT(driver_->fake()->zg_writes.load(), 0);

  auto stop = std::make_shared<SetBool::Request>();
  stop->data = false;
  const auto left = invoke<SetBool::Response>(
    [&](auto out) {driver_->handle_zero_g(stop, out);});
  ASSERT_TRUE(left->success) << left->message;
  EXPECT_FALSE(driver_->build_status(nullptr).zero_g_active);
}

TEST_F(LitearmDriverTest, ZeroGravityPeriodOutsideTheSdkContractIsRefused)
{
  ASSERT_TRUE(prepare());
  driver_->set_parameter(rclcpp::Parameter("zero_g_keepalive_period_s", 0.2));
  const auto before = driver_->fake()->tx_count();

  auto start = std::make_shared<SetBool::Request>();
  start->data = true;
  const auto response = invoke<SetBool::Response>(
    [&](auto out) {driver_->handle_zero_g(start, out);});
  EXPECT_FALSE(response->success);
  EXPECT_EQ(driver_->fake()->tx_count(), before);
}

// ────────────────────────────────────────────────────────────────────────────────
// status
// ────────────────────────────────────────────────────────────────────────────────

TEST_F(LitearmDriverTest, GetStatusReturnsAFreshFrame)
{
  ASSERT_TRUE(prepare());
  auto request = std::make_shared<litearm_msgs::srv::GetStatus::Request>();
  request->timeout = 0.5;
  const auto response = invoke<litearm_msgs::srv::GetStatus::Response>(
    [&](auto out) {driver_->handle_get_status(request, out);});
  ASSERT_TRUE(response->success) << response->message;
  EXPECT_TRUE(response->status.connected);
  EXPECT_EQ(response->status.joint_names.size(), 7u);
}

TEST_F(LitearmDriverTest, BuildStatusMapsFaultsAndDiagnostics)
{
  ASSERT_TRUE(prepare());

  litearm::RobotState state;
  state.mode = 6;
  state.mode_name = "EMERGENCY";
  state.joints.resize(3);  // a short frame: axes 3..6 have no values
  state.joint_fault = 0b10;  // axis 1
  state.joints[1].t_mos = 41.5;
  state.joints[1].t_coil = 52.0;
  state.joints[1].err = 3;

  const auto status = driver_->build_status(&state);
  EXPECT_TRUE(status.faulted);
  EXPECT_TRUE(status.drop_hold_inferred);
  EXPECT_EQ(status.joint_fault, 0b10);
  EXPECT_EQ(status.joint_error_code[1], 3);
  EXPECT_DOUBLE_EQ(status.temperature_mos[1], 41.5);
  EXPECT_DOUBLE_EQ(status.temperature_coil[1], 52.0);
  // An axis the frame does not cover reads NaN, not zero: zero is a temperature.
  EXPECT_TRUE(std::isnan(status.temperature_mos[6]));
}

TEST_F(LitearmDriverTest, StatusIsPublishedOnTheRelativeTopic)
{
  ASSERT_TRUE(prepare());

  auto listener = std::make_shared<rclcpp::Node>("status_listener");
  std::atomic<int> received{0};
  auto subscription = listener->create_subscription<litearm_msgs::msg::LitearmStatus>(
    "status", rclcpp::QoS(10).reliable(),
    [&received](litearm_msgs::msg::LitearmStatus::SharedPtr message) {
      if (message->connected && message->joint_names.size() == 7) {
        ++received;
      }
    });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(driver_->get_node_base_interface());
  executor.add_node(listener);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (received.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds(50));
  }
  EXPECT_GT(received.load(), 0);
}

TEST_F(LitearmDriverTest, MotionServicesAreRefusedWithoutAllowMotion)
{
  ASSERT_TRUE(prepare());
  auto request = std::make_shared<litearm_msgs::srv::MoveJ::Request>();
  request->q = std::vector<double>(7, 0.1);
  request->speed = 0.2;

  const auto response = invoke<litearm_msgs::srv::MoveJ::Response>(
    [this, &request](const std::shared_ptr<litearm_msgs::srv::MoveJ::Response> & out) {
      driver_->handle_move_j(request, out);
    });
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("allow_motion"), std::string::npos) << response->message;

  const auto homed = trigger(&TestDriver::handle_home);
  EXPECT_FALSE(homed->success);
  EXPECT_NE(homed->message.find("allow_motion"), std::string::npos) << homed->message;
}

TEST_F(LitearmDriverTest, AllowMotionUnlocksThemAndTheSpeedGateStillBites)
{
  ASSERT_TRUE(prepare());
  driver_->set_parameter(rclcpp::Parameter("allow_motion", true));

  auto percent = std::make_shared<litearm_msgs::srv::MoveJ::Request>();
  percent->q = std::vector<double>(7, 0.1);
  percent->speed = 30.0;   // 30x, not 30 percent
  const auto refused = invoke<litearm_msgs::srv::MoveJ::Response>(
    [this, &percent](const std::shared_ptr<litearm_msgs::srv::MoveJ::Response> & out) {
      driver_->handle_move_j(percent, out);
    });
  EXPECT_FALSE(refused->success);
  EXPECT_NE(refused->message.find("fraction"), std::string::npos) << refused->message;

  auto wrong_length = std::make_shared<litearm_msgs::srv::MoveJ::Request>();
  wrong_length->q = std::vector<double>(6, 0.1);
  wrong_length->speed = 0.2;
  const auto short_q = invoke<litearm_msgs::srv::MoveJ::Response>(
    [this, &wrong_length](const std::shared_ptr<litearm_msgs::srv::MoveJ::Response> & out) {
      driver_->handle_move_j(wrong_length, out);
    });
  EXPECT_FALSE(short_q->success);
  EXPECT_NE(short_q->message.find("axes"), std::string::npos) << short_q->message;
}

TEST_F(LitearmDriverTest, InverseKinematicsIsNotBehindAllowMotion)
{
  ASSERT_TRUE(prepare());
  auto request = std::make_shared<litearm_msgs::srv::InverseKinematics::Request>();
  request->pose = std::array<double, 6>{0.3, 0.0, 0.4, 3.14, 0.0, 0.0};
  request->timeout = 0.5;

  const auto response = invoke<litearm_msgs::srv::InverseKinematics::Response>(
    [this, &request](const std::shared_ptr<litearm_msgs::srv::InverseKinematics::Response> & out) {
      driver_->handle_inverse_kinematics(request, out);
    });
  // Whether the fake firmware solves it or not, the refusal must not be the motion switch:
  // this service only computes.
  EXPECT_EQ(response->message.find("allow_motion"), std::string::npos) << response->message;
}

TEST_F(LitearmDriverTest, FeedforwardCatalogNeedsNoSession)
{
  // Never configured: the tables are compile-time constants, so the service still answers.
  const auto response = invoke<litearm_msgs::srv::GetFeedforwardCatalog::Response>(
    [this](const std::shared_ptr<litearm_msgs::srv::GetFeedforwardCatalog::Response> & out) {
      driver_->handle_get_feedforward_catalog(
        std::make_shared<litearm_msgs::srv::GetFeedforwardCatalog::Request>(), out);
    });
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->vector_items.empty());
  EXPECT_FALSE(response->scalar_items.empty());
  EXPECT_NE(response->vector_items.front().find(": "), std::string::npos);
}

TEST_F(LitearmDriverTest, DiagnosticsRefuseWhenThereIsNoSession)
{
  const auto response = invoke<litearm_msgs::srv::GetDiagnostics::Response>(
    [this](const std::shared_ptr<litearm_msgs::srv::GetDiagnostics::Response> & out) {
      driver_->handle_get_diagnostics(
        std::make_shared<litearm_msgs::srv::GetDiagnostics::Request>(), out);
    });
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->message.empty());
}

TEST_F(LitearmDriverTest, LogDumpRefusesAPath)
{
  auto request = std::make_shared<litearm_msgs::srv::LogDump::Request>();
  request->filename = "../../etc/passwd";
  const auto response = invoke<litearm_msgs::srv::LogDump::Response>(
    [this, &request](const std::shared_ptr<litearm_msgs::srv::LogDump::Response> & out) {
      driver_->handle_log_dump(request, out);
    });
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("plain file name"), std::string::npos) << response->message;
}

}  // namespace
}  // namespace litearm_driver
