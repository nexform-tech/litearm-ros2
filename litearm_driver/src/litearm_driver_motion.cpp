// litearm_driver_motion.cpp — handlers for the SDK's motion surface.
//
// Joint moves, the firmware's own cartesian planner, inverse kinematics and the raw MIT
// frames. Every one of them can energise the motors, so all of them sit behind the node's
// allow_motion parameter on top of the SDK's own guards; inverse kinematics only solves and
// is therefore available without it.
//
// These are the maintenance driver's motion calls, not a control interface. The driver owns
// the serial port exclusively, so nothing else is streaming commands while one of these
// runs, and none of them sets up a stream: the firmware's 100 ms command watchdog takes the
// arm back to its hold after a single frame.

#include "litearm_driver/litearm_driver_node.hpp"
#include "litearm_driver/service_helpers.hpp"

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "litearm/cart.hpp"
#include "litearm/rot.hpp"

namespace litearm_driver
{

namespace
{

/// Fill a cartesian plan report from what the firmware answered.
template<class Response>
void fill_plan(Response & response, const litearm::CartPlan & plan)
{
  response.settled = plan.settled;
  response.err = plan.err;
  response.waypoints = plan.n_wp;
  response.plan_us = plan.plan_us;
  response.settle_err_rad = plan.settle_err_rad;
  response.q_final = plan.q_final;
}

/// A flat array of six values per waypoint turned into the SDK's pose list.
bool read_poses(
  const std::vector<double> & flat, std::vector<litearm::rot::PoseInput> & poses,
  GateResult & gate)
{
  gate = check_finite_values(flat, "poses");
  if (!gate.ok) {
    return false;
  }
  if (flat.size() % 6 != 0) {
    gate.ok = false;
    gate.message = "poses must hold six values per waypoint; got " +
      std::to_string(flat.size()) + ", which is not a multiple of six";
    return false;
  }
  for (std::size_t i = 0; i < flat.size(); i += 6) {
    poses.emplace_back(std::array<double, 6>{
      flat[i], flat[i + 1], flat[i + 2], flat[i + 3], flat[i + 4], flat[i + 5]});
  }
  return true;
}

}  // namespace

void LitearmDriverNode::handle_move_j(
  const litearm_msgs::srv::MoveJ::Request::SharedPtr request,
  litearm_msgs::srv::MoveJ::Response::SharedPtr response)
{
  if (!require_motion_allowed(*response)) {
    return;
  }
  const GateResult values = check_finite_values(request->q, joint_count(), "q");
  if (!values.ok) {
    fill_failure(*response, values);
    return;
  }
  const GateResult speed = check_speed_fraction(request->speed);
  if (!speed.ok) {
    fill_failure(*response, speed);
    return;
  }
  run_command(arm_.get(), "move_j", *response, [&request](litearm::Arm & arm) {
    arm.movej(request->q, request->speed);
  });
  if (response->success) {
    response->message += " (blocks until the firmware reports the move finished)";
  }
}

void LitearmDriverNode::handle_move_j_sync(
  const litearm_msgs::srv::MoveJSync::Request::SharedPtr request,
  litearm_msgs::srv::MoveJSync::Response::SharedPtr response)
{
  if (!require_motion_allowed(*response)) {
    return;
  }
  const GateResult values = check_finite_values(request->q, joint_count(), "q");
  if (!values.ok) {
    fill_failure(*response, values);
    return;
  }
  const GateResult speed = check_speed_fraction(request->speed);
  if (!speed.ok) {
    fill_failure(*response, speed);
    return;
  }
  run_command(arm_.get(), "move_j_sync", *response, [&request](litearm::Arm & arm) {
    arm.movej_sync(request->q, request->speed);
  });
}

void LitearmDriverNode::handle_move_p(
  const litearm_msgs::srv::MoveP::Request::SharedPtr request,
  litearm_msgs::srv::MoveP::Response::SharedPtr response)
{
  if (!require_motion_allowed(*response)) {
    return;
  }
  const GateResult pose = check_finite_pose(request->pose);
  if (!pose.ok) {
    fill_failure(*response, pose);
    return;
  }
  const GateResult speed = check_speed_fraction(request->speed);
  if (!speed.ok) {
    fill_failure(*response, speed);
    return;
  }
  run_command(arm_.get(), "move_p", *response, [&request](litearm::Arm & arm) {
    arm.move_p(request->pose, request->speed, request->wait);
  });
  if (response->success && !request->wait) {
    response->message += " (wait=false: the plan is in flight; poll_cart reports it)";
  }
}

void LitearmDriverNode::handle_move_js(
  const litearm_msgs::srv::MoveJs::Request::SharedPtr request,
  litearm_msgs::srv::MoveJs::Response::SharedPtr response)
{
  if (!require_motion_allowed(*response)) {
    return;
  }
  const GateResult q = check_finite_values(request->q, joint_count(), "q");
  if (!q.ok) {
    fill_failure(*response, q);
    return;
  }
  const GateResult dq = request->dq.empty()
    ? GateResult{} : check_finite_values(request->dq, joint_count(), "dq");
  if (!dq.ok) {
    fill_failure(*response, dq);
    return;
  }
  const GateResult tau = request->tau_ff.empty()
    ? GateResult{} : check_finite_values(request->tau_ff, joint_count(), "tau_ff");
  if (!tau.ok) {
    fill_failure(*response, tau);
    return;
  }
  run_command(arm_.get(), "move_js", *response, [&request](litearm::Arm & arm) {
    arm.move_js(request->q, request->dq, request->tau_ff);
  });
  if (response->success) {
    response->message +=
      " (one frame only: the firmware's 100 ms watchdog takes the arm back to its hold "
      "unless a caller keeps re-sending, which is what the ros2_control component does)";
    if (!request->tau_ff.empty()) {
      response->message +=
        " (a tau_ff section switches the firmware's built-in feedforward off for this frame)";
    }
  }
}

void LitearmDriverNode::handle_send_mit(
  const litearm_msgs::srv::SendMit::Request::SharedPtr request,
  litearm_msgs::srv::SendMit::Response::SharedPtr response)
{
  if (!require_motion_allowed(*response)) {
    return;
  }
  const GateResult index = check_joint_index(static_cast<int>(request->joint), joint_count());
  if (!index.ok) {
    fill_failure(*response, index);
    return;
  }
  const std::vector<double> values{request->q, request->dq, request->kp, request->kd, request->tau};
  const GateResult finite = check_finite_values(values, "q/dq/kp/kd/tau");
  if (!finite.ok) {
    fill_failure(*response, finite);
    return;
  }
  run_command(arm_.get(), "send_mit", *response, [&request](litearm::Arm & arm) {
    arm.send_mit(static_cast<int>(request->joint), request->q, request->dq, request->kp,
                 request->kd, request->tau);
  });
}

void LitearmDriverNode::handle_send_mit_all(
  const litearm_msgs::srv::SendMitAll::Request::SharedPtr request,
  litearm_msgs::srv::SendMitAll::Response::SharedPtr response)
{
  if (!require_motion_allowed(*response)) {
    return;
  }
  const std::vector<std::pair<const char *, const std::vector<double> *>> arrays{
    {"q", &request->q}, {"dq", &request->dq}, {"kp", &request->kp}, {"kd", &request->kd},
    {"tau", &request->tau}};
  for (const auto & entry : arrays) {
    const GateResult gate = check_finite_values(*entry.second, joint_count(), entry.first);
    if (!gate.ok) {
      fill_failure(*response, gate);
      return;
    }
  }
  run_command(arm_.get(), "send_mit_all", *response, [&request](litearm::Arm & arm) {
    arm.send_mit_all(request->q, request->dq, request->kp, request->kd, request->tau);
  });
  if (response->success) {
    response->message +=
      " (raw gains for one frame: the firmware adds no feedforward of its own on this path)";
  }
}

void LitearmDriverNode::handle_home(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  if (!require_motion_allowed(*response)) {
    return;
  }
  run_command(arm_.get(), "home", *response, [](litearm::Arm & arm) {
    arm.home();
  });
  if (response->success) {
    response->message +=
      " (the firmware walks to the URDF zero pose at its own low speed; support the arm first)";
  }
}

void LitearmDriverNode::handle_move_l(
  const litearm_msgs::srv::MoveL::Request::SharedPtr request,
  litearm_msgs::srv::MoveL::Response::SharedPtr response)
{
  if (!require_motion_allowed(*response)) {
    return;
  }
  const GateResult pose = check_finite_pose(request->pose);
  if (!pose.ok) {
    fill_failure(*response, pose);
    return;
  }
  const GateResult speed = check_speed_fraction(request->speed);
  if (!speed.ok) {
    fill_failure(*response, speed);
    return;
  }
  run_command(arm_.get(), "move_l", *response, [&request, &response](litearm::Arm & arm) {
    fill_plan(*response, arm.move_l(request->pose, request->speed, request->wait));
  });
}

void LitearmDriverNode::handle_move_c(
  const litearm_msgs::srv::MoveC::Request::SharedPtr request,
  litearm_msgs::srv::MoveC::Response::SharedPtr response)
{
  if (!require_motion_allowed(*response)) {
    return;
  }
  for (const auto & entry :
    std::vector<std::pair<const char *, const std::array<double, 6> *>>{
      {"start", &request->start}, {"via", &request->via}, {"end", &request->end}})
  {
    GateResult gate = check_finite_pose(*entry.second);
    if (!gate.ok) {
      gate.message = std::string(entry.first) + ": " + gate.message;
      fill_failure(*response, gate);
      return;
    }
  }
  const GateResult speed = check_speed_fraction(request->speed);
  if (!speed.ok) {
    fill_failure(*response, speed);
    return;
  }
  run_command(arm_.get(), "move_c", *response, [&request, &response](litearm::Arm & arm) {
    fill_plan(*response, arm.move_c(request->start, request->via, request->end, request->speed,
                                    request->wait));
  });
}

void LitearmDriverNode::handle_move_path(
  const litearm_msgs::srv::MovePath::Request::SharedPtr request,
  litearm_msgs::srv::MovePath::Response::SharedPtr response)
{
  if (!require_motion_allowed(*response)) {
    return;
  }
  std::vector<litearm::rot::PoseInput> poses;
  GateResult gate;
  if (!read_poses(request->poses, poses, gate)) {
    fill_failure(*response, gate);
    return;
  }
  const GateResult speed = check_speed_fraction(request->speed);
  if (!speed.ok) {
    fill_failure(*response, speed);
    return;
  }
  run_command(arm_.get(), "move_path", *response,
    [&poses, &request, &response](litearm::Arm & arm) {
      fill_plan(*response, arm.move_path(poses, request->speed, request->wait));
    });
}

void LitearmDriverNode::handle_poll_cart(
  const litearm_msgs::srv::PollCart::Request::SharedPtr request,
  litearm_msgs::srv::PollCart::Response::SharedPtr response)
{
  (void)request;
  run_command(arm_.get(), "poll_cart", *response, [&response](litearm::Arm & arm) {
    const auto plan = arm.poll_cart();
    if (!plan.has_value()) {
      response->pending = true;
      response->message = "no cartesian plan has been reported yet";
      return;
    }
    response->pending = false;
    fill_plan(*response, *plan);
    response->message = plan->ok ? "cartesian plan finished" : "cartesian plan was refused";
  });
}

void LitearmDriverNode::handle_inverse_kinematics(
  const litearm_msgs::srv::InverseKinematics::Request::SharedPtr request,
  litearm_msgs::srv::InverseKinematics::Response::SharedPtr response)
{
  // No allow_motion gate: this one only solves.
  const GateResult pose = check_finite_pose(request->pose);
  if (!pose.ok) {
    fill_failure(*response, pose);
    return;
  }
  if (!request->seed.empty()) {
    const GateResult seed = check_finite_values(request->seed, 7u, "seed");
    if (!seed.ok) {
      fill_failure(*response, seed);
      return;
    }
  }
  const GateResult timeout = check_timeout(request->timeout, "timeout");
  if (!timeout.ok) {
    fill_failure(*response, timeout);
    return;
  }
  run_command(arm_.get(), "inverse_kinematics", *response,
    [&request, &response](litearm::Arm & arm) {
      response->q = arm.ik(request->pose, request->seed,
        request->timeout > 0.0 ? request->timeout : 3.0);
    });
}

}  // namespace litearm_driver
