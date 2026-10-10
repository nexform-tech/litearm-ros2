// litearm_driver_model.cpp — handlers for the feedforward reads and the dynamics model store.
//
// Two groups that both write into the firmware's parameter space: the feedforward/gravity
// knobs, and the model store the firmware can import over. Writes here are staged in RAM
// unless the caller asks for a commit, which is the one call in this file that touches flash
// and therefore requires the motors to be disabled.

#include "litearm_driver/litearm_driver_node.hpp"
#include "litearm_driver/service_helpers.hpp"

#include <string>
#include <vector>

#include "litearm/model.hpp"

namespace litearm_driver
{

namespace
{

/// "item: name" for the SDK's static feedforward tables.
std::vector<std::string> format_items(
  const std::vector<std::pair<int, std::string>> & items)
{
  std::vector<std::string> out;
  out.reserve(items.size());
  for (const auto & entry : items) {
    out.push_back(std::to_string(entry.first) + ": " + entry.second);
  }
  return out;
}

}  // namespace

void LitearmDriverNode::handle_get_feedforward_vector(
  const litearm_msgs::srv::GetFeedforwardVector::Request::SharedPtr request,
  litearm_msgs::srv::GetFeedforwardVector::Response::SharedPtr response)
{
  run_command(arm_.get(), "get_feedforward_vector", *response,
    [&request, &response](litearm::Arm & arm) {
      response->values = arm.get_ff_vec(static_cast<int>(request->item)).value;
    });
}

void LitearmDriverNode::handle_get_feedforward_mask(
  const litearm_msgs::srv::GetFeedforwardMask::Request::SharedPtr request,
  litearm_msgs::srv::GetFeedforwardMask::Response::SharedPtr response)
{
  (void)request;
  run_command(arm_.get(), "get_feedforward_mask", *response, [&response](litearm::Arm & arm) {
    response->mask = static_cast<uint32_t>(arm.get_ff_mask());
  });
}

void LitearmDriverNode::handle_get_feedforward_catalog(
  const litearm_msgs::srv::GetFeedforwardCatalog::Request::SharedPtr request,
  litearm_msgs::srv::GetFeedforwardCatalog::Response::SharedPtr response)
{
  (void)request;
  // The tables are compile-time constants, so this one works without a link.
  response->vector_items = format_items(litearm::Arm::ff_vec_items());
  response->scalar_items = format_items(litearm::Arm::ff_scalar_items());
  response->scalar_read_only_items = format_items(litearm::Arm::ff_scalar_ro_items());
  response->success = true;
  response->message = "static SDK tables, not a firmware read: " +
    std::to_string(response->vector_items.size()) + " vector items, " +
    std::to_string(response->scalar_items.size()) + " scalar items, " +
    std::to_string(response->scalar_read_only_items.size()) + " read-only scalars";
}

void LitearmDriverNode::handle_set_gravity_scale(
  const litearm_msgs::srv::SetGravityScale::Request::SharedPtr request,
  litearm_msgs::srv::SetGravityScale::Response::SharedPtr response)
{
  const GateResult gate = check_finite_values(request->values, 7u, "values");
  if (!gate.ok) {
    fill_failure(*response, gate);
    return;
  }
  run_command(arm_.get(), "set_gravity_scale", *response, [&request](litearm::Arm & arm) {
    arm.set_gravity_scale(request->values);
  });
}

void LitearmDriverNode::handle_set_inertia_scale(
  const litearm_msgs::srv::SetInertiaScale::Request::SharedPtr request,
  litearm_msgs::srv::SetInertiaScale::Response::SharedPtr response)
{
  const GateResult gate = check_finite_values(request->values, 7u, "values");
  if (!gate.ok) {
    fill_failure(*response, gate);
    return;
  }
  run_command(arm_.get(), "set_inertia_scale", *response, [&request](litearm::Arm & arm) {
    arm.set_inertia_scale(request->values);
  });
  if (response->success) {
    response->message +=
      " (the default MOVE_JS path has no acceleration source, so this has no effect there)";
  }
}

void LitearmDriverNode::handle_set_gravity_vector(
  const litearm_msgs::srv::SetGravityVector::Request::SharedPtr request,
  litearm_msgs::srv::SetGravityVector::Response::SharedPtr response)
{
  const GateResult gate = check_finite_vector3(request->g);
  if (!gate.ok) {
    fill_failure(*response, gate);
    return;
  }
  const std::array<double, 3> g{request->g[0], request->g[1], request->g[2]};
  run_command(arm_.get(), "set_gravity_vector", *response, [&g](litearm::Arm & arm) {
    arm.set_gravity_vector(g);
  });
}

void LitearmDriverNode::handle_probe_model(
  const litearm_msgs::srv::ProbeModel::Request::SharedPtr request,
  litearm_msgs::srv::ProbeModel::Response::SharedPtr response)
{
  (void)request;
  run_command(arm_.get(), "probe_model", *response, [&response](litearm::Arm & arm) {
    response->supported = arm.model().probe();
    response->message = response->supported
      ? "the firmware answers the model store commands"
      : "the firmware has no model store: it refuses or ignores those commands";
  });
}

void LitearmDriverNode::handle_get_model_body(
  const litearm_msgs::srv::GetModelBody::Request::SharedPtr request,
  litearm_msgs::srv::GetModelBody::Response::SharedPtr response)
{
  run_command(arm_.get(), "get_model_body", *response,
    [&request, &response](litearm::Arm & arm) {
      response->values = arm.model().get_body(static_cast<int>(request->body)).value;
    });
}

void LitearmDriverNode::handle_set_model_body(
  const litearm_msgs::srv::SetModelBody::Request::SharedPtr request,
  litearm_msgs::srv::SetModelBody::Response::SharedPtr response)
{
  const GateResult gate = check_finite_values(request->values, "values");
  if (!gate.ok) {
    fill_failure(*response, gate);
    return;
  }
  run_command(arm_.get(), "set_model_body", *response, [&request](litearm::Arm & arm) {
    arm.model().set_body(static_cast<int>(request->body), request->values);
  });
  if (response->success) {
    response->message +=
      " (staged in RAM: commit_model stores it, revert_model drops it)";
  }
}

void LitearmDriverNode::handle_get_model_jm(
  const litearm_msgs::srv::GetModelJm::Request::SharedPtr request,
  litearm_msgs::srv::GetModelJm::Response::SharedPtr response)
{
  (void)request;
  run_command(arm_.get(), "get_model_jm", *response, [&response](litearm::Arm & arm) {
    response->values = arm.model().get_jm().value;
  });
}

void LitearmDriverNode::handle_set_model_jm(
  const litearm_msgs::srv::SetModelJm::Request::SharedPtr request,
  litearm_msgs::srv::SetModelJm::Response::SharedPtr response)
{
  const GateResult gate = check_finite_values(request->values, "values");
  if (!gate.ok) {
    fill_failure(*response, gate);
    return;
  }
  run_command(arm_.get(), "set_model_jm", *response, [&request](litearm::Arm & arm) {
    arm.model().set_jm(request->values);
  });
  if (response->success) {
    response->message += " (staged in RAM: commit_model stores it)";
  }
}

void LitearmDriverNode::handle_commit_model(
  const litearm_msgs::srv::CommitModel::Request::SharedPtr request,
  litearm_msgs::srv::CommitModel::Response::SharedPtr response)
{
  const GateResult disabled =
    check_requires_disabled(!motors_enabled(), "commit_model");
  if (!disabled.ok) {
    fill_failure(*response, disabled);
    return;
  }
  run_command(arm_.get(), "commit_model", *response, [&request](litearm::Arm & arm) {
    arm.model().commit(static_cast<uint16_t>(request->expected_mask));
  });
  if (response->success) {
    response->message +=
      " (expected_mask must match what the firmware has staged, or it refuses the commit)";
  }
}

void LitearmDriverNode::handle_revert_model(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  run_command(arm_.get(), "revert_model", *response, [](litearm::Arm & arm) {
    arm.model().revert();
  });
}

void LitearmDriverNode::handle_get_model_status(
  const litearm_msgs::srv::GetModelStatus::Request::SharedPtr request,
  litearm_msgs::srv::GetModelStatus::Response::SharedPtr response)
{
  (void)request;
  run_command(arm_.get(), "get_model_status", *response, [&response](litearm::Arm & arm) {
    const litearm::ModelStatus status = arm.model().status().value;
    response->override_level = status.override_level;
    response->staged_mask = status.staged_mask;
    response->dirty = status.dirty;
    response->message = status.dirty
      ? "RAM differs from flash: commit_model or revert_model"
      : "RAM and flash agree";
  });
}

}  // namespace litearm_driver
