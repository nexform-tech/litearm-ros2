// safety_gates.hpp — argument validation for the driver's service handlers.
//
// Every rule here exists because the firmware would otherwise accept the call and do
// something other than what was asked: clamp a value silently, fold an unknown bit into
// "all off", or ACK a mode change that never happens. The rules are free functions with
// no ROS and no SDK dependency, which is what makes them testable without hardware.

#ifndef LITEARM_DRIVER__SAFETY_GATES_HPP_
#define LITEARM_DRIVER__SAFETY_GATES_HPP_

#include <array>
#include <string>
#include <vector>

namespace litearm_driver
{

/// Pass or fail, plus the text a service response carries on failure.
struct GateResult
{
  bool ok = true;
  std::string message;
};

/// The speed governor is an integer percentage, so anything outside 0..100 is a mistake.
/// In particular 1 is one percent, not full speed.
GateResult check_speed_percent(int percent);

/// Feedforward presets are 0 (all off), 1 (factory) and 2 (all on).
GateResult check_ff_preset(int preset);

/// The zero-gravity keep-alive period has to satisfy the SDK contract [0.005, 0.10). The
/// upper bound exists because the firmware command watchdog trips at 0.10 s.
GateResult check_zero_g_period(double period_s);

/// An axis index has to address an axis the firmware actually reported.
GateResult check_joint_index(int index, int joint_count);

/// The firmware clamps mass into [0, 20] kg silently, so a value outside that range would
/// be stored as a different number than the one requested. Refuse it instead.
GateResult check_mass(double mass);

/// The centre of mass has to be finite in all three components.
GateResult check_com(const std::array<double, 3> & com);

/// Feedforward vectors must be non-empty and finite. The length the firmware expects
/// depends on the item (3 for the gravity vector, 7 elsewhere) and the SDK enforces it;
/// this gate only rejects values that could never be valid.
GateResult check_ff_values(const std::vector<double> & values);

/// Flash writes and the joint parameter reset require the motors to be disabled. The SDK
/// and the firmware both refuse otherwise; refusing here first gives a message that says
/// why, before a frame goes out.
GateResult check_requires_disabled(bool enabled, const std::string & operation);

/// Some services are dangerous enough that a parameter has to opt in explicitly.
GateResult check_parameter_allows(bool allowed, const std::string & parameter_name);

/// Most services need a live link; the message names the likely cause.
GateResult check_connected(bool connected);

/// Speeds the motion services take are fractions of full speed: 0 < speed <= 1. The SDK
/// and the firmware both read 30 as 30x, not as 30 percent.
GateResult check_speed_fraction(double speed);

/// Every value has to be finite. The two-argument form checks only that; the
/// three-argument form also requires an exact length, which is how a joint-length array
/// is checked against the axes the firmware reported.
GateResult check_finite_values(const std::vector<double> & values, const char * what);
GateResult check_finite_values(
  const std::vector<double> & values, std::size_t expected, const char * what);

/// A pose: six finite values, metres and radians.
GateResult check_finite_pose(const std::array<double, 6> & pose);

/// A three-component vector, such as the gravity vector.
GateResult check_finite_vector3(const std::array<double, 3> & values);

/// A log dump file name: a plain name inside the node's log directory, never a path. The
/// service writes files, so where they land is not the caller's choice.
GateResult check_log_filename(const std::string & filename);

/// Deadlines, not durations to guess: a negative timeout is a mistake except where the
/// SDK documents -1 as "use the default".
GateResult check_timeout(double timeout, const char * what);

}  // namespace litearm_driver

#endif  // LITEARM_DRIVER__SAFETY_GATES_HPP_
