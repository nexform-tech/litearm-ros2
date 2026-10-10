// Tests for the driver's argument gates: the rules that stop a call which the firmware
// would otherwise accept and quietly turn into a different one.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "litearm_driver/safety_gates.hpp"

namespace litearm_driver
{
namespace
{

TEST(SafetyGates, SpeedPercentAcceptsTheWholeRange)
{
  EXPECT_TRUE(check_speed_percent(0).ok);
  EXPECT_TRUE(check_speed_percent(1).ok);
  EXPECT_TRUE(check_speed_percent(100).ok);
}

TEST(SafetyGates, SpeedPercentRejectsOutOfRangeAndSaysWhyOneIsNotFullSpeed)
{
  const auto too_high = check_speed_percent(101);
  EXPECT_FALSE(too_high.ok);
  EXPECT_NE(too_high.message.find("one percent"), std::string::npos);

  EXPECT_FALSE(check_speed_percent(-1).ok);
}

TEST(SafetyGates, FeedforwardPresetAcceptsOnlyTheThreeDocumentedValues)
{
  EXPECT_TRUE(check_ff_preset(0).ok);
  EXPECT_TRUE(check_ff_preset(1).ok);
  EXPECT_TRUE(check_ff_preset(2).ok);
  EXPECT_FALSE(check_ff_preset(3).ok);
  EXPECT_FALSE(check_ff_preset(-1).ok);
}

TEST(SafetyGates, ZeroGravityPeriodMatchesTheSdkContract)
{
  EXPECT_TRUE(check_zero_g_period(0.005).ok);
  EXPECT_TRUE(check_zero_g_period(0.04).ok);
  // 0.10 is excluded: the firmware command watchdog trips at 0.10 s.
  EXPECT_FALSE(check_zero_g_period(0.10).ok);
  EXPECT_FALSE(check_zero_g_period(0.004).ok);
  EXPECT_FALSE(check_zero_g_period(0.0).ok);
}

TEST(SafetyGates, ZeroGravityPeriodRejectsNonFiniteValues)
{
  const double nan = std::nan("");
  EXPECT_FALSE(check_zero_g_period(nan).ok);
}

TEST(SafetyGates, JointIndexMustAddressAReportedAxis)
{
  EXPECT_TRUE(check_joint_index(0, 7).ok);
  EXPECT_TRUE(check_joint_index(6, 7).ok);
  EXPECT_FALSE(check_joint_index(7, 7).ok);
  EXPECT_FALSE(check_joint_index(-1, 7).ok);
  // A bench setup with a single axis.
  EXPECT_TRUE(check_joint_index(0, 1).ok);
}

TEST(SafetyGates, JointIndexBeforeTheFirmwareReportsAxesIsRefused)
{
  const auto gate = check_joint_index(0, 0);
  EXPECT_FALSE(gate.ok);
  EXPECT_NE(gate.message.find("connect"), std::string::npos);
}

TEST(SafetyGates, MassRefusesWhatTheFirmwareWouldClampSilently)
{
  EXPECT_TRUE(check_mass(0.0).ok);
  EXPECT_TRUE(check_mass(1.5).ok);
  EXPECT_TRUE(check_mass(20.0).ok);

  const auto too_heavy = check_mass(20.5);
  EXPECT_FALSE(too_heavy.ok);
  EXPECT_NE(too_heavy.message.find("clamps"), std::string::npos);

  // A negative mass would be clamped to zero, so the stored value would not be the
  // requested one.
  EXPECT_FALSE(check_mass(-1.0).ok);
}

TEST(SafetyGates, MassRejectsNonFiniteValues)
{
  EXPECT_FALSE(check_mass(std::nan("")).ok);
  EXPECT_FALSE(check_mass(INFINITY).ok);
}

TEST(SafetyGates, CentreOfMassMustBeFinite)
{
  EXPECT_TRUE(check_com({0.0, 0.0, 0.05}).ok);
  EXPECT_FALSE(check_com({0.0, std::nan(""), 0.0}).ok);
}

TEST(SafetyGates, FeedforwardVectorsMustBeNonEmptyAndFinite)
{
  EXPECT_TRUE(check_ff_values({1.0, 2.0, 3.0}).ok);
  EXPECT_FALSE(check_ff_values({}).ok);
  EXPECT_FALSE(check_ff_values({1.0, INFINITY}).ok);
}

TEST(SafetyGates, FlashWritesRequireTheMotorsToBeDisabled)
{
  EXPECT_TRUE(check_requires_disabled(false, "save_params").ok);

  const auto gate = check_requires_disabled(true, "save_params");
  EXPECT_FALSE(gate.ok);
  EXPECT_NE(gate.message.find("save_params"), std::string::npos);
  EXPECT_NE(gate.message.find("disable"), std::string::npos);
}

TEST(SafetyGates, OptInParametersNameThemselvesInTheRefusal)
{
  EXPECT_TRUE(check_parameter_allows(true, "allow_dfu").ok);

  const auto gate = check_parameter_allows(false, "allow_dfu");
  EXPECT_FALSE(gate.ok);
  EXPECT_NE(gate.message.find("allow_dfu"), std::string::npos);
}

TEST(SafetyGates, DisconnectedMessageNamesTheLikelyCauses)
{
  EXPECT_TRUE(check_connected(true).ok);

  const auto gate = check_connected(false);
  EXPECT_FALSE(gate.ok);
  EXPECT_NE(gate.message.find("USB"), std::string::npos);
  EXPECT_NE(gate.message.find("ros2_control"), std::string::npos);
}

TEST(SafetyGates, SpeedFractionRejectsPercentAndOutOfRange)
{
  EXPECT_TRUE(check_speed_fraction(0.5).ok);
  EXPECT_TRUE(check_speed_fraction(1.0).ok);
  EXPECT_FALSE(check_speed_fraction(0.0).ok);
  EXPECT_FALSE(check_speed_fraction(30.0).ok);          // 30x, not 30 percent
  EXPECT_FALSE(check_speed_fraction(-0.1).ok);
  EXPECT_FALSE(check_speed_fraction(std::nan("")).ok);
}

TEST(SafetyGates, FiniteValuesCheckLengthAndFiniteness)
{
  EXPECT_TRUE(check_finite_values({1.0, 2.0}, "values").ok);
  EXPECT_FALSE(check_finite_values({}, "values").ok);
  EXPECT_TRUE(check_finite_values({1.0, 2.0}, 2u, "q").ok);
  EXPECT_FALSE(check_finite_values({1.0}, 7u, "q").ok);
  EXPECT_FALSE(check_finite_values({1.0, std::nan("")}, "values").ok);
  EXPECT_FALSE(check_finite_values({std::numeric_limits<double>::infinity()}, "values").ok);
}

TEST(SafetyGates, PosesAndVectorsAreCheckedByShape)
{
  EXPECT_TRUE(check_finite_pose({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}).ok);
  EXPECT_FALSE(check_finite_pose({0.0, 0.0, 0.0, 0.0, 0.0, std::nan("")}).ok);
  EXPECT_TRUE(check_finite_vector3({0.0, 0.0, -9.81}).ok);
  EXPECT_FALSE(check_finite_vector3({0.0, 0.0, std::nan("")}).ok);
}

TEST(SafetyGates, LogFilenameIsANameNotAPath)
{
  EXPECT_TRUE(check_log_filename("tick_log.bin").ok);
  EXPECT_FALSE(check_log_filename("").ok);
  EXPECT_FALSE(check_log_filename("sub/dir.bin").ok);
  EXPECT_FALSE(check_log_filename("..").ok);
  EXPECT_FALSE(check_log_filename("..\\escape.bin").ok);
}

TEST(SafetyGates, TimeoutIsADeadlineNotANegativeGuess)
{
  EXPECT_TRUE(check_timeout(0.0, "timeout").ok);
  EXPECT_TRUE(check_timeout(1.5, "timeout").ok);
  EXPECT_FALSE(check_timeout(-1.0, "timeout").ok);
  EXPECT_FALSE(check_timeout(std::nan(""), "timeout").ok);
}

}  // namespace
}  // namespace litearm_driver
