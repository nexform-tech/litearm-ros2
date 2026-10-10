cmake_minimum_required(VERSION 3.16)
project(litearm_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(litearm_msgs REQUIRED)

# ─────────────────────────────────────────────────────────────────────────────
# litearm C++ SDK
#
# Same resolution order as litearm_ros2_control: an installed litearm first, otherwise the
# litearm-cpp source tree that sits beside this repository in the workspace. An executable
# may link a static, non-PIC litearm, so this package does not need the shared-object
# check that the plugin needs.
# ─────────────────────────────────────────────────────────────────────────────
set(LITEARM_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../litearm-cpp"
  CACHE PATH "litearm-cpp source tree, used when no installed litearm package is found")

find_package(litearm CONFIG QUIET)

# ⚠ The robot description package is also called `litearm`, so find_package can resolve
# to its CMake config instead of the SDK's -- they share a name, and the description
# package exports no target at all. Only the SDK exports litearm::litearm, so the
# target, not litearm_FOUND, is the test for "the SDK is installed".
if(litearm_FOUND AND TARGET litearm::litearm)
  message(STATUS "Using installed litearm (${litearm_DIR})")
else()
  if(NOT EXISTS "${LITEARM_CPP_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "No installed litearm package was found, and the litearm-cpp source tree is not at "
      "'${LITEARM_CPP_DIR}'. Either install litearm-cpp or pass "
      "-DLITEARM_CPP_DIR=<path to litearm-cpp>.")
  endif()
  set(BUILD_SHARED_LIBS OFF)
  set(LITEARM_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(LITEARM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  add_subdirectory("${LITEARM_CPP_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/litearm-cpp"
    EXCLUDE_FROM_ALL)
  message(STATUS "Using in-tree litearm from ${LITEARM_CPP_DIR} (static, linked in)")
endif()

# ─────────────────────────────────────────────────────────────────────────────
# node library + executable
#
# The node lives in a library so the offline test can drive its lifecycle and its service
# handlers directly, without a client and without spinning an executor.
# ─────────────────────────────────────────────────────────────────────────────
add_library(litearm_driver_node STATIC
  src/litearm_driver_node.cpp
  src/litearm_driver_motion.cpp
  src/litearm_driver_model.cpp
  src/litearm_driver_diagnostics.cpp
  src/safety_gates.cpp
)
target_include_directories(litearm_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(litearm_driver_node
  rclcpp
  rclcpp_lifecycle
  std_srvs
  sensor_msgs
  diagnostic_msgs
  litearm_msgs
)
target_link_libraries(litearm_driver_node litearm::litearm)

add_executable(litearm_driver src/litearm_driver_main.cpp)
target_link_libraries(litearm_driver litearm_driver_node)

# ─────────────────────────────────────────────────────────────────────────────
# install
# ─────────────────────────────────────────────────────────────────────────────
install(TARGETS litearm_driver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ─────────────────────────────────────────────────────────────────────────────
# tests
# ─────────────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Pure validation rules: no ROS, no SDK, no hardware.
  ament_add_gtest(test_safety_gates test/test_safety_gates.cpp)
  target_link_libraries(test_safety_gates litearm_driver_node)

  # Whole lifecycle plus every service handler, against the SDK's FakeTransport.
  ament_add_gtest(test_litearm_driver test/test_litearm_driver.cpp)
  ament_target_dependencies(test_litearm_driver
    rclcpp
    rclcpp_lifecycle
    std_srvs
    sensor_msgs
    diagnostic_msgs
    litearm_msgs
  )
  target_link_libraries(test_litearm_driver litearm_driver_node litearm::litearm)
endif()

ament_package()
