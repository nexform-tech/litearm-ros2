cmake_minimum_required(VERSION 3.16)
project(litearm_ros2_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ─────────────────────────────────────────────────────────────────────────────
# Shared memory contract library
#
# The C++ plugin links it directly; the Python daemon calls the same .so
# through ctypes. The layout is defined in exactly one place,
# include/litearm_ros2_control/litearm_shm.h, the sizes on the two sides are
# cross-checked when the shared library is loaded (see
# shm_bridge._verify_sizes), and the field offsets are cross-checked by
# test/test_shm_layout.py using litearm_shm_layout_probe.
# ─────────────────────────────────────────────────────────────────────────────
add_library(litearm_shm SHARED src/litearm_shm.cpp)
target_include_directories(litearm_shm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(litearm_shm PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ─────────────────────────────────────────────────────────────────────────────
# ros2_control hardware interface plugin
# ─────────────────────────────────────────────────────────────────────────────
add_library(litearm_ros2_control SHARED src/litearm_system.cpp)
target_include_directories(litearm_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(litearm_ros2_control litearm_shm)
ament_target_dependencies(litearm_ros2_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface litearm_ros2_control.xml)

# ─────────────────────────────────────────────────────────────────────────────
# Layout probe: exports struct size/offsetof for the Python-side tests
# ─────────────────────────────────────────────────────────────────────────────
add_executable(litearm_shm_layout_probe test/litearm_shm_layout_probe.cpp)
target_link_libraries(litearm_shm_layout_probe litearm_shm)

# ─────────────────────────────────────────────────────────────────────────────
# Install
# ─────────────────────────────────────────────────────────────────────────────
install(TARGETS litearm_shm litearm_ros2_control
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS litearm_shm_layout_probe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY urdf config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(litearm_shm)
ament_export_targets(export_${PROJECT_NAME})

# ─────────────────────────────────────────────────────────────────────────────
# Python package (shared memory bindings + hardware daemon)
# ─────────────────────────────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/${PROJECT_NAME}
)

install(PROGRAMS scripts/litearm_hw_daemon
  DESTINATION lib/${PROJECT_NAME}
)

# Field acceptance / probe scripts (they must be executable, and both the .sh
# and the .py have to be installed)
install(PROGRAMS
  scripts/acceptance_control.sh
  scripts/e2e_probe.py
  scripts/demo_motion.py
  scripts/stm32_smoke.py
  scripts/firmware_reset.py
  DESTINATION lib/${PROJECT_NAME}
)

# ─────────────────────────────────────────────────────────────────────────────
# Tests
# ─────────────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # PYTEST_DISABLE_PLUGIN_AUTOLOAD=1: this machine has a newer anyio in
  # ~/.local than the system pytest 6.2.5, and its pytest plugin does
  # `import _pytest.scope` (pytest>=7 only) during plugin autoload, which
  # errors out immediately. These two tests only use pytest core features
  # (fixture/parametrize), so turning off third-party plugin autoload makes
  # them run reliably; if your pytest is newer, the variable is harmless.
  #
  # Note: ament_has_pytest() probes pytest at **configure time** with
  # execute_process, so set(ENV{..}) must be used here rather than APPEND_ENV
  # (the latter only takes effect while the tests run); otherwise the probe
  # fails -> the tests are never registered at all.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)

  ament_add_pytest_test(test_shm_layout test/test_shm_layout.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
    APPEND_ENV "LITEARM_SHM_LAYOUT_PROBE=$<TARGET_FILE:litearm_shm_layout_probe>"
  )
  ament_add_pytest_test(test_hw_daemon test/test_hw_daemon.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
    TIMEOUT 120
  )
  # litearm-stm32 protocol layer / link layer: golden vectors + round trip
  # against a fake firmware.
  # No hardware and no pyserial needed (the link layer is the stdlib's
  # os/termios/select).
  ament_add_pytest_test(test_firmware_reset test/test_firmware_reset.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
    TIMEOUT 120
  )

  ament_add_pytest_test(test_stm32_proto test/test_stm32_proto.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
    TIMEOUT 120
  )
  # Static validation of the XML assets + validation of the interface set
  # after xacro expansion.
  # Requires litearm / litearm_ros2_control to be installed (the xacro files
  # locate things with $(find ...)).
  ament_add_pytest_test(test_xml_assets test/test_xml_assets.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # Plugin classes referenced by the RViz config must actually exist (a wrong
  # class name does not make RViz fail to start, it just silently drops one
  # display, which is hard to pin down).
  ament_add_pytest_test(test_rviz_config test/test_rviz_config.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # The JTC tolerances must not be tighter than the firmware's own criterion
  # (this includes default_kp/kd matching the firmware parameter table value
  # for value), otherwise on real hardware normal motion would be misjudged by
  # JTC before the firmware notices and the whole trajectory would abort.
  # The expected values come from the litearm-stm32 parameter table; the test
  # skips automatically when that repository is not present.
  ament_add_pytest_test(test_controller_config test/test_controller_config.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

ament_package()
