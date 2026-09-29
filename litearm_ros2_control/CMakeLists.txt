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
# 共享内存契约库
#
# C++ 插件直接链接它；Python 守护进程用 ctypes 调用同一份 .so。
# 布局只定义在 include/litearm_ros2_control/litearm_shm.h 一处，
# 两侧大小在动态库加载时交叉校验（见 shm_bridge._verify_sizes），
# 字段偏移由 test/test_shm_layout.py 用 litearm_shm_layout_probe 交叉校验。
# ─────────────────────────────────────────────────────────────────────────────
add_library(litearm_shm SHARED src/litearm_shm.cpp)
target_include_directories(litearm_shm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(litearm_shm PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ─────────────────────────────────────────────────────────────────────────────
# ros2_control 硬件接口插件
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
# 布局探针：为 Python 侧测试导出结构体 size/offsetof
# ─────────────────────────────────────────────────────────────────────────────
add_executable(litearm_shm_layout_probe test/litearm_shm_layout_probe.cpp)
target_link_libraries(litearm_shm_layout_probe litearm_shm)

# ─────────────────────────────────────────────────────────────────────────────
# 安装
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
# Python 包（共享内存绑定 + 硬件守护进程）
# ─────────────────────────────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/${PROJECT_NAME}
)

install(PROGRAMS scripts/litearm_hw_daemon
  DESTINATION lib/${PROJECT_NAME}
)

# 现场验收 / 探针脚本（脚本需可执行，且要同时装 x.sh 与 .py）
install(PROGRAMS
  scripts/acceptance_control.sh
  scripts/e2e_probe.py
  scripts/demo_motion.py
  scripts/stm32_smoke.py
  scripts/firmware_reset.py
  DESTINATION lib/${PROJECT_NAME}
)

# ─────────────────────────────────────────────────────────────────────────────
# 测试
# ─────────────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # PYTEST_DISABLE_PLUGIN_AUTOLOAD=1：本机 ~/.local 里装了比系统 pytest 6.2.5
  # 更新的 anyio，其 pytest 插件在自动加载时会 import _pytest.scope（pytest>=7
  # 才有）而直接报错。这两个测试只用 pytest 核心功能（fixture/parametrize），
  # 关掉第三方插件自动加载即可稳定运行；若你的环境 pytest 版本较新，这个变量无害。
  #
  # 注意：ament_has_pytest() 是在【配置期】用 execute_process 探测 pytest 的，
  # 所以这里必须用 set(ENV{..}) 而非 APPEND_ENV（后者只作用于测试运行期），
  # 否则探测失败 -> 测试根本不会被注册。
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)

  ament_add_pytest_test(test_shm_layout test/test_shm_layout.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
    APPEND_ENV "LITEARM_SHM_LAYOUT_PROBE=$<TARGET_FILE:litearm_shm_layout_probe>"
  )
  ament_add_pytest_test(test_hw_daemon test/test_hw_daemon.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
    TIMEOUT 120
  )
  # litearm-stm32 协议层/链路层：黄金向量 + 假固件往返。
  # 不需要硬件、不需要 pyserial（链路层是 stdlib 的 os/termios/select）。
  ament_add_pytest_test(test_firmware_reset test/test_firmware_reset.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
    TIMEOUT 120
  )

  ament_add_pytest_test(test_stm32_proto test/test_stm32_proto.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
    TIMEOUT 120
  )
  # XML 资产静态校验 + xacro 展开后的接口集合校验。
  # 需要 litearm / litearm_ros2_control 已安装（xacro 里用 $(find ...) 定位）。
  ament_add_pytest_test(test_xml_assets test/test_xml_assets.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # RViz 配置引用的插件类必须真实存在（写错类名不会让 RViz 启动失败，
  # 只会静默少一块显示，很难定位）。
  ament_add_pytest_test(test_rviz_config test/test_rviz_config.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # JTC 容差不得比固件自己的判据更紧（含 default_kp/kd 与固件参数表逐值一致），
  # 否则真机上正常运动会先于固件被 JTC 误判并中止整条轨迹。
  # 期望值来自 litearm-stm32 的参数表；该仓库不在场时自动 skip。
  ament_add_pytest_test(test_controller_config test/test_controller_config.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

ament_package()
