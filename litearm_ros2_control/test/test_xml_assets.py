#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""XML 资产与描述配置的静态校验。

存在的理由（都是踩过的坑）
--------------------------
1. **XML 注释里不能出现 ASCII 双连字符 ``--``**。
   我用 ``------`` 做注释分割线、用 ``--shm-name`` 引命令行参数，两处都让
   xacro 直接抛 "not well-formed (invalid token)"，而且报的是抽象的行列号，
   定位成本很高。这里显式校验，并把违规位置指出来。

2. **xacro 展开后的接口集合必须与插件导出的一致**。
   描述文件写了几个 <command_interface>，插件就必须导出对应的接口名；
   两边对不上时，ros2_control 的表现是控制器激活失败或静默读写错位。
   这里把"7 个关节 × 6 个命令接口 / 7 个状态接口"钉死。

   注意：本测试只读源码树，不需要起节点。
"""

import os
import re
import sys
import xml.etree.ElementTree as ET

import pytest

NUM_JOINTS = 7
EXPECTED_COMMAND_INTERFACES = ["position", "velocity", "acceleration", "effort",
                               "kp", "kd"]
EXPECTED_STATE_INTERFACES = ["position", "velocity", "effort", "temperature_mos",
                             "temperature_coil", "error_code", "feedback_age"]
EXPECTED_DIAGNOSTIC_FREE_STATES = ["position", "velocity", "effort"]


def _package_source_dir() -> str:
    """本包源码根（脚本向上两级：test/ → 包根）。"""
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def _workspace_src_dir() -> str:
    """工作区 src 目录（包根再向上一级）。"""
    return os.path.dirname(_package_source_dir())


def _xml_assets():
    """收集工作区 src 下全部 XML 类资产（含 litearm 描述包自带的 URDF）。"""
    pattern = re.compile(r".*\.(xacro|urdf|srdf|xml)$")
    found = []
    for root, dirs, files in os.walk(_workspace_src_dir()):
        dirs[:] = [d for d in dirs
                   if d not in ("build", "install", "__pycache__", ".git")]
        for name in files:
            if pattern.match(name):
                found.append(os.path.join(root, name))
    return sorted(found)


def test_xml_assets_exist():
    """至少应该找到 xacro / srdf / plugin xml —— 找不到说明路径推算错了。"""
    assets = _xml_assets()
    names = {os.path.basename(p) for p in assets}
    for expected in ("litearm.ros2_control.xacro", "litearm.srdf",
                     "litearm_ros2_control.xml"):
        assert expected in names, f"未收集到 {expected}；实际: {sorted(names)}"


@pytest.mark.parametrize("path", _xml_assets(), ids=os.path.basename)
def test_no_double_hyphen_in_xml_comments(path):
    """XML 注释体内不得出现 ``--``（``-->`` 的收尾不算）。"""
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    offenders = []
    for match in re.finditer(r"<!--(.*?)-->", text, re.S):
        body = match.group(1)
        if "--" in body:
            line = text[:match.start()].count("\n") + 1
            column = text.rfind("\n", 0, match.start()) + 1
            snippet = body.strip().splitlines()[0][:60] if body.strip() else ""
            offenders.append(f"第 {line} 行（注释起点列 {match.start() - column + 1}）: {snippet}")
    assert not offenders, (
        f"{path} 的 XML 注释里出现非法双连字符，xacro/解析器会直接报 "
        f"'not well-formed'：\n  " + "\n  ".join(offenders) +
        "\n请改成全角破折号或改写措辞。")


@pytest.mark.parametrize("path", _xml_assets(), ids=os.path.basename)
def test_xml_assets_are_well_formed(path):
    """所有 XML 资产都能被解析（xacro 文件本身也是合法 XML）。"""
    try:
        ET.parse(path)
    except ET.ParseError as exc:
        pytest.fail(f"{path} 不是合法 XML: {exc}")


def test_ros2_control_xacro_expands_to_expected_interfaces():
    """展开 litearm.urdf.xacro，校验 ros2_control 块的接口集合。"""
    xacro = pytest.importorskip("xacro")
    from ament_index_python.packages import get_package_share_directory

    urdf_path = os.path.join(get_package_share_directory("litearm_ros2_control"),
                             "urdf", "litearm.urdf.xacro")
    if not os.path.exists(urdf_path):
        pytest.skip("尚未 colcon build / install，找不到已安装的 xacro")

    document = xacro.process_file(urdf_path)
    root = ET.fromstring(document.toxml())

    assert root.get("name") == "litearm"

    # 上游描述包必须被真正包含进来（否则 meshes / 关节都会缺失）
    links = [link.get("name") for link in root.findall("link")]
    joints = [joint.get("name") for joint in root.findall("joint")]
    assert len(links) == 9, f"应有 9 个 link，实际 {links}"
    assert len(joints) == 8, f"应有 8 个 joint（7 revolute + ee_link fixed），实际 {joints}"
    assert "ee_link" in links

    blocks = root.findall("ros2_control")
    assert len(blocks) == 1, "应恰好有一个 <ros2_control> 块"
    block = blocks[0]
    assert block.get("type") == "system"
    plugin = block.find("hardware/plugin")
    assert plugin is not None
    assert plugin.text == "litearm_ros2_control/LitearmSystem"

    hardware_params = {p.get("name") for p in block.findall("hardware/param")}
    for required in ("shm_name", "default_kp", "default_kd",
                     "export_diagnostic_interfaces", "connect_timeout_s",
                     "heartbeat_timeout_s"):
        assert required in hardware_params, f"缺少硬件参数 {required}"

    ros_joints = block.findall("joint")
    assert len(ros_joints) == NUM_JOINTS, \
        f"ros2_control 应声明 {NUM_JOINTS} 个关节，实际 {len(ros_joints)}"
    assert [j.get("name") for j in ros_joints] == [f"joint{i}"
                                                   for i in range(1, NUM_JOINTS + 1)]

    for joint in ros_joints:
        commands = [ci.get("name") for ci in joint.findall("command_interface")]
        states = [si.get("name") for si in joint.findall("state_interface")]
        assert commands == EXPECTED_COMMAND_INTERFACES, \
            f"{joint.get('name')} 命令接口不匹配: {commands}"
        assert states == EXPECTED_STATE_INTERFACES, \
            f"{joint.get('name')} 状态接口不匹配: {states}"

    # default_kp / default_kd 必须是 7 个逗号分隔的数 —— 插件侧会解析失败并报错，
    # 这里提前拦住
    for name in ("default_kp", "default_kd"):
        value = block.find(f"hardware/param[@name='{name}']").text
        parts = [p for p in value.split(",") if p.strip()]
        assert len(parts) == NUM_JOINTS, f"{name}='{value}' 不是 {NUM_JOINTS} 个数"
        for part in parts:
            float(part)


def test_diagnostics_can_be_disabled():
    """export_diagnostic_interfaces=false 时不应再导出诊断状态接口。"""
    xacro = pytest.importorskip("xacro")
    from ament_index_python.packages import get_package_share_directory

    share = get_package_share_directory("litearm_ros2_control")
    xacro_path = os.path.join(share, "urdf", "litearm.urdf.xacro")
    control_xacro = os.path.join(share, "urdf", "litearm.ros2_control.xacro")
    if not os.path.exists(xacro_path):
        pytest.skip("尚未 colcon build / install，找不到已安装的 xacro")

    # 直接展开 ros2_control 片段并注入 arg，避免依赖上层文件的 arg 默认值
    document = xacro.process_file(
        control_xacro,
        mappings={"litearm_export_diagnostics": "false"})
    root = ET.fromstring(document.toxml())
    for joint in root.findall("ros2_control/joint"):
        states = [si.get("name") for si in joint.findall("state_interface")]
        assert states == EXPECTED_DIAGNOSTIC_FREE_STATES, \
            f"关掉诊断接口后仍导出: {states}"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
