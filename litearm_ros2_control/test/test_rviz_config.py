#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RViz 配置里引用的插件类必须真实存在。

为什么需要这个测试
------------------
RViz 遇到配置里不存在的 ``Class`` 时**不会启动失败**，只在日志里刷一行
``The plugin for class '...' failed to load``，然后那张显示/面板就不出现。
从界面看就是"少了东西"，很难联想到是自己把类名写错了 —— 本工程就踩过：
配置里写了 ``rviz_default_plugins/Plot`` 与 ``rviz_default_plugins/JointStatePublisher``，
两个类在 Humble 里都不存在，结果 RViz 起来后既没有曲线也没有关节状态。

离线渲染（``QT_QPA_PLATFORM=offscreen``）在本环境里因为拿不到 GL 上下文而
直接崩在创建渲染窗口那一步，没法当测试手段，所以在这里做静态校验。

类名的两个来源
--------------
1. 所有已安装 RViz 插件描述 XML 里的 ``<class name="...">`` —— 覆盖 Display 与 Tool。
   扫描范围取自 ``AMENT_PREFIX_PATH``（ROS 安装前缀 + 本工作区 install 树），
   不写死 ``/opt/ros/<distro>``，换发行版/换机器都能用。
2. RViz **原生注册、不出现在任何 XML 里**的 Panel / View。名单取自
   ``$ROS_DISTRO/share/rviz_common/default.rviz``（RViz 自带默认配置）
   与官方 MoveIt 模板 ``moveit_setup_app_plugins/templates/config/moveit.rviz``。
"""

import glob
import os
import sys
import xml.etree.ElementTree as ET
from typing import List, Set

import pytest
import yaml

# RViz 原生注册、无插件描述 XML 的类（来源见模块 docstring）
NATIVE_RVIZ_CLASSES: Set[str] = {
    "rviz_common/Displays",
    "rviz_common/Views",
    "rviz_common/Selection",
    "rviz_common/Tool Properties",
    "rviz_common/Time",
    "rviz_common/Help",
    "rviz_common/Identity",
}


def _ament_prefixes() -> List[str]:
    """所有 ament 安装前缀（ROS 安装 + 本工作区 install 树）。

    由 colcon test / ROS 环境注入的 ``AMENT_PREFIX_PATH`` 决定，
    不假设 ROS 装在 ``/opt/ros/<distro>``。
    """
    return [p for p in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if p]


def _installed_plugin_classes() -> Set[str]:
    """扫描所有 ament 前缀下 RViz 插件描述 XML，收集带命名空间的类名。"""
    classes: Set[str] = set()
    for prefix in _ament_prefixes():
        for pattern in ("share/*/*plugin*.xml", "share/*/*.xml"):
            for path in glob.glob(os.path.join(prefix, pattern)):
                if "rviz" not in path and "moveit" not in path and "visual_tools" not in path:
                    continue
                try:
                    root = ET.parse(path).getroot()
                except ET.ParseError:
                    continue
                for element in root.iter("class"):
                    name = element.get("name")
                    if name and "/" in name:
                        classes.add(name)
    return classes


def _workspace_src_dir() -> str:
    return os.path.dirname(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


def _rviz_configs():
    found = []
    for root, dirs, files in os.walk(os.path.dirname(_workspace_src_dir())):
        dirs[:] = [d for d in dirs
                   if d not in ("build", "install", "__pycache__", ".git")]
        for name in files:
            if name.endswith(".rviz"):
                found.append(os.path.join(root, name))
    return sorted(found)


def _collect_classes(node, out: Set[str]) -> None:
    """递归收集所有 ``Class`` 键的值（面板/显示/工具/视图里都有）。"""
    if isinstance(node, dict):
        for key, value in node.items():
            if key == "Class" and isinstance(value, str) and value.strip():
                out.add(value.strip())
            else:
                _collect_classes(value, out)
    elif isinstance(node, list):
        for item in node:
            _collect_classes(item, out)


@pytest.fixture(scope="module")
def known_classes() -> Set[str]:
    return _installed_plugin_classes() | NATIVE_RVIZ_CLASSES


def test_plugin_catalogue_is_non_trivial(known_classes):
    """清单本身要合理：拿不到插件描述就说明扫描逻辑坏了，而不是配置没问题。"""
    assert len(known_classes) > 30, \
        f"只找到 {len(known_classes)} 个插件类，插件描述扫描可能失效"
    for expected in ("rviz_default_plugins/Grid", "rviz_default_plugins/RobotModel",
                     "rviz_default_plugins/TF", "moveit_rviz_plugin/MotionPlanning"):
        assert expected in known_classes, f"清单里缺少 {expected}"


def test_rviz_configs_exist():
    configs = _rviz_configs()
    names = {os.path.basename(p) for p in configs}
    for expected in ("litearm_control.rviz", "moveit.rviz"):
        assert expected in names, f"未收集到 {expected}；实际: {sorted(names)}"


@pytest.mark.parametrize("path", _rviz_configs(), ids=os.path.basename)
def test_all_referenced_classes_are_known(path, known_classes):
    """配置里出现的每个 Class 都必须是本机真实存在的插件。"""
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    used: Set[str] = set()
    _collect_classes(data, used)

    unknown = sorted(name for name in used if name not in known_classes)
    assert not unknown, (
        f"{path} 引用了不存在的 RViz 插件类: {unknown}\n"
        "RViz 不会因此启动失败，只会静默跳过那项显示/面板 —— 界面上看就是"
        "'少了东西'。请核对类名（可用以下命令列出本机全部可用类）：\n"
        "  python3 -c \"import glob, os, xml.etree.ElementTree as ET; "
        "[print(c.get('name')) for p in os.environ['AMENT_PREFIX_PATH'].split(os.pathsep) "
        "for f in glob.glob(os.path.join(p, 'share', '*', '*.xml')) "
        "for c in ET.parse(f).getroot().iter('class')]\"")


@pytest.mark.parametrize("path", _rviz_configs(), ids=os.path.basename)
def test_robot_model_uses_transient_local(path):
    """RobotModel 若从话题取描述，Durability 必须是 Transient Local。

    robot_state_publisher 以 transient_local 发布 /robot_description（锁存，
    新订阅者也能拿到最后一条）。写成 Volatile 的话，RViz 在 RSP 之后启动就
    拿不到那条消息，表现为"RViz 起来了但没有机器人"。
    """
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    if "rviz_default_plugins/RobotModel" not in text:
        pytest.skip("该配置未使用 RobotModel 显示")
    assert "Transient Local" in text, (
        f"{path} 的 RobotModel 显示没有用 Transient Local 订阅 /robot_description，"
        f"RViz 后启动时会取不到锁存的机器人描述")
    assert "Description Source: Topic" in text or "Description Source: File" in text, \
        f"{path} 缺少 Description Source，RobotModel 不知道该从哪取描述"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
