from setuptools import find_packages, setup
import os

package_name = "litearm_ros2"

data_files = []
# install launch files
data_files.append(
    (os.path.join("share", package_name, "launch"),
     [os.path.join("launch", "litearm.launch.py")])
)
# install config
data_files.append(
    (os.path.join("share", package_name, "config"),
     [os.path.join("config", "litearm.yaml")])
)
# ament index resource marker
data_files.append(
    (os.path.join("share", "ament_index", "resource_index", "packages"),
     [os.path.join("resource", package_name)])
)
# package.xml
data_files.append(
    (os.path.join("share", package_name), ["package.xml"])
)

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages("src"),
    package_dir={"": "src"},
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="luochun",
    maintainer_email="56000204@qq.com",
    description="ROS2 (Humble) bridge package for the LiteArm robotic arm.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "litearm_node = litearm_ros2.litearm_node:main",
        ],
    },
)
