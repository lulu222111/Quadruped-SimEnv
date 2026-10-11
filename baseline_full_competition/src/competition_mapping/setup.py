#!/usr/bin/env python3
# 中文说明：catkin_python_setup() 使用的安装描述——把 src/competition_mapping
# 作为 Python 包对外安装（导出楼层地图管理与栅格核心），供 scripts/mapping_node
# 以及其它模块直接 import。
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=["competition_mapping"],
    package_dir={"": "src"},
)
setup(**setup_args)
