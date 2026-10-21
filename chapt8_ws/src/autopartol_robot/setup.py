# Copyright 2026 hd
# SPDX-License-Identifier: Apache-2.0

from setuptools import find_packages, setup

package_name = 'autopartol_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE']),
        ('share/' + package_name+"/config", ['config/partol_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hd',
    maintainer_email='1363508082@qq.com',
    description='基于 Nav2 的多目标点循环巡检与图像记录节点',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
             "partol_node = autopartol_robot.partol_node:main",
        ],
    },
)
