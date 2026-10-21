# Copyright 2026 hd
# SPDX-License-Identifier: Apache-2.0

"""巡检参数校验与文件命名，无需启动 ROS 即可验证。"""

import math
from uuid import uuid4


def parse_points(values, single=False):
    """将有限数值数组按 x、y、yaw 分组，拒绝空数组及残缺目标点。"""
    if not values or len(values) % 3 or (single and len(values) != 3):
        raise ValueError('位姿参数必须包含完整的 [x, y, yaw]，且不能为空')
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) for value in values):
        raise ValueError('位姿参数只能包含有限数值，不能包含 NaN 或无穷大')
    return [list(values[index:index + 3]) for index in range(0, len(values), 3)]


def image_filename(x, y, seconds, nanoseconds):
    """保留采集时间、地图位置和随机标识，避免循环巡检覆盖同名文件。"""
    return (f'image_{x:.2f}_{y:.2f}_{seconds}_{nanoseconds:09d}_'
            f'{uuid4().hex}.png')
