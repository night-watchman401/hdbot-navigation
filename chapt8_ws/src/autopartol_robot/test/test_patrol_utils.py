# Copyright 2026 hd
# SPDX-License-Identifier: Apache-2.0

"""检查巡检参数边界与重复到点图像的命名行为。"""

import pytest

from autopartol_robot.patrol_utils import image_filename, parse_points


def test_group_points():
    assert parse_points([0.0, 1.0, 0.5, 2.0, 3.0, -0.5]) == [
        [0.0, 1.0, 0.5], [2.0, 3.0, -0.5]]


@pytest.mark.parametrize('values', [[], [1, 2], [1, 2, 3, 4],
                                   [0, 0, float('nan')],
                                   [0, float('inf'), 0], [0, '1', 0],
                                   [False, 0, 0]])
def test_reject_invalid_points(values):
    with pytest.raises(ValueError):
        parse_points(values)


def test_initial_pose_is_exactly_one_point():
    assert parse_points([0, 0, 0], single=True) == [[0, 0, 0]]
    with pytest.raises(ValueError):
        parse_points([0, 0, 0, 1, 1, 1], single=True)


def test_repeated_capture_does_not_reuse_filename():
    first = image_filename(1.5, -2.0, 10, 5)
    second = image_filename(1.5, -2.0, 10, 5)
    assert first.startswith('image_1.50_-2.00_10_000000005_')
    assert first.endswith('.png')
    assert first != second
