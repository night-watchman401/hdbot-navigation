# Copyright 2026 hd
# SPDX-License-Identifier: Apache-2.0

"""使用真实 ROS 消息和替身节点验证巡检分支，无需启动 DDS。"""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from autopartol_robot import partol_node
from geometry_msgs.msg import Transform
from sensor_msgs.msg import Image


def fake_patrol(tmp_path):
    image = Image()
    image.header.stamp.sec = 10
    logger = Mock()
    node = SimpleNamespace(
        latest_image=image,
        image_save_path=str(tmp_path / 'nested' / 'images'),
        image_max_age_s=2.0,
        get_logger=lambda: logger,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=11_000_000_000)),
        get_current_pose=lambda: Transform(),
        bridge=Mock())
    return node


def test_rejected_goal_does_not_wait_for_result():
    node = SimpleNamespace(
        waitUntilNav2Active=Mock(), goToPose=Mock(return_value=False),
        get_logger=lambda: Mock(), isTaskComplete=Mock())
    assert partol_node.PatrolNode.nav_to_pose(node, Mock()) is None
    node.isTaskComplete.assert_not_called()


def test_missing_image_skips_save(tmp_path):
    node = fake_patrol(tmp_path)
    node.latest_image = None
    assert partol_node.PatrolNode.record_image(node) is False
    node.bridge.imgmsg_to_cv2.assert_not_called()


@pytest.mark.parametrize('seconds', [5, 12])
def test_stale_or_future_image_skips_save(tmp_path, seconds):
    node = fake_patrol(tmp_path)
    node.latest_image.header.stamp.sec = seconds
    assert partol_node.PatrolNode.record_image(node) is False
    node.bridge.imgmsg_to_cv2.assert_not_called()


def test_directory_creation_and_write_failure(tmp_path, monkeypatch):
    node = fake_patrol(tmp_path)
    write = Mock(return_value=False)
    monkeypatch.setattr(partol_node.cv2, 'imwrite', write)
    assert partol_node.PatrolNode.record_image(node) is False
    assert (tmp_path / 'nested' / 'images').is_dir()
    node.get_logger().error.assert_called_once()


def test_successful_write_uses_bgr_encoding(tmp_path, monkeypatch):
    node = fake_patrol(tmp_path)
    monkeypatch.setattr(partol_node.cv2, 'imwrite', Mock(return_value=True))
    assert partol_node.PatrolNode.record_image(node) is True
    node.bridge.imgmsg_to_cv2.assert_called_once_with(node.latest_image, 'bgr8')
