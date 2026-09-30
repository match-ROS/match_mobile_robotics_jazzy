"""Causal planar moving average for Qualisys rigid-body poses."""

import math
from collections import deque

from geometry_msgs.msg import PoseStamped


class PlanarMovingAverage:
    def __init__(self, window_sec=0.2):
        if window_sec <= 0:
            raise ValueError('window_sec must be positive')
        self.window_sec = float(window_sec)
        self.samples = deque()

    def add(self, pose, received_at):
        """Return an x/y/yaw-only pose, stamped at the sample-window midpoint."""
        q = pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        stamp = pose.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        self.samples.append((
            received_at, stamp_ns, pose.pose.position.x, pose.pose.position.y,
            math.sin(yaw), math.cos(yaw), yaw,
        ))
        while self.samples and received_at - self.samples[0][0] > self.window_sec:
            self.samples.popleft()
        count = len(self.samples)
        mean_x = sum(sample[2] for sample in self.samples) / count
        mean_y = sum(sample[3] for sample in self.samples) / count
        sin_sum = sum(sample[4] for sample in self.samples)
        cos_sum = sum(sample[5] for sample in self.samples)
        mean_yaw = (
            math.atan2(sin_sum, cos_sum)
            if math.hypot(sin_sum, cos_sum) > 1e-9 else self.samples[-1][6]
        )
        mean_stamp_ns = sum(sample[1] for sample in self.samples) // count
        result = PoseStamped()
        result.header.frame_id = pose.header.frame_id
        result.header.stamp.sec, result.header.stamp.nanosec = divmod(mean_stamp_ns, 1_000_000_000)
        result.pose.position.x = mean_x
        result.pose.position.y = mean_y
        result.pose.orientation.z = math.sin(mean_yaw / 2.0)
        result.pose.orientation.w = math.cos(mean_yaw / 2.0)
        return result
