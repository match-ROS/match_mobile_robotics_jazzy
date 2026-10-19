"""Timestamp-aligned planar pose errors, independent of ROS and Qt."""
from bisect import bisect_left
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def interpolate(samples, stamp, max_gap=0.1):
    """Interpolate (time, x, y, yaw); never extrapolate across lost tracking."""
    i = bisect_left(samples, (stamp,))
    if i < len(samples) and abs(samples[i][0] - stamp) < 1e-8:
        return samples[i][1:]
    if not 0 < i < len(samples):
        return None
    a, b = samples[i-1], samples[i]
    if b[0] - a[0] > max_gap:
        return None
    fraction = (stamp - a[0]) / (b[0] - a[0])
    return (a[1] + fraction*(b[1]-a[1]), a[2] + fraction*(b[2]-a[2]),
            wrap(a[3] + fraction*wrap(b[3]-a[3])))


def errors(estimate, reference):
    dx, dy = estimate[0]-reference[0], estimate[1]-reference[1]
    return dx, dy, math.hypot(dx, dy), math.degrees(wrap(estimate[2]-reference[2]))


def distribution(values):
    if not values:
        return None
    ordered = sorted(values)
    def quantile(p):
        index = (len(ordered)-1)*p
        low = int(index)
        return ordered[low] + (index-low)*(ordered[min(low+1, len(ordered)-1)]-ordered[low])
    return dict(rmse=math.sqrt(sum(v*v for v in values)/len(values)),
                median=quantile(0.5), p95=quantile(0.95), maximum=ordered[-1])
