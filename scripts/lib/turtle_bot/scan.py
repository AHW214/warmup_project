"""
TurtleBot3 LiDAR interface.
"""

from dataclasses import dataclass
from math import isinf
from typing import Optional
from sensor_msgs.msg import LaserScan


@dataclass
class ScanPoint:
    """
    A point representing an object scanned by LiDAR.
    """

    angle_deg: int
    angle_rad: float
    distance: float


def closest_scan_point(scan: LaserScan) -> Optional[ScanPoint]:
    """
    Create a ScanPoint from the closest measurement in the LaserScan.
    """
    (angle_deg_360, distance) = min(enumerate(scan.ranges), key=lambda t: t[1])

    if isinf(distance):
        return None

    angle_deg = angle_deg_360 - (360 if angle_deg_360 > 180 else 0)
    angle_rad = scan.angle_increment * angle_deg

    return ScanPoint(angle_deg, angle_rad, distance)


def scan_point_at(angle_deg: int, scan: LaserScan) -> Optional[ScanPoint]:
    """
    Try to create a ScanPoint from the measurement at the given degree angle.
    """
    index = angle_deg + (360 if angle_deg < 0 else 0)

    if index < 0 or index >= len(scan.ranges) or isinf(distance := scan.ranges[index]):
        return None

    angle_rad = scan.angle_increment * angle_deg

    return ScanPoint(angle_deg, angle_rad, distance)
