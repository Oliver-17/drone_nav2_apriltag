#!/usr/bin/env python3
"""Small ENU/NED conversion helpers for ROS and PX4 frames."""

import math
from dataclasses import dataclass
from typing import List


@dataclass
class VectorENU:
    """ROS ENU vector: x=East, y=North, z=Up."""

    east: float
    north: float
    up: float

    def as_list(self) -> List[float]:
        return [self.east, self.north, self.up]


@dataclass
class VectorNED:
    """PX4 NED vector: x=North, y=East, z=Down."""

    north: float
    east: float
    down: float

    def as_list(self) -> List[float]:
        return [self.north, self.east, self.down]


def enu_to_ned(vector: VectorENU) -> VectorNED:
    """ROS ENU -> PX4 NED."""
    return VectorNED(
        north=vector.north,
        east=vector.east,
        down=-vector.up,
    )


def ned_to_enu(vector: VectorNED) -> VectorENU:
    """PX4 NED -> ROS ENU."""
    return VectorENU(
        east=vector.east,
        north=vector.north,
        up=-vector.down,
    )


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_enu_to_ned(yaw_enu: float) -> float:
    """ROS ENU yaw -> PX4 NED yaw."""
    return normalize_angle(math.pi / 2.0 - yaw_enu)


def yaw_ned_to_enu(yaw_ned: float) -> float:
    """PX4 NED yaw -> ROS ENU yaw."""
    return normalize_angle(math.pi / 2.0 - yaw_ned)
