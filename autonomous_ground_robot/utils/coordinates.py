"""
Coordinate conversion utilities.

GPS ↔ local Cartesian (ENU – East/North/Up) conversions and geodetic helpers.
The local frame origin is set at the first GPS fix received or at a
user-supplied reference coordinate.
"""

import math
from dataclasses import dataclass, field
from typing import Tuple

EARTH_RADIUS_M = 6_371_000.0  # mean Earth radius in metres


@dataclass
class GPSCoordinate:
    """WGS-84 geographic coordinate."""

    latitude: float   # degrees  [-90, 90]
    longitude: float  # degrees  [-180, 180]
    altitude: float = 0.0  # metres above ellipsoid


@dataclass
class LocalPoint:
    """Point in a local East/North/Up (ENU) Cartesian frame (metres)."""

    x: float = 0.0  # East
    y: float = 0.0  # North
    z: float = 0.0  # Up


def haversine_distance(a: GPSCoordinate, b: GPSCoordinate) -> float:
    """Return the great-circle distance (metres) between two GPS coordinates."""
    lat1, lon1 = math.radians(a.latitude), math.radians(a.longitude)
    lat2, lon2 = math.radians(b.latitude), math.radians(b.longitude)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(h))


def bearing_between(a: GPSCoordinate, b: GPSCoordinate) -> float:
    """
    Return the initial bearing (degrees, 0 = North, clockwise) from *a* to *b*.
    """
    lat1, lon1 = math.radians(a.latitude), math.radians(a.longitude)
    lat2, lon2 = math.radians(b.latitude), math.radians(b.longitude)
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360


def gps_to_local(coord: GPSCoordinate, origin: GPSCoordinate) -> LocalPoint:
    """
    Convert a GPS coordinate to a local ENU point relative to *origin*.

    Uses a simple flat-Earth approximation valid for distances up to ~100 km.
    """
    dlat = math.radians(coord.latitude - origin.latitude)
    dlon = math.radians(coord.longitude - origin.longitude)
    avg_lat = math.radians((coord.latitude + origin.latitude) / 2.0)
    x = EARTH_RADIUS_M * dlon * math.cos(avg_lat)  # East
    y = EARTH_RADIUS_M * dlat                       # North
    z = coord.altitude - origin.altitude            # Up
    return LocalPoint(x=x, y=y, z=z)


def local_to_gps(point: LocalPoint, origin: GPSCoordinate) -> GPSCoordinate:
    """
    Convert a local ENU point back to a GPS coordinate.
    """
    avg_lat_rad = math.radians(origin.latitude)
    dlat = math.degrees(point.y / EARTH_RADIUS_M)
    dlon = math.degrees(point.x / (EARTH_RADIUS_M * math.cos(avg_lat_rad)))
    return GPSCoordinate(
        latitude=origin.latitude + dlat,
        longitude=origin.longitude + dlon,
        altitude=origin.altitude + point.z,
    )
