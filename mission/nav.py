import math
import time

from mission.const import DEGREE_FULL_CIRCLE, EARTH_RADIUS_METERS, MILLISECOND_SCALE


def current_milli_time():
    return round(time.time() * MILLISECOND_SCALE)


def calc_distance_and_azimuth(lat1, lng1, lat2, lng2):
    rad_lat1 = math.radians(lat1)
    rad_lng1 = math.radians(lng1)
    rad_lat2 = math.radians(lat2)
    rad_lng2 = math.radians(lng2)
    d_lat = rad_lat2 - rad_lat1
    d_lng = rad_lng2 - rad_lng1
    cos_lat1 = math.cos(rad_lat1)
    cos_lat2 = math.cos(rad_lat2)
    haversine = (
        math.sin(d_lat / 2.0) ** 2
        + cos_lat1 * cos_lat2 * math.sin(d_lng / 2.0) ** 2
    )
    haversine = max(0.0, min(1.0, haversine))
    central_angle = 2.0 * math.atan2(
        math.sqrt(haversine),
        math.sqrt(1.0 - haversine),
    )
    dist = EARTH_RADIUS_METERS * central_angle
    sin_lat1 = math.sin(rad_lat1)
    sin_lat2 = math.sin(rad_lat2)
    cos_d_lng = math.cos(d_lng)
    y = math.sin(d_lng) * cos_lat2
    x = cos_lat1 * sin_lat2 - sin_lat1 * cos_lat2 * cos_d_lng
    azi = math.degrees(math.atan2(y, x))
    if azi < 0.0:
        azi += DEGREE_FULL_CIRCLE
    return dist, azi
