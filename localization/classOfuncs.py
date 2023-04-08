import numpy as np
from datetime import time as dtime

earth_radius = 6317000  # in metres


class helpers:
    """Helper functions for geo-conversions, temporal synchronisation, etc."""
    def __init__(self) -> None:
        pass

    def getlcl(self, time_utc):
        """UTC-to-local time conversion
        
        Args:
          utc time from serial port in datatime.time()

        Returns:
          local time in datatime.time()
        """
        t_crt_h = time_utc.hour + 8  # Time zone's modified here.
        if t_crt_h >= 24:
            t_crt_h = t_crt_h - 24  # Avoid hrs-count overflow
        t_crt_m = time_utc.minute
        t_crt_s = time_utc.second
        t_crt_mius = time_utc.microsecond
        t_crt = dtime(t_crt_h, t_crt_m, t_crt_s, t_crt_mius)
        return t_crt

    def GeoDist(self, lon_past, lat_past, lon_curr, lat_curr):
        """Use longitude and latitude to calculate distance in METRES, 
        based on Haversine formula.
     
        Args:
        lon_past: body's longitude, last instance
        lat_past: body's latitude, last instance
        lon_curr: body's longitude, current instance
        lat_curr: body's latitude, current instance
     
        Return:
        Distance d from the last to the current instance, in METRES.
        """
        lon1 = lon_past * np.pi / 180
        lat1 = lat_past * np.pi / 180
        lon2 = lon_curr * np.pi / 180
        lat2 = lat_curr * np.pi / 180

        deltaLon = lon2 - lon1
        deltaLat = lat2 - lat1
        a = np.power(np.sin(deltaLat / 2),
                     2) + np.cos(lat1) * np.cos(lat2) * np.power(
                         np.sin(deltaLon / 2), 2)
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
        d = earth_radius * c

        return d

    def Lon2Cartesian(self, lon):
        """Convert a geographical point on earth's surface to a coordinate 
        in Cartesian system where y-axis represents longtitude in METRES.
        
        Arg:
        lon: longitude of the geo-point
        
        Return:
        lonMtrs: longitude represented in Cartesian coordinate
        """
        lonMtrs = self.GeoDist(0, 0, lon, 0)
        if (lonMtrs < 0):
            lonMtrs *= -1

        return lonMtrs
    
    def Lat2Cartesian(self, lat):
        """Convert the latitude of a geo-point on earth's surface to Cartesian 
        coordinate where x-axis represents latitude in METRES.
        
        Arg:
        lat: latitude of the geo-point

        Return:
        latMtrs: latitude represented in Cartesian coordinate
        """
        latMtrs = self.GeoDist(0, 0, 0, lat)
        if (latMtrs < 0):
            latMtrs *= -1

        return latMtrs

    def Cartesian2Geo(self, lonMtrs, latMtrs):
        """Convert the Cartesian location in metres back to geopraphical 
        representation in longitude and latitude.
        
        Args:
        lonMtrs: longitude in Cartesian system
        latMtrs: latitude in Cartesian system

        Returns:
        lon = geographical longitude
        lat = geographical latitude
        """
        # Set the origin as starting point
        lon_start = 0
        lat_start = 0
        azimuth_lon = 90 * np.pi / 180
        angularDist_lon = lonMtrs / earth_radius
        # Compute temporary geo-location by moving it horizontally.
        lat_temp = np.arcsin(
            np.sin(lat_start) * np.cos(angularDist_lon) +
            np.cos(lat_start) * np.sin(angularDist_lon) * np.cos(azimuth_lon))
        lon_temp = lon_start + np.arctan2(
            np.sin(azimuth_lon) * np.sin(angularDist_lon) * np.cos(lat_start),
            np.cos(angularDist_lon) - np.sin(lat_start) * np.sin(lat_temp))
        # Normalise the longitude to -180 to +180
        lon_temp = np.remainder(lon_temp + 3 * np.pi, 2 * np.pi) - np.pi

        # Compute final geo-location by moving temp location vertically.
        azimuth_lat = 0
        lon_temp = lon_temp * np.pi / 180
        lat_temp = lat_temp * np.pi / 180
        angularDist_lat = latMtrs / earth_radius
        lat = np.arcsin(
            np.sin(lat_temp) * np.cos(angularDist_lat) +
            np.cos(lat_temp) * np.sin(angularDist_lat) * np.cos(azimuth_lat))
        lon_unnorm = lon_temp + np.arctan2(
            np.sin(azimuth_lat) * np.sin(angularDist_lat) * np.cos(lat_temp),
            np.cos(angularDist_lat) - np.sin(lat_temp) * np.sin(lat))
        lon = np.remainder(lon_unnorm + 3 * np.pi, 2 * np.pi) - np.pi

        return lon, lat
