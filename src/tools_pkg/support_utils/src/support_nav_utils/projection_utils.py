import numpy as np
from pyproj import Proj


projection = Proj(proj='utm', zone=10, ellps='WGS84', preserve_units=False)


def utm_to_wgs(x, y):
    """
    Convert points between UTM (x,y) to WGS84 (lat, lon).

    Args:
        x: either a single X coord or list of X coords
        y: either a single Y coord or list of Y coords

    Returns:
        lat: either a single latitude or list of latitudes
        lon: either a single longitude or list of longitudes
    """
    # ensure of same type and list are of same length
    if isinstance(x, (list, np.ndarray)) or isinstance(y, (list, np.ndarray)):
        if isinstance(x, (list, np.ndarray)) and isinstance(y, (list, np.ndarray)):
            assert len(x) == len(y), "X and Y coord lists must both be of same size."
        else:
            raise ValueError("X and Y coord lists must both be of same size.")

    lon, lat = projection(x, y, inverse=True) # project coordinates from UTM to WGS84

    return lat, lon


def wgs_to_utm(lat, lon):
    """
    Convert points between WGS84 (lat, lon) to UTM (x,y).

    Args:
        lat: either a single latitude or list of latitudes
        lon: either a single longitude or list of longitudes

    Returns:
        x: either a single X coord or list of X coords
        y: either a single Y coord or list of Y coords
    """
    # ensure of same type and lists are of same length
    if isinstance(lat, (list, np.ndarray)) or isinstance(lon, (list, np.ndarray)):
        if isinstance(lat, (list, np.ndarray)) and isinstance(lon, (list, np.ndarray)):
            assert len(lat) == len(lon), "Latitude and Longitude lists must both be of same size."
        else:
            raise ValueError("Latitude and Longitude lists must both be of same size.")

    x, y = projection(lon, lat)  # project coordinates from WGS84 to UTM

    return x, y
