#! /usr/bin/env python3

from shapely.geometry import Polygon
from onav_sdk.onav_api import *
import pyproj 
import math

MAP_NAME = "field_example" #name of map from row generator (saved copy)
MISSION_NAME = f"{MAP_NAME}_mission"
POIS = 14
feet = lambda f : f * 0.3048 
P = pyproj.Proj(proj='utm', zone=17, ellps='WGS84', preserve_units=True)
#PTZ Position 1
pan1, pos1 = -math.pi/2, 1.0
#PTZ Position 2
pan2, pos2 = math.pi/2, 2.0

def get_map(map_name):
    ONAV_MM_GET_ALL_MAPS.RequestType()
    res = ONAV_MM_GET_ALL_MAPS.call()
    for m in res.maps:
        if m.name == map_name:
            print(f"Retrieved map: {m.name}")
            return m
    return None

def LonLat_To_XY(Lon, Lat):
    return P(Lon, Lat)    

def XY_To_LonLat(x,y):
    return P(x, y, inverse=True)    

def convert_list_to_xy(points):
    converted = []
    for point in points:
        converted.append(LonLat_To_XY(point[0],point[1]))
    return converted

def convert_list_to_lat_lon(points):
    converted = []
    for point in points:
        converted.append(XY_To_LonLat(point[0],point[1]))
    return converted

def calc_bearing(lat1, long1, lat2, long2):
  lat1 = math.radians(lat1)
  long1 = math.radians(long1)
  lat2 = math.radians(lat2)
  long2 = math.radians(long2)
  bearing = math.atan2(
      math.sin(long2 - long1) * math.cos(lat2),
      math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(long2 - long1)
  )
  bearing = math.degrees(bearing)
  bearing = (bearing + 360) % 360
  return bearing

def ned_to_enu(ned_heading):
    enu_heading = 90 - ned_heading
    if enu_heading < 0:
        enu_heading += 360
    return enu_heading

def get_polygon(m):
    try:
        vertices = [(p.latitude, p.longitude) for p in m.points]
        utm_vertices = convert_list_to_xy(vertices)
        map_polygon = Polygon(utm_vertices)
        print("Retrieved polygon")
        return utm_vertices, map_polygon
    except:
        return None,None

def create_ptz_task(pan, position):
    
    req = ONAV_MM_TASK_CREATE.RequestType()
    req.name = "Move PTZ"
    req.action_server_name = "/camera/move_ptz_task_adaptor"
    req.version = "0.0.0"
    req.floats = [pan, 0.0, 1.0, position]
    req.strings = ["/sensors/camera_0","/sensors/camera_0/image_raw_out"]
    req.allow_failure = True
    res = ONAV_MM_TASK_CREATE.call(req)
    if res.result:
        print(f"Task created with UUID: {res.result.uuid}")
        return res.result.uuid
    return None
    

def create_mission(mission_name, waypoint_uuids):
    req = ONAV_MM_MISSION_CREATE.RequestType()
    req.name = mission_name
    req.waypoint_ids = waypoint_uuids
    req.network_replan_enabled = True
    res = ONAV_MM_MISSION_CREATE.call(req)
    if res.result:
        print(f"Mission {mission_name} created with UUID: {res.result.uuid}")
        return res.result.uuid
    return None

def generate_waypoints(edges, heading, uuid1, uuid2):
    
    waypoint_uuids = []    
    counter = 1

    for i in range(len(edges)-POIS):
        counter = counter if 1<=counter<=4 else 1
        uuid = ""
        if counter == 1 or counter == 2 :
            point = edges[i]
            waypoint_lat, waypoint_lon = XY_To_LonLat(point[0], point[1])
            waypoint_heading = heading
            if counter == 1:
                uuid = uuid1
        elif counter == 3:
            point = edges[i+1]
            waypoint_lat, waypoint_lon = XY_To_LonLat(point[0], point[1])
            waypoint_heading = heading + math.pi
            uuid = uuid2
        elif counter == 4:
            point = edges[i-1]
            waypoint_lat, waypoint_lon = XY_To_LonLat(point[0], point[1])
            waypoint_heading = heading + math.pi
        else:
            print("Counter error")
            break
        print(i)
        req = ONAV_MM_MISSION_CREATE_WAYPOINT.RequestType()
        req.name = f"Waypoint_{waypoint_lat}_{waypoint_lon}"
        req.latitude = waypoint_lat
        req.longitude = waypoint_lon
        req.heading = waypoint_heading # Adjust heading as needed
        req.position_tolerance = 1.0
        req.yaw_tolerance = 5.0
        if uuid != "":
            req.task_ids = [uuid]
        res = ONAV_MM_MISSION_CREATE_WAYPOINT.call(req)
        if res.result:
            waypoint_uuids.append(res.result.uuid)
        counter +=1
    return waypoint_uuids


def main():
    m = get_map(MAP_NAME)
    if not m:
        print(f"Map {MAP_NAME} not found!")
        return

    map_edges, map_polygon = get_polygon(m)
    if not map_edges or not map_polygon:
        print(f"Map has no valid edges!")
        return

    lat1, lon1 = m.points[0].latitude, m.points[0].longitude
    lat2, lon2 = m.points[1].latitude, m.points[1].longitude
    heading_deg = calc_bearing(lat1, lon1, lat2, lon2)
    heading_enu = ned_to_enu(heading_deg)
    heading_rad = math.radians(heading_enu)

    uuid1 = create_ptz_task(pan1, pos1)
    uuid2 = create_ptz_task(pan2, pos2)


    waypoint_uuids = generate_waypoints(map_edges, heading_rad, uuid1, uuid2)
    if not waypoint_uuids:
        rospy.logerr("No waypoints created.")
        return

    mission_uuid = create_mission(MISSION_NAME, waypoint_uuids)
    if mission_uuid:
        rospy.loginfo(f"Mission created successfully with UUID: {mission_uuid}")
    else:
        rospy.logerr("Mission creation failed.")

if __name__ == "__main__":
    main()

