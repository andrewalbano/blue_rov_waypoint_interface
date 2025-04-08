#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler,quaternion_matrix, translation_matrix, euler_matrix, concatenate_matrices, quaternion_from_matrix, translation_from_matrix, euler_from_quaternion
from tkinter import Tk, Label, Entry, Button, LabelFrame, messagebox
from nav_msgs.msg import Path
import utm

'''
last_waypoint_transform = np.eye(4)
print(last_waypoint_transform)

# move 1 in x
x,y,z = 1, 0, 0 
roll, pitch, yaw = 0,0,0
trans_matrix = translation_matrix([x,y,z])
rot_matrix = euler_matrix(roll,pitch,yaw)
print(rot_matrix)
transform = concatenate_matrices(rot_matrix,trans_matrix)
print(transform)

print("\nNext transform\n")
x,y,z = 0, 0, 0 
roll, pitch, yaw = 0,0,np.pi/2
trans_matrix = translation_matrix([x,y,z])
rot_matrix = euler_matrix(roll,pitch,yaw)
print(rot_matrix)
transform = concatenate_matrices(transform, rot_matrix,trans_matrix)

# transform = concatenate_matrices(rot_matrix,trans_matrix)
print(transform)


print("\nNext transform\n")
x,y,z = 1,0, 0 
roll, pitch, yaw = 0,0,0
trans_matrix = translation_matrix([x,y,z])
rot_matrix = euler_matrix(roll,pitch,yaw)
print(rot_matrix)
transform = concatenate_matrices(transform, rot_matrix,trans_matrix)
print(transform)



x,y,z = 1, 0, 0 
roll, pitch, yaw = 0,0, np.pi/2
print(yaw)
q = quaternion_from_euler(roll,pitch,yaw)
roll,pitch,yaw = euler_from_quaternion(q) 

print(yaw)



desired_path = Path()
desired_path.header.frame_id='NED'

# desired_path.poses.append(self.init_pose)
if not desired_path.poses:
    print("empty")

    import numpy as np



'''
def wgs84_to_ecef(lat, lon, h):
    # fossen pg 36 table 2.2
    # WGS84 ellipsoid constants
    re = 6378137    # semi-major axis (meters)
    rp = 6356752  # semi-minor axis (meters)

    # conversion to radians
    lat = np.radians(lat)
    lon = np.radians(lon)
    
    # fossen pg 36 eq 2.88
    # radius of curvature of the prime vertical
    N = (re**2) / np.sqrt((re**2)* (np.cos(lat)**2) + (rp**2)* (np.sin(lat)**2))

    # fossen pg 38 eq 2.93
    X = (N + h) * np.cos(lat) * np.cos(lon)
    Y = (N + h) * np.cos(lat) * np.sin(lon)
    
    Z = ((rp**2)*N/(re**2)+ h) * np.sin(lat)
    

    return X, Y, Z

def ecef_to_enu(x, y, z, lat_ref, lon_ref, alt_ref):
    # Convert reference position to ECEF
    x_ref, y_ref, z_ref = wgs84_to_ecef(lat_ref, lon_ref, alt_ref)

    # Translation to the reference point
    dx = x - x_ref
    dy = y - y_ref
    dz = z - z_ref

    # Reference latitude and longitude in radians
    lat_ref = np.radians(lat_ref)
    lon_ref = np.radians(lon_ref)

    # Create the rotation matrix
    R = np.array([
        [-np.sin(lon_ref),  np.cos(lon_ref), 0],
        [-np.sin(lat_ref) * np.cos(lon_ref), -np.sin(lat_ref) * np.sin(lon_ref), np.cos(lat_ref)],
        [ np.cos(lat_ref) * np.cos(lon_ref),  np.cos(lat_ref) * np.sin(lon_ref), np.sin(lat_ref)]
    ])

    # Apply the rotation
    enu = R.dot(np.array([dx, dy, dz]))

    return enu

def wgs84_to_enu(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    x, y, z = wgs84_to_ecef(lat, lon, alt)
    enu = ecef_to_enu(x, y, z, lat_ref, lon_ref, alt_ref)
    return enu

# Example usage:
lat, lon, alt = 42.2808, -83.7430, 270  # Ann Arbor, MI
lat_ref, lon_ref, alt_ref = 42.281, -83.742, 260  # Reference point origin of ned frame


enu_coords = wgs84_to_enu(lat, lon, alt, lat_ref, lon_ref, alt_ref)
print("ENU Coordinates:", enu_coords)



lon = 10.3 
lat = 63
h = 0

ecef_coords  = wgs84_to_ecef(lat, lon, h)

print(f"ECEF COORDS: {ecef_coords}")


x, y,_,_ = utm.from_latlon(51.2, 7.5)
print(x)


