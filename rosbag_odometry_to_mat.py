#!/usr/bin/env python

# to ignore numpy errors
# pylint: disable=e1101
# to ignore to long lines
# pylint: disable=c0301
# to ignore rosbag import false positives
# pylint: disable=f0401

import argparse
import numpy as np
import os
import pyproj
import rosbag
import scipy.io as sio

# from nmea_navsat_driver.msg import NavSatInfo
# from NavSatInfo.msg import num_satellites
# from NavSatInfo.msg import last_dgps_update
# from NavSatInfo.msg import hdop

def append_raw_imu_msg(msg_in, list_in_out):
    list_in_out.append([msg_in.header.stamp.to_sec(), msg_in.linear_acceleration.x, msg_in.linear_acceleration.y, msg_in.linear_acceleration.z, msg_in.angular_velocity.x, msg_in.angular_velocity.y, msg_in.angular_velocity.z])

def append_mag_msg(msg_in, list_in_out):
    list_in_out.append([msg_in.header.stamp.to_sec(), msg_in.magnetic_field.x, msg_in.magnetic_field.y, msg_in.magnetic_field.z])
     
def append_imu_msg(msg_in, list_in_out):
    list_in_out.append([msg_in.header.stamp.to_sec(), msg_in.orientation.x, msg_in.orientation.y, msg_in.orientation.z, msg_in.orientation.w])
## User settings
# topic names (incase of change)
Odom_tn = '/robot/odom'

Xsens_mag_tn = '/sensor/imu/xsens_mti/magfield'#[MagneticField]
Xsens_imu_tn = '/sensor/imu/xsens_mti/data'    #[Imu]
# Xsens_imu_raw_tn = '/sensor/imu/xsens_mti/raw' #[Imu]
Um7_imu_tn = '/sensor/imu/um7/data'            #[Imu]
Um7_mag_tn = '/sensor/imu/um7/magfield_msg'    #[MagneticField]
Razor_mag_tn = ""
Razor_imu_tn = ""

Trimble_fix_tn = '/sensor/gps/trimble/fix'     #[NavSatFix]
Trimble_vel_tn = '/sensor/gps/trimble/vel'     #[TwistStamped]
Trimble_orientation_tn = "/sensor/gps/trimble/orientation" #[nmea_navsat_driver/NavSatOrientation]

Piksi_fix_tn = "/sensor/gps/piksi_1/gps/fix"  # [NavSatFix]
Piksi_rtkfix_tn = "/sensor/gps/piksi_1/gps/rtkfix" #[nav_msgs/Odometry]


# diagnostics_tn = 

read_topic_list = [
        Odom_tn, 
        Xsens_mag_tn,
        Xsens_imu_tn,
        Trimble_fix_tn,
        Trimble_vel_tn,
        Trimble_orientation_tn]
#         '/gps/fix',
#         '/imu/mag',
#         '/imu/rpy',
#         '/sensor/gps/leica1200/vel',
#         '/sensor/gps/leica1200/time_reference',
#         '/sensor/gps/evk5h/fix',
#         '/sensor/gps/evk7ppp/fix',
#         '/enu',
#         '/sensor/gps/leica1200/fix',
#         '/sensor/gps/leica1200/vel',
#         '/sensor/gps/leica1200/info']

proj = pyproj.Proj(proj='utm', zone=32, ellps='WGS84')

# parsing input arguments
parser = argparse.ArgumentParser(description='Convertig bag odometry, GPS and compass data to matlab data')
parser.add_argument('input_bag', help='Rosbag to extract from')
parser.add_argument('output_directory', help='Directory to extract to')
args = parser.parse_args()
## legacy 
# var init
e_offset = 0
n_offset = 0
u_offset = 0

list_odom = []

list_gps = []
list_gps_rtklib = []
list_gps_rtk = []
list_gps_rtk_vel = []
list_gps_rtk_info = []

# Pkisi
list_gps_piksi = []
list_rtk_gps_piksi = []
# Trimble
list_gps_trimble = []
list_gps_trimble_vel = []
list_gps_trimble_orientation = []
# um_7
list_um_7_mag = []
list_um_7_imu = []
list_um7_acc_gyro = []
# Xsens
list_xsens_orientation = []
list_xsens_mag = []
list_xsens_acc_gyro = []

# Razor
list_Razor_mag = []
list_Razor_imu = []
list_Razor_acc_gyro = []

# CHRobotics GPS + IMU
list_chr_gps = []
list_chr_mag = []
list_chr_rpy = []


# extract data
with rosbag.Bag(args.input_bag, 'r') as input_file:
    # for topic, msg, t in input_file:
    for topic, msg, t in input_file.read_messages(topics = read_topic_list):
        # odometry
        if Odom_tn == topic:
            list_odom.append([msg.header.stamp.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
#Xsens
        # magfield from xsens
        if Xsens_mag_tn == topic:
            append_mag_msg(msg, list_xsens_mag)
        # orientation from xsens
        if Xsens_imu_tn == topic:
            append_imu_msg(msg, list_xsens_orientation)
            append_raw_imu_msg(msg, list_xsens_acc_gyro)

#Um7
        # magfield from Um7
        if Um7_mag_tn == topic:
            append_mag_msg(msg, list_um_7_mag)
        # orientation from Um7
        if Um7_imu_tn == topic:
            append_imu_msg(msg, list_um_7_imu)
            append_raw_imu_msg(msg, list_um7_acc_gyro)

#Razor
        # magfield from Razor
        if Um7_mag_tn == topic:
            append_mag_msg(msg, list_Razor_mag)
        # orientation from Razor
        if Um7_imu_tn == topic:
            append_imu_msg(msg, list_Razor_imu)
            append_raw_imu_msg(msg, list_Razor_acc_gyro)
            

# gps from u-blox
        if topic in ('/sensor/gps/evk5h/fix', '/sensor/gps/evk7ppp/fix'):
            e, n = proj(msg.longitude, msg.latitude)

            if e_offset == 0:
                e_offset = e
                n_offset = n
                u_offset = msg.altitude

            e = e - e_offset
            n = n - n_offset
            u = msg.altitude - u_offset

            list_gps.append([msg.header.stamp.to_sec(), msg.latitude, msg.longitude, msg.altitude, e, n, u])

# gps from RTKLIB
        if '/enu' == topic:
            e, n = proj(msg.pose.position.x, msg.pose.position.y)

            if e_offset == 0:
                e_offset = e
                n_offset = n
                u_offset = msg.pose.position.z

            e = e - e_offset
            n = n - n_offset
            u = msg.pose.position.z - u_offset

            list_gps_rtklib.append([msg.header.stamp.to_sec(), msg.pose.position.y, msg.pose.position.x, msg.pose.position.z, e, n, u])

# gps from leica
        if '/sensor/gps/leica1200/fix' == topic:
            if np.isfinite(msg.longitude) and np.isfinite(msg.latitude):
                e, n = proj(msg.longitude, msg.latitude)

                if e_offset == 0:
                    e_offset = e
                    n_offset = n
                    u_offset = msg.altitude

                e = e - e_offset
                n = n - n_offset
                u = msg.altitude - u_offset

                list_gps_rtk.append([msg.header.stamp.to_sec(), msg.latitude, msg.longitude, msg.altitude, e, n, u, msg.status.status, np.sqrt(msg.position_covariance[0])])

        # additional data from leica
        if '/sensor/gps/leica1200/vel' == topic:
            velx = msg.twist.linear.x
            vely = msg.twist.linear.y

            angle = np.arctan2(vely, velx)
            vel = np.sqrt(velx * velx + vely * vely)

            list_gps_rtk_vel.append([msg.header.stamp.to_sec(), vel, angle])

        # additional data from leica
        if '/sensor/gps/leica1200/info' == topic:
            pass
            # list_gps_rtk_info.append([msg.header.stamp.to_sec(), msg.num_satellites, msg.last_dgps_update, msg.hdop])

# gps from trimble
        if Trimble_fix_tn == topic:
            if np.isfinite(msg.longitude) and np.isfinite(msg.latitude):
                e, n = proj(msg.longitude, msg.latitude)

                if e_offset == 0:
                    e_offset = e
                    n_offset = n
                    u_offset = msg.altitude

                e = e - e_offset
                n = n - n_offset
                u = msg.altitude - u_offset

                list_gps_trimble.append([msg.header.stamp.to_sec(), msg.latitude, msg.longitude, msg.altitude, e, n, u, msg.status.status, np.sqrt(msg.position_covariance[0])])

        # additional data from trimble
        if Trimble_vel_tn == topic:
            velx = msg.twist.linear.x
            vely = msg.twist.linear.y

            angle = np.arctan2(vely, velx)
            vel = np.sqrt(velx * velx + vely * vely)

            list_gps_trimble_vel.append([msg.header.stamp.to_sec(), vel, angle])
        # Trimble orientation
        if Trimble_orientation_tn == topic:
            print "adding Trimble orientation data"
            

# ch robotics gp9 gps
        if '/gps/fix' == topic:
            e, n = proj(msg.longitude, msg.latitude)

            if e_offset == 0:
                e_offset = e
                n_offset = n
                u_offset = msg.altitude

            e = e - e_offset
            n = n - n_offset
            u = msg.altitude - u_offset

            list_chr_gps.append([msg.header.stamp.to_sec(), msg.latitude, msg.longitude, msg.altitude, e, n, u, msg.status.status, np.sqrt(msg.position_covariance[0])])

# ch robotics gp9 magnetometer
        if '/imu/mag' == topic:
            list_chr_mag.append([msg.header.stamp.to_sec(), msg.vector.x, msg.vector.y, msg.vector.z])

        # ch robotics gp9 imu
        if '/imu/rpy' == topic:
            list_chr_rpy.append([msg.header.stamp.to_sec(), msg.vector.x, msg.vector.y, msg.vector.z])

        if '/sensor/gps/leica1200/vel' == topic:
            pass

        if '/sensor/gps/leica1200/time_reference' == topic:
            pass

# convert to numpy arrays
# odom
array_odom = np.zeros((len([i[0] for i in list_odom]), 4))
array_odom[:, :] = np.transpose([[i[0] for i in list_odom], [i[1] for i in list_odom], [i[2] for i in list_odom], [i[3] for i in list_odom]])
odom_headers = ['odom', 'time', 'x', 'y', 'z']

array_magfield = np.zeros((len([i[0] for i in list_xsens_mag]), 4))
array_magfield[:, :] = np.transpose([[i[0] for i in list_xsens_mag], [i[1] for i in list_xsens_mag], [i[2] for i in list_xsens_mag], [i[3] for i in list_xsens_mag]])
magfield_headers = ['magfiled', 'time', 'x', 'y', 'z']

array_xsens_orientation = np.zeros((len([i[0] for i in list_xsens_orientation]), 5))
array_xsens_orientation[:, :] = np.transpose([[i[0] for i in list_xsens_orientation], [i[1] for i in list_xsens_orientation], [i[2] for i in list_xsens_orientation], [i[3] for i in list_xsens_orientation], [i[4] for i in list_xsens_orientation]])
xsens_orientation_headers = ['xsens_orientation', 'time', 'x', 'y', 'z', 'w']

array_gps = np.zeros((len([i[0] for i in list_gps]), 7))
array_gps[:, :] = np.transpose([[i[0] for i in list_gps], [i[1] for i in list_gps], [i[2] for i in list_gps], [i[3] for i in list_gps], [i[4] for i in list_gps], [i[5] for i in list_gps], [i[6] for i in list_gps]])
gps_headers = ['gps', 'time', 'latitude', 'longitude', 'altitude', 'east', 'north', 'up']

array_gps_rtklib = np.zeros((len([i[0] for i in list_gps_rtklib]), 7))
array_gps_rtklib[:, :] = np.transpose([[i[0] for i in list_gps_rtklib], [i[1] for i in list_gps_rtklib], [i[2] for i in list_gps_rtklib], [i[3] for i in list_gps_rtklib], [i[4] for i in list_gps_rtklib], [i[5] for i in list_gps_rtklib], [i[6] for i in list_gps_rtklib]])
gps_rtklib_headers = ['gps_rtklib', 'time', 'latitude', 'longitude', 'altitude', 'east', 'north', 'up']

array_gps_rtk = np.zeros((len([i[0] for i in list_gps_rtk]), 9))
array_gps_rtk[:, :] = np.transpose([[i[0] for i in list_gps_rtk], [i[1] for i in list_gps_rtk], [i[2] for i in list_gps_rtk], [i[3] for i in list_gps_rtk], [i[4] for i in list_gps_rtk], [i[5] for i in list_gps_rtk], [i[6] for i in list_gps_rtk], [i[7] for i in list_gps_rtk], [i[8] for i in list_gps_rtk]])
gps_rtk_headers = ['gps_rtk', 'time', 'latitude', 'longitude', 'altitude', 'east', 'north', 'up', 'status', 'hdop']

array_gps_rtk_vel = np.zeros((len([i[0] for i in list_gps_rtk_vel]), 3))
array_gps_rtk_vel[:, :] = np.transpose([[i[0] for i in list_gps_rtk_vel], [i[1] for i in list_gps_rtk_vel], [i[2] for i in list_gps_rtk_vel]])
gps_rtk_vel_headers = ['gps_rtk_vel', 'time', 'velocity', 'heading']

# array_gps_rtk_info = np.zeros((len([i[0] for i in list_gps_rtk]), 4))
# array_gps_rtk_info[:, :] = np.transpose([[i[0] for i in list_gps_rtk_info], [i[1] for i in list_gps_rtk_info], [i[2] for i in list_gps_rtk_info], [i[3] for i in list_gps_rtk_info]])
# gps_rtk_info_headers = ['gps_rtk_info', 'time', 'num_satellites', 'last_dgps_update', 'hdop']

array_gps_trimble = np.zeros((len([i[0] for i in list_gps_trimble]), 9))
array_gps_trimble[:, :] = np.transpose([[i[0] for i in list_gps_trimble], [i[1] for i in list_gps_trimble], [i[2] for i in list_gps_trimble], [i[3] for i in list_gps_trimble], [i[4] for i in list_gps_trimble], [i[5] for i in list_gps_trimble], [i[6] for i in list_gps_trimble], [i[7] for i in list_gps_trimble], [i[8] for i in list_gps_trimble]])
gps_trimble_headers = ['gps_trimble', 'time', 'latitude', 'longitude', 'altitude', 'east', 'north', 'up', 'status', 'hdop']

array_gps_trimble_vel = np.zeros((len([i[0] for i in list_gps_trimble_vel]), 3))
array_gps_trimble_vel[:, :] = np.transpose([[i[0] for i in list_gps_trimble_vel], [i[1] for i in list_gps_trimble_vel], [i[2] for i in list_gps_trimble_vel]])
gps_trimble_vel_headers = ['gps_trimble_vel', 'time', 'velocity', 'heading']

array_chr_gps = np.zeros((len([i[0] for i in list_chr_gps]), 9))
array_chr_gps[:, :] = np.transpose([[i[0] for i in list_chr_gps], [i[1] for i in list_chr_gps], [i[2] for i in list_chr_gps], [i[3] for i in list_chr_gps], [i[4] for i in list_chr_gps], [i[5] for i in list_chr_gps], [i[6] for i in list_chr_gps], [i[7] for i in list_chr_gps], [i[8] for i in list_chr_gps]])
chr_gps_headers = ['gps_chr', 'time', 'latitude', 'longitude', 'altitude', 'east', 'north', 'up', 'status', 'hdop']

array_chr_mag = np.zeros((len([i[0] for i in list_chr_mag]), 4))
array_chr_mag[:, :] = np.transpose([[i[0] for i in list_chr_mag], [i[1] for i in list_chr_mag], [i[2] for i in list_chr_mag], [i[3] for i in list_chr_mag]])
chr_mag_headers = ['magfiled', 'time', 'x', 'y', 'z']

array_chr_rpy = np.zeros((len([i[0] for i in list_chr_rpy]), 4))
array_chr_rpy[:, :] = np.transpose([[i[0] for i in list_chr_rpy], [i[1] for i in list_chr_rpy], [i[2] for i in list_chr_rpy], [i[3] for i in list_chr_rpy]])
chr_rpy_headers = ['roll_pitch_yaw', 'time', 'r', 'p', 'y']

headers = [odom_headers, magfield_headers, xsens_orientation_headers, gps_headers, gps_rtklib_headers, gps_rtk_headers, gps_rtk_vel_headers, gps_trimble_headers, gps_trimble_vel_headers, chr_gps_headers, chr_mag_headers, chr_rpy_headers]

# save as matlab file
if args.output_directory.endswith('/'):
    args.output_directory = args.output_directory[:-1]
print args.output_directory + '/' + os.path.basename(args.input_bag)[:-4] + '.mat'

sio.savemat(args.output_directory + '/' + os.path.basename(args.input_bag)[:-4] + '.mat', {'odom':array_odom, 'magfield':array_magfield, 'xsens_orientation':array_xsens_orientation, 'gps':array_gps, 'gps_rtklib':array_gps_rtklib, 'gps_rtk':array_gps_rtk, 'gps_rtk_vel':array_gps_rtk_vel, 'gps_trimble':array_gps_trimble, 'gps_trimble_vel':array_gps_trimble_vel, 'chr_gps':array_chr_gps, 'chr_mag':array_chr_mag, 'chr_rpy':array_chr_rpy, 'headers':headers})
print "Finished press ctrl+C to exit or any key to continue"
raw_input("print_mat ?")
mat = sio.loadmat(args.output_directory + '/' + os.path.basename(args.input_bag)[:-4] + '.mat')
for entry in ["odom","xsens_orientation"]:
    print entry
    print mat.get(entry)
print list_xsens_acc_gyro