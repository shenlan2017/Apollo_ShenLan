#!/usr/bin/env python3

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

import argparse
import sys
import os

from cyber.python.cyber_py3.record import RecordReader

from modules.canbus.proto import chassis_pb2

from modules.drivers.proto import pointcloud_pb2
from modules.drivers.proto import conti_radar_pb2
from modules.drivers.proto import sensor_image_pb2

from modules.localization.proto import localization_pb2
from modules.localization.proto import gps_pb2

from modules.drivers.gnss.proto import ins_pb2
from modules.drivers.gnss.proto import imu_pb2
from modules.drivers.gnss.proto import heading_pb2
from modules.drivers.gnss.proto import gnss_best_pose_pb2

# from modules.perception.proto import perception_obstacle_pb2
# from modules.planning.proto import planning_pb2
# from modules.control.proto import control_cmd_pb2


def create_folder(filename):
    filename = filename.strip()
    filename = filename.rstrip("\\")
    is_exists = os.path.exists(filename)

    if not is_exists:
        os.makedirs(filename)
        print("[INFO] Build " + filename + " success!")
        return  True
    else:
        print("[INFO] Folder already exists.")
        return False


def parse_data(channelname, msg, out_folder):
    """
    """
    msg_lidar = pointcloud_pb2.PointCloud()
    msg_lidar.ParseFromString(msg)
    nPts = len(msg_lidar.point)

    pcd = []
    for j in range(nPts):
        p = msg_lidar.point[j]
        pcd.append([p.x, p.y, p.z, p.intensity])

    tstamp = msg_lidar.measurement_time
    temp_time = str(tstamp).split('.')

    if len(temp_time[1]) == 1:
        temp_time1_adj = temp_time[1] + '0'
    else:
        temp_time1_adj = temp_time[1]

    pcd_time = temp_time[0] + '.' + temp_time1_adj
    pcd_filename = pcd_time + ".txt"

    with open(out_folder + pcd_filename, 'w') as outfile:
        for item in pcd:
            data = str(item)[1:-1]
            outfile.write("%s\n" % data)

    return tstamp

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Recode Analyzer is a tool to analyze record files.",
        prog="main.py")

    parser.add_argument(
        "-f", "--file", action="store", type=str, required=True,
        help="Specify the record file for message dumping.")

    parser.add_argument(
        "-o", "--output", action="store", type=str, required=True,
        help="which folder to output.")

    args = parser.parse_args()

    record_file = args.file
    reader = RecordReader(record_file)
    create_folder(args.output)
    create_folder(args.output + "/localization")
    create_folder(args.output + "/radar_front")
    create_folder(args.output + "/chassis")
    create_folder(args.output + "/ins_stat")
    create_folder(args.output + "/imu")
    create_folder(args.output + "/heading")
    create_folder(args.output + "/best_pose")
    create_folder(args.output + "/rtk_pose")
    create_folder(args.output + "/lidar_pcd")

    for msg in reader.read_messages():
        timestamp = msg.timestamp / float(1e9)

        if msg.topic == "/apollo/localization/pose":
            localization = localization_pb2.LocalizationEstimate()
            localization.ParseFromString(msg.message)
            file_name = args.output + "/localization/" + str(timestamp) + ".txt"
            with open(file_name, 'w') as f:
                ts = localization.header.timestamp_sec
                mt = localization.measurement_time
                px = localization.pose.position.x
                py = localization.pose.position.y
                pz = localization.pose.position.z
                qx = localization.pose.orientation.qx
                qy = localization.pose.orientation.qy
                qz = localization.pose.orientation.qz
                qw = localization.pose.orientation.qw
                vx = localization.pose.linear_velocity.x
                vy = localization.pose.linear_velocity.y
                vz = localization.pose.linear_velocity.z
                gx = localization.pose.linear_acceleration.x
                gy = localization.pose.linear_acceleration.y
                gz = localization.pose.linear_acceleration.z
                ax = localization.pose.angular_velocity.x
                ay = localization.pose.angular_velocity.y
                az = localization.pose.angular_velocity.z
                f.write("# Simplify Information: \n")
                f.write("# timestamp_sec measurement_time position.x position.y position.z orientation.qx orientation.qy orientation.qz orientation.qw velocity.x velocity.y velocity.z linear_acceleration.x linear_acceleration.y linear_acceleration.z angular_velocity.x angular_velocity.y angular_velocity.z \n")
                f.write((' ').join(str(x) for x in [ts, mt, px, py, pz, qx, qy, qz, qw, vx, vy, vz, gx, gy, gz, ax, ay, az]))
                f.write("\n\n# Complete Information\n")
                f.write(str(localization))
            continue

        if msg.topic == "/apollo/sensor/radar/front" :
            radar = conti_radar_pb2.ContiRadar()
            radar.ParseFromString(msg.message)
            file_name = args.output + "/radar_front/" + str(timestamp) + ".txt"
            with open(file_name, 'w') as f:
                f.write(str(radar))
            continue

        if msg.topic == "/apollo/canbus/chassis":
            chassis = chassis_pb2.Chassis()
            chassis.ParseFromString(msg.message)
            file_name = args.output + "/chassis/" + str(timestamp) + ".txt"
            with open(file_name, 'w') as f:
                ts = chassis.header.timestamp_sec
                rr = chassis.wheel_speed.wheel_spd_rr
                rl = chassis.wheel_speed.wheel_spd_rl
                fr = chassis.wheel_speed.wheel_spd_fr
                fl = chassis.wheel_speed.wheel_spd_fl
                f.write("# Simplify Information: \n")
                f.write("# timestamp_sec wheel_spd_rr wheel_spd_rl wheel_spd_fr wheel_spd_fl\n")
                f.write((' ').join(str(x) for x in [ts, rr, rl, fr, fl]))
                f.write("\n\n# Complete Information\n")
                f.write(str(chassis))
            continue

        if msg.topic == "/apollo/sensor/gnss/ins_stat" :
            ins_status = ins_pb2.InsStat()
            ins_status.ParseFromString(msg.message)
            file_name = args.output + "/ins_stat/" + str(timestamp) + ".txt"
            with open(file_name, 'w') as f:
                f.write(str(ins_status))
            continue

        if msg.topic == "/apollo/sensor/gnss/imu" :
            imu = imu_pb2.Imu()
            imu.ParseFromString(msg.message)
            file_name = args.output + "/imu/" + str(timestamp) + ".txt"
            with open(file_name, 'w') as f:
                ts = imu.header.timestamp_sec
                mt = imu.measurement_time
                gx = imu.linear_acceleration.x
                gy = imu.linear_acceleration.y
                gz = imu.linear_acceleration.z
                ax = imu.angular_velocity.x
                ay = imu.angular_velocity.y
                az = imu.angular_velocity.z
                f.write("# Simplify Information: \n")
                f.write("# timestamp_sec measurement_time linear_acceleration.x linear_acceleration.y linear_acceleration.z angular_velocity.x angular_velocity.y angular_velocity.z\n")
                f.write((' ').join(str(x) for x in [ts, mt, gx, gy, gz, ax, ay, az]))
                f.write("\n\n# Complete Information\n")
                f.write(str(imu))
            continue

        if msg.topic == "/apollo/sensor/gnss/heading" :
            heading = heading_pb2.Heading()
            heading.ParseFromString(msg.message)
            file_name = args.output + "/heading/" + str(timestamp) + ".txt"
            with open(file_name, 'w') as f:
                ts = heading.measurement_time
                hds = heading.heading
                hdv = heading.heading_std_dev
                phs = heading.pitch
                phv = heading.pitch_std_dev
                # 移动站与基准站的距离
                bl = heading.baseline_length
                f.write("# Simplify Information: \n")
                f.write("# measurement_time heading heading_std_dev pitch pitch_std_dev baseline_length\n")
                f.write((' ').join(str(x) for x in [ts, hds, hdv, phs, phv, bl]))
                f.write("\n\n# Complete Information\n")
                f.write(str(heading))
            continue

        if msg.topic == "/apollo/sensor/gnss/best_pose" :
            best_pose = gnss_best_pose_pb2.GnssBestPose()
            best_pose.ParseFromString(msg.message)
            file_name = args.output + "/best_pose/" + str(timestamp) + ".txt"
            with open(file_name, 'w') as f:
                ts = best_pose.header.timestamp_sec
                mt = best_pose.measurement_time
                lat = best_pose.latitude
                lon = best_pose.longitude
                hei = best_pose.height_msl
                lat_v = best_pose.latitude_std_dev
                lon_v = best_pose.longitude_std_dev
                hei_v = best_pose.height_std_dev
                f.write("# Simplify Information: \n")
                f.write("# timestamp_sec measurement_time latitude longitude height_msl latitude_std_dev longitude_std_dev height_std_dev \n")
                f.write((' ').join(str(x) for x in [ts, mt, lat, lon, hei, lat_v, lon_v, hei_v]))
                f.write("\n\n# Complete Information\n")
                f.write(str(best_pose))
            continue

        if msg.topic == "/apollo/sensor/gnss/odometry" :
            rtk_pose = gps_pb2.Gps()
            rtk_pose.ParseFromString(msg.message)
            file_name = args.output + "/rtk_pose/" + str(timestamp) + ".txt"
            with open(file_name, 'w') as f:
                ts = rtk_pose.header.timestamp_sec
                px = rtk_pose.localization.position.x
                py = rtk_pose.localization.position.y
                pz = rtk_pose.localization.position.z
                qx = rtk_pose.localization.orientation.qx
                qy = rtk_pose.localization.orientation.qy
                qz = rtk_pose.localization.orientation.qz
                qw = rtk_pose.localization.orientation.qw
                vx = rtk_pose.localization.linear_velocity.x
                vy = rtk_pose.localization.linear_velocity.y
                vz = rtk_pose.localization.linear_velocity.z
                f.write("# Simplify Information: \n")
                f.write("# timestamp_sec position.x position.y position.z orientation.qx orientation.qy orientation.qz orientation.qw velocity.x velocity.y velocity.z\n")
                f.write((' ').join(str(x) for x in [ts, px, py, pz, qx, qy, qz, qw, vx, vy, vz]))
                f.write("\n\n# Complete Information\n")
                f.write(str(rtk_pose))
            continue

        if msg.topic == "/apollo/sensor/lidar128/compensator/PointCloud2" or \
           msg.topic == "/apollo/sensor/lidar16/compensator/PointCloud2" or \
           msg.topic == "/apollo/sensor/lidar32/compensator/PointCloud2":
            file_name = args.output + "/lidar_pcd/"
            parse_data(msg.topic, msg.message, file_name)
            continue
