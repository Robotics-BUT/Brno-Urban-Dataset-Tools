import os
import csv
from tqdm import tqdm
import pypcd

import rospy
import rosbag

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import PoseStamped

from custom_msgs.msg import TimeRef

from utils import constants
from utils import folders
from utils import frames
from utils import topics


class BagBuilder:

    def __init__(self):

        self.bag_ext = '.bag'
        self.cameras = constants.Cameras()
        self.folders = folders.Folders()
        self.frames = frames.Frames()
        self.topics = topics.Topics()

    def build_bag(self, folder, bag_name):

        bag_file = folder + bag_name + self.bag_ext
        compressions = ['none', 'bz2', 'lz4']
        print('Building: ', bag_file)
        with rosbag.Bag(bag_file, 'w', compression=compressions[2]) as bag:

            self._build_gnss_data(bag, folder + bag_name + '/' + self.folders.gnss)

            cameras = [
                [self.topics.camera_left_front, self.folders.camera_left_front, self.frames.camera_left_front],
                [self.topics.camera_left_side, self.folders.camera_left_side, self.frames.camera_left_side],
                [self.topics.camera_right_front, self.folders.camera_right_front, self.frames.camera_right_front],
                [self.topics.camera_right_side, self.folders.camera_right_side, self.frames.camera_right_side]
            ]

            for camera in cameras:
                self._build_camera_data(bag, folder + bag_name + '/' + camera[1], camera[0], camera[2])

            self._build_ir_camera_data(bag, folder + bag_name + '/' + self.folders.camera_ir,
                                       self.topics.camera_ir, self.frames.camera_ir)

            self._build_imu_data(bag, folder + bag_name + '/' + self.folders.imu)

            lidars = [
                (self.topics.lidar_left, self.folders.lidar_left, self.frames.lidar_left),
                (self.topics.lidar_right, self.folders.lidar_right, self.frames.lidar_right),
            ]

            for lidar in lidars:
                self._build_lidar_data(bag, folder + bag_name + '/' + lidar[1], lidar[0], lidar[2])

            print('Done')

    def _build_imu_data(self, bag, folder):

        print('Building: ' + self.topics.xsens_imu)
        with open(folder + 'imu.txt') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            seq = 0
            for row in tqdm(csv_reader):
                msg = Imu()

                t = max(0, int(row[0]))
                time = rospy.Time(0)
                time.secs = int(t / 1e9)
                time.nsecs = int(t) - int(time.secs * 1e9)

                msg.header.seq = seq
                seq += 1

                msg.header.stamp = time
                msg.header.frame_id = self.frames.xsens

                msg.linear_acceleration.x = float(row[1])
                msg.linear_acceleration.y = float(row[2])
                msg.linear_acceleration.z = float(row[3])

                msg.angular_velocity.x = float(row[4])
                msg.angular_velocity.y = float(row[5])
                msg.angular_velocity.z = float(row[6])

                msg.orientation.x = float(row[7])
                msg.orientation.y = float(row[8])
                msg.orientation.z = float(row[9])
                msg.orientation.w = float(row[10])

                bag.write(self.topics.xsens_imu, msg, time)

        print('Building: ' + self.topics.xsens_mag)
        with open(folder + 'mag.txt') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            seq = 0
            for row in tqdm(csv_reader):
                msg = MagneticField()

                t = max(0, int(row[0]))
                time = rospy.Time(0)
                time.secs = int(t / 1e9)
                time.nsecs = int(t) - int(time.secs * 1e9)

                msg.header.seq = seq
                seq += 1

                msg.header.stamp = time
                msg.header.frame_id = self.frames.xsens

                msg.magnetic_field.x = float(row[1])
                msg.magnetic_field.y = float(row[2])
                msg.magnetic_field.z = float(row[3])

                bag.write(self.topics.xsens_mag, msg, time)

        print('Building: ' + self.topics.xsens_gnss)
        with open(folder + 'gnss.txt') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            seq = 0
            for row in tqdm(csv_reader):

                msg = NavSatFix()

                t = max(0, int(row[0]))
                time = rospy.Time(0)
                time.secs = int(t / 1e9)
                time.nsecs = int(t) - int(time.secs * 1e9)

                msg.header.seq = seq
                seq += 1

                msg.header.stamp = time
                msg.header.frame_id = self.frames.xsens

                try:
                    msg.latitude = float(row[1])
                    msg.longitude = float(row[2])
                    msg.altitude = float(row[3])
                except:
                    print('Unable to read: ' + row)

                bag.write(self.topics.xsens_gnss, msg, time)

        print('Building: ' + self.topics.xsens_dela_q)
        with open(folder + 'd_quat.txt') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            seq = 0
            for row in tqdm(csv_reader):
                msg = QuaternionStamped()

                t = max(0, int(row[0]))
                time = rospy.Time(0)
                time.secs = int(t / 1e9)
                time.nsecs = int(t) - int(time.secs * 1e9)

                msg.header.seq = seq
                seq += 1

                msg.header.stamp = time
                msg.header.frame_id = self.frames.xsens

                msg.quaternion.x = float(row[1])
                msg.quaternion.y = float(row[2])
                msg.quaternion.z = float(row[3])
                msg.quaternion.w = float(row[3])

                bag.write(self.topics.xsens_dela_q, msg, time)

        print('Building: ' + self.topics.xsens_pressure)
        with open(folder + 'pressure.txt') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            seq = 0
            for row in tqdm(csv_reader):
                msg = FluidPressure()

                t = max(0, int(row[0]))
                time = rospy.Time(0)
                time.secs = int(t / 1e9)
                time.nsecs = int(t) - int(time.secs * 1e9)

                msg.header.seq = seq
                seq += 1

                msg.header.stamp = time
                msg.header.frame_id = self.frames.xsens

                msg.fluid_pressure = float(row[1])

                bag.write(self.topics.xsens_pressure, msg, time)

        print('Building: ' + self.topics.xsens_temp)
        with open(folder + 'temp.txt') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            seq = 0
            for row in tqdm(csv_reader):
                msg = Temperature()

                t = max(0, int(row[0]))
                time = rospy.Time(0)
                time.secs = int(t / 1e9)
                time.nsecs = int(t) - int(time.secs * 1e9)

                msg.header.seq = seq
                seq += 1

                msg.header.stamp = time
                msg.header.frame_id = self.frames.xsens

                msg.temperature = float(row[1])

                bag.write(self.topics.xsens_temp, msg, time)

        print('Building: ' + self.topics.xsens_time)
        with open(folder + 'time.txt') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            seq = 0
            for row in tqdm(csv_reader):
                msg = TimeRef()

                t = max(0, int(row[0]))
                time = rospy.Time(0)
                time.secs = int(t / 1e9)
                time.nsecs = int(t) - int(time.secs * 1e9)

                msg.header.seq = seq
                seq += 1

                msg.header.stamp = time
                msg.header.frame_id = self.frames.xsens

                msg.year = float(row[1])
                msg.month = float(row[2])
                msg.day = float(row[3])
                msg.hours = float(row[4])
                msg.minutes = float(row[5])
                msg.secs = float(row[6])
                msg.nsecs = float(row[7])

                bag.write(self.topics.xsens_time, msg, time)

    def _build_gnss_data(self, bag, folder):

        print('Building: ' + self.topics.gnss_time)
        with open(folder + 'time.txt') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            seq = 0
            for row in tqdm(csv_reader):

                msg = TimeRef()

                t = max(0, int(row[0]))
                time = rospy.Time(0)
                time.secs = int(t / 1e9)
                time.nsecs = int(t) - int(time.secs * 1e9)

                msg.header.seq = seq
                seq += 1

                msg.header.stamp = time
                msg.header.frame_id = self.frames.gnss_antena_rear

                try:
                    msg.year = float(row[1])
                    msg.month = float(row[2])
                    msg.day = float(row[3])
                    msg.hours = float(row[4])
                    msg.minutes = float(row[5])
                    msg.secs = float(row[6])
                    msg.nsecs = float(row[7])

                    bag.write(self.topics.gnss_time, msg, time)
                except:
                    print('Unable to read: ' + str(row))

        print('Building: ' + self.topics.gnss_pose)
        with open(folder + 'pose.txt') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            seq = 0
            for row in tqdm(csv_reader):

                msg = PoseStamped()

                t = max(0, int(row[0]))
                time = rospy.Time(0)
                time.secs = int(t / 1e9)
                time.nsecs = int(t) - int(time.secs * 1e9)

                msg.header.seq = seq
                seq += 1

                msg.header.stamp = time
                msg.header.frame_id = self.frames.gnss_antena_rear

                try:
                    msg.pose.position.x = float(row[1])
                    msg.pose.position.y = float(row[2])
                    msg.pose.position.z = float(row[3])
                    msg.pose.orientation.w = float(row[4])

                    bag.write(self.topics.gnss_pose, msg, time)
                except:
                    print('Unable to read: ' + str(row))

    @staticmethod
    def _build_camera_data(bag, folder, topic, frame):

        bridge = CvBridge()
        print('Building: ' + topic)
        with open(folder + 'timestamps.txt') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            seq = 0
            for row in tqdm(csv_reader):
                path = os.path.join(folder, "frame%06i.jpeg" % seq)
                img = cv2.imread(path)
                msg = bridge.cv2_to_imgmsg(img, "bgr8")

                t = max(0, int(row[0]))
                time = rospy.Time(0)
                time.secs = int(t / 10e8)
                time.nsecs = int(t) - int(time.secs * 10e8)

                msg.header.seq = seq
                seq += 1

                msg.header.stamp = time
                msg.header.frame_id = frame

                bag.write(topic, msg, time)

    @staticmethod
    def _build_ir_camera_data(bag, folder, topic, frame):

        bridge = CvBridge()
        print('Building: ' + topic)
        with open(folder + 'timestamps.txt') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            seq = 0
            for row in tqdm(csv_reader):
                path = os.path.join(folder, "frame%06i.jpeg" % seq)
                #                 print(path)
                img = cv2.imread(path)
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                msg = bridge.cv2_to_imgmsg(gray, "mono8")

                t = max(0, int(row[0]))
                time = rospy.Time(0)
                time.secs = int(t / 10e8)
                time.nsecs = int(t) - int(time.secs * 10e8)

                msg.header.seq = seq
                seq += 1

                msg.header.stamp = time
                msg.header.frame_id = frame

                bag.write(topic, msg, time)

    @staticmethod
    def _build_lidar_data(bag, folder, topic, frame):

        print('Building: ' + topic)
        with open(folder + 'timestamps.txt') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            seq = 0
            for row in tqdm(csv_reader):
                pc = pypcd.PointCloud.from_path(os.path.join(folder, "scan%06i.pcd" % seq))
                msg = pc.to_msg()

                t = max(0, int(row[0]))
                time = rospy.Time(0)
                time.secs = int(t / 1e9)
                time.nsecs = int(t) - int(time.secs * 1e9)

                msg.header.seq = seq
                seq += 1

                msg.header.stamp = time
                msg.header.frame_id = frame

                bag.write(topic, msg, time)
