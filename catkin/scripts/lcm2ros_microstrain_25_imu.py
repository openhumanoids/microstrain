#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import Imu
#from tf import transformations

from geometry_msgs.msg import PoseStamped

import lcm
from bot_core.pose_t import pose_t
from bot_core.ins_t import ins_t

lc = lcm.LCM()


def on_imu(channel, data):
    m = ins_t.decode(data)
    pyTime = rospy.Time.from_sec(m.utime*1E-6)
    #print "Received ins message", pyTime

    pose = PoseStamped()

    t = rospy.get_rostime()
    pose.header.stamp = pyTime
    pose.header.frame_id = 'imu_link'

    pose.pose.orientation.w = m.quat[0] 
    pose.pose.orientation.x = m.quat[1]
    pose.pose.orientation.y = m.quat[2]
    pose.pose.orientation.z = m.quat[3]
    imuRosPosePub.publish(pose)


    imu = Imu()
    imu.header = pose.header
    imu.orientation = pose.pose.orientation
    imu.angular_velocity.x = m.gyro[0]
    imu.angular_velocity.y = m.gyro[1]
    imu.angular_velocity.z = m.gyro[2]
    imu.linear_acceleration.x = m.accel[0]
    imu.linear_acceleration.y = m.accel[1]
    imu.linear_acceleration.z = m.accel[2]
    imuRosPub.publish(imu)


####################################################################
#lc = lcm.LCM()
rospy.init_node('lcm2ros_microstrain_25_imu', anonymous=True)
imuRosPosePub = rospy.Publisher('/imu/pose_from_orientation', PoseStamped, queue_size=10)
imuRosPub = rospy.Publisher('/imu/imu', Imu, queue_size=10)

print "started"

lc.subscribe("MICROSTRAIN_INS", on_imu)

while True:
  lc.handle()