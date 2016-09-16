#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2016
# Original Author Michal Drwiega 2016
# Modified by Manuel Ahumada 2016
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#####################################################################

import serial
import rospy
import sys
import struct as st
import binascii
import tf
from Adafruit_BNO055 import BNO055

from time import time
from sensor_msgs.msg import Imu, Temperature, MagneticField

# Create and configure the BNO sensor connection.  Make sure only ONE of the
# below 'bno = ...' lines is uncommented:
# Raspberry Pi configuration with serial UART and RST connected to GPIO 4:
bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=4)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

imu_data = Imu()            # Filtered data
imu_raw = Imu()             # Raw IMU data
temperature_msg = Temperature() # Temperature
mag_msg = MagneticField()       # Magnetometer data

# Write data to IMU
def write_to_dev(ser, reg_addr, length, data):
    buf_out = bytearray()
    buf_out.append(START_BYTE_WR)
    buf_out.append(WRITE)
    buf_out.append(reg_addr)
    buf_out.append(length)
    buf_out.append(data)

    try:
        ser.write(buf_out)
        buf_in = bytearray(ser.read(2))
        # print("Writing, wr: ", binascii.hexlify(buf_out), "  re: ", binascii.hexlify(buf_in))
    except:
        return False

    if (buf_in.__len__() != 2) or (buf_in[1] != 0x01):
        #rospy.logerr("Incorrect Bosh IMU device response.")
        return False
    return True


# Main function
if __name__ == '__main__':
    rospy.init_node("bosch_imu_node")

    # Sensor measurements publishers
    pub_data = rospy.Publisher('imu/data', Imu, queue_size=1)
    pub_raw = rospy.Publisher('imu/raw', Imu, queue_size=1)
    pub_mag = rospy.Publisher('imu/mag', MagneticField, queue_size=1)
    pub_temp = rospy.Publisher('imu/temp', Temperature, queue_size=1)
    br = tf.TransformBroadcaster()

    # Get parameters values
    port = rospy.get_param('~port', '/dev/ttyAMA0')
    frame_id = rospy.get_param('~frame_id', 'imu_link')
    frequency = rospy.get_param('frequency', 100)
    operation_mode = rospy.get_param('operation_mode', OPER_MODE_NDOF)

    # Open serial port
    rospy.loginfo("Opening serial port: %s...", port)
    try:
        ser = serial.Serial(port, 115200, timeout=0.02)
    except serial.serialutil.SerialException:
        rospy.logerr("IMU not found at port " + port + ". Check the port in the launch file.")
        sys.exit(0)
    print("got here before")
    # Check if IMU ID is correct
    buf = read_from_dev(ser, CHIP_ID, 1)
    if buf == 0 or buf[0] != BNO055_ID:
        print(buf)
        print("Device ID incorrect")
        #rospy.logerr("Device ID is incorrect. Shutdown.")
        sys.exit(0)

    # IMU Configuration
    if not(write_to_dev(ser, BNO055_OPR_MODE_ADDR, 1, OPERATION_MODE_CONFIG)):
        rospy.logerr("Unable to set IMU into config mode.")

    if not(write_to_dev(ser, BNO055_PWR_MODE_ADDR, 1, POWER_MODE_NORMAL)):
        rospy.logerr("Unable to set IMU normal power mode.")

    if not(write_to_dev(ser, BNO055_PAGE_ID_ADDR, 1, 0x00)):
        rospy.logerr("Unable to set IMU register page 0.")

    if not(write_to_dev(ser, BNO055_SYS_TRIGGER_ADDR, 1, 0x00)):
        rospy.logerr("Unable to start IMU.")

    if not(write_to_dev(ser, BNO055_UNIT_SEL_ADDR, 1, 0x83)):
        rospy.logerr("Unable to set IMU units.")

    if not(write_to_dev(ser, BNO055_AXIS_MAP_CONFIG_ADDR, 1, 0x24)):
        rospy.logerr("Unable to remap IMU axis.")

    if not(write_to_dev(ser, BNO055_AXIS_MAP_SIGN_ADDR, 1, 0x06)):
        rospy.logerr("Unable to set IMU axis signs.")

    if not(write_to_dev(ser, BNO055_OPR_MODE_ADDR, 1, OPERATION_MODE_NDOF)):
        rospy.logerr("Unable to set IMU operation mode into operation mode.")

    rospy.loginfo("Bosch BNO055 IMU configuration complete.")

    rate = rospy.Rate(frequency)

    while not rospy.is_shutdown():
        buf = read_from_dev(ser, ACCEL_DATA, 45)
        if buf != 0:

            # data header 
            imu_data.header.stamp = rospy.Time.now()
            imu_data.header.frame_id = frame_id
            imu_data.header.seq = seq
            
            # orientation
            imu_data.orientation.x , imu_data.orientation.y , imu_data.orientation. z , imu_data.orientation.w = bno.read_quaternion() 
            
            # linear acceleration
            imu_data.linear_acceleration.x , imu_data.linear_acceleration.y , imu_data.linear_acceleration.z = bno.read_linear_acceleration()
            imu_data.linear_acceleration_covariance[0] = -1
            
            # angular velocity
            imu_data.angular_velocity.x , imu_data.angular_velocity.y , imu_data.angular_velocity.z = bno.read_gyroscope()
            imu_data.angular_velocity_covariance[0] = -1
            
            # publish imu data and tf transformation
            pub_data.publish(imu_data)
            br.sendTransform((0,0,0),(imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z,imu_data.orientation.w),rospy.Time.now(),"imuNew","imu_link")
            
            seq = seq + 1
        rate.sleep()
    ser.close()
