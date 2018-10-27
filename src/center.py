#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import time
import serial
import sys

from PID import pid
from numpy import genfromtxt

import tf
import math

from random import randint


# yaw = pid(9,0,100)
yaw = pid(3.4,0,2)
# velo = pid(18500,0,10000)
velo = pid(9800,0,5000)

yaw_callback_time = 0.015 #15 ms
yaw_callback_last_time = 0

ser = None
__yaw = 0
__velo = 0



def open_connection():
    global ser

    try:
        ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
        )
    except:
        ser = serial.Serial(
        port='/dev/ttyACM1',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
        )
    ser.close()
    ser.open()

    while True:
        if int(ser.readline()) == 20:
            print('connection started')
            break




def conver2polar(robot_x,robot_y,point_x,point_y):
    phi =  math.degrees(math.atan2(point_y - robot_y,point_x - robot_x))
    y = (- phi - 90)
    if y < 0 : y = y + 360
    rho = math.sqrt(((point_y - robot_y)**2) + ((point_x - robot_x)**2))
    return rho,y


def read_position(data):
    global yaw_callback_last_time , __yaw
    robot_x_pos = data.pose.position.x * 10 # meter
    robot_y_pos = data.pose.position.y * 10 # meter
    (eu_roll, eu_pitch, eu_yaw) = tf.transformations.euler_from_quaternion(
        [data.pose.orientation.x,
         data.pose.orientation.y,
         data.pose.orientation.z,
         data.pose.orientation.w]
          )
    now = time.time()
    if now - yaw_callback_last_time > yaw_callback_time:
        R,Tetha = conver2polar(robot_x_pos,robot_y_pos,0,0)
        __yaw = yaw.update_pid(0,eu_yaw*100)
        __velo = velo.update_pid(0,R)

        print("x error -> %f"%(robot_x_pos))
        print("Y error -> %f"%(robot_y_pos))


        ser.write('%d,%d,%d,%d\n'%(__velo,Tetha,__yaw,0))
        # print('%d,%d,%d,%d\n'%(__velo,Tetha,__yaw,dot))
        yaw_callback_last_time = now


def listen_to_aruco_single_node():

    rospy.init_node('Well_O_Graph', anonymous=False)

    rospy.Subscriber("/aruco_single/pose", geometry_msgs.msg.PoseStamped, read_position)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()







if __name__ == '__main__':
    open_connection()
    listen_to_aruco_single_node()
