#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import time
import serial

from PID import pid

import tf
import math

# yaw = pid(6,0.00008,20) #9V
# yaw = pid(8,0.0008,450)
yaw = pid(7.6,0,450)

velo = pid(10000,0,0)

Y = pid(10000,0,0)

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
    # if phi < 0 : phi = phi + 360
    # if phi > 360 : phi = phi - 360
    rho = math.sqrt(((point_y - robot_y)**2) + ((point_x - robot_x)**2))
    return rho,y



def callback(data):
    global yaw_callback_last_time , __yaw
    robot_x_pos = data.pose.position.x * 10 # meter
    robot_y_pos = data.pose.position.y * 10 # meter
    # print(data.pose.position.z * 10)
    (eu_roll, eu_pitch, eu_yaw) = tf.transformations.euler_from_quaternion(
        [data.pose.orientation.x,
         data.pose.orientation.y,
         data.pose.orientation.z,
         data.pose.orientation.w]
          )
    now = time.time()
    if now - yaw_callback_last_time > yaw_callback_time:
        # print('ori -> %f'%(eu_yaw*100))
        # R,Tetha = conver2polar(robot_x_pos,robot_y_pos,0,0)
        #R = 0.6
        Tetha = 90
        __yaw = yaw.update_pid(0,eu_yaw*100)
        __velo = 0
        dot = 0
        # print("phi -> %f"%Tetha)
        # print("velo pid -> %f"%__velo)
        print("term p yaw -> %f"%Y.get_term_p())
        print("term d yaw -> %f"%Y.get_term_d())
        __velo = velo.update_pid(0,robot_y_pos)
        __y = Y.update_pid(0,robot_x_pos)
        ser.write('%d,%d,%d,%d\n'%(0,0,eu_yaw * 1000,dot))
        # print('%d,%d,%d,%d\n'%(__velo,Tetha,__yaw,dot))
        yaw_callback_last_time = now


def listener():

    rospy.init_node('Well_O_Graph', anonymous=False)

    rospy.Subscriber("/aruco_single/pose", geometry_msgs.msg.PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    open_connection()
    listener()
