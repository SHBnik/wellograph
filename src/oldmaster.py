#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import time
import serial

from PID import pid
from numpy import genfromtxt

import tf
import math

from random import randint


# yaw = pid(10,0,450)
yaw = pid(9,0,100)
# velo = pid(9000,0,10)
# velo = pid(18500,50,10000)
velo = pid(18500,100,10000)

yaw_callback_time = 0.015 #15 ms
yaw_callback_last_time = 0

ser = None
__yaw = 0
__velo = 0

my_data = []


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


index = 6000
once = True
def callback(data):
    global yaw_callback_last_time , __yaw,index,once
    robot_x_pos = data.pose.position.x * 10 # meter
    robot_y_pos = data.pose.position.y * 10 # meter
    point_x = float(my_data[index][0]/1000)
    point_y = float(my_data[index][1]/1000)
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
        R,Tetha = conver2polar(robot_x_pos,robot_y_pos,point_x,point_y)
        #R = 0.6
        #Tetha = 0
        __yaw = yaw.update_pid(0,eu_yaw*100)
        __velo = 0
        dot = 0
        # print("phi -> %f"%Tetha)
        # print("velo pid -> %f"%__velo)
        print("term P yaw -> %f"%velo.get_term_p())
        # print("term P velo -> %f"%velo.get_term_p())
        # print("term d yaw -> %f"%velo.get_term_d())
        print("x error -> %f"%(robot_x_pos - point_x))
        print("Y error -> %f"%(robot_y_pos - point_y))
        print('%d dots of %d'%(index,len(my_data)));
        __velo = velo.update_pid(0,R)
        if velo.get_term_p() < 300 and velo.get_term_p() > -300 and once == True:
            velo.resetI()
            once = False
        if velo.get_term_p() < 50 and velo.get_term_p() > -50 :
            dot = 1
            dot_done = True
            velo.resetI()
            once = True
            my_data.pop(index)
            index = randint(0, len(my_data))
            if index == len(my_data):
                while True:
                    pass

        ser.write('%d,%d,%d,%d\n'%(__velo,Tetha,__yaw,dot))
        print('%d,%d,%d,%d\n'%(__velo,Tetha,__yaw,dot))
        yaw_callback_last_time = now


def listener():

    rospy.init_node('Well_O_Graph', anonymous=False)

    rospy.Subscriber("/aruco_single/pose", geometry_msgs.msg.PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    y = genfromtxt('/home/shb/Desktop/Gaavkhouni.csv', delimiter=',')
    my_data = y.tolist()
    open_connection()
    listener()
