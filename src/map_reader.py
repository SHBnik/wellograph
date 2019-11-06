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
from random import shuffle

import threading
import os




yaw = pid(3.4,0,2)
#velo = pid(9800,50,5000)
velo = pid(1550,0.095,9.7)

base_path = os.path.dirname(os.path.realpath(__file__))

yaw_callback_time = 0.015 #15 ms
yaw_callback_last_time = 0

ser = None
__yaw = 0
__velo = 0


time_frame_num = 0

map_index = None
index = 0
Drawn_points = 0
all_points = 0
once = True
home_flag = False

list_of_maps = [
    str(base_path + '/maps/Bakhtegan_Maharlou.csv'),#0
    str(base_path + '/maps/Gaavkhuni.csv'),#1
    str(base_path + '/maps/Parishan.csv'),#2
    str(base_path + '/maps/Urmia.csv')]#3


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


def home():
    global time_frame_num , home_flag
    time_frame_num += 1
    if time_frame_num > 10:
        time_frame_num = 0
        home_flag = True
    
    timer = threading.Timer(1, home)
    timer.daemon = True
    timer.start()


def get_backup_of_index(inx):
    global map_index
    with open('index.bc', 'w') as backup_index:
        backup_index.write(str(map_index)+'\n')
        backup_index.write(str(inx)+'\n')




def read_position(data):
    global yaw_callback_last_time , __yaw,index,once,Drawn_points,wells_position_list,home_flag,time_frame_num
    robot_x_pos = data.pose.position.x * 10 # meter
    robot_y_pos = data.pose.position.y * 10 # meter
    point_x = float(wells_position_list[index][1]/100)#convert centimeter to milimeter 
    point_y = float(wells_position_list[index][0]/100)#convert centimeter to milimeter
    (eu_roll, eu_pitch, eu_yaw) = tf.transformations.euler_from_quaternion(
        [data.pose.orientation.x,
         data.pose.orientation.y,
         data.pose.orientation.z,
         data.pose.orientation.w]
          )
    now = time.time()
    if now - yaw_callback_last_time > yaw_callback_time:
        dot = 0
        if home_flag:
	        # R = 0.1
	        # Tetha = 270
            R,Tetha = conver2polar(robot_x_pos,robot_y_pos,3,2)
        else:
            R,Tetha = conver2polar(robot_x_pos,robot_y_pos,point_x,point_y)
        __yaw = yaw.update_pid(0,eu_yaw*100)
        __velo = velo.update_pid(0,R)

        print("x error -> %f"%(robot_x_pos - point_x))
        print("Y error -> %f"%(robot_y_pos - point_y))
        print('%d dots of %d'%(Drawn_points,all_points))

        if home_flag == True and velo.get_error() < 0.8 and velo.get_error() > -0.8:
            home_flag = False
            return

        if velo.get_error() < 0.2 and velo.get_error() > -0.2 and once == True:
            velo.resetI()
            once = False

	               
        if velo.get_error() < 0.04 and velo.get_error() > -0.04 and home_flag != True :
            home_flag = False
            time_frame_num = 0
            Drawn_points += 1
            dot = 1
            once = True
            velo.resetI()
            # wells_position_list.pop(index)
            # index = randint(0, len(wells_position_list))
            index += 1
            get_backup_of_index(index)

        ser.write('%d,%d,%d,%d\n'%(__velo,Tetha,__yaw,dot))

        if Drawn_points == all_points:
            while True:
                print("map Done!!!!\nrerun the program with new map!!!!")
        # print('%d,%d,%d,%d\n'%(__velo,Tetha,__yaw,dot))
        yaw_callback_last_time = now


def listen_to_aruco_single_node():

    rospy.init_node('Well_O_Graph', anonymous=False)

    rospy.Subscriber("/aruco_single/pose", geometry_msgs.msg.PoseStamped, read_position)

    rospy.spin()





if __name__ == '__main__':
    map_index = int(sys.argv[1])
    if map_index == 97:

        backup_index = open('index.bc', 'r')
        map_index = int(backup_index.readline())
        index = int(backup_index.readline())
        Drawn_points = index

    elif map_index > 4 or map_index < 0:
        print("!!! index out of range !!!")
        while True:pass
    map = genfromtxt(list_of_maps[map_index], delimiter=',')
    wells_position_list = map.tolist()
    all_points = len(wells_position_list)

    open_connection()

    timer = threading.Timer(10, home)
    timer.daemon = True
    timer.start()

    listen_to_aruco_single_node()
