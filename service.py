import tkinter as tk
import Adafruit_PCA9685	
import RPi.GPIO as GPIO		 # 引入GPIO模块
import time					 # 引入time模块
import threading
import socket
import threading
import queue
from mpu6050 import mpu6050
from datetime import datetime
import math

data_queue = queue.Queue()
angle_queue = queue.Queue()

thread_list = []

channel_angle = [None, None, None, None, None, None, None, None, None]
channel_speed = [0, 0, 0, 0, 0, 0, 0, 0, 0]
g_channel = 0


var = None
entry = None











def get_current_direction():
	sampling_times = 5

	sum = 0
	for i in range(sampling_times):
		sum += angle_queue.get()

	return sum / sampling_times


    



	
def main():
	motor_ouput_init()
	camera_init()
	mThread = threading.Thread(target = motor_thread)
	mThread.setDaemon(True)
	mThread.start()
	thread_list.append(mThread)

	mrThread = threading.Thread(target = motor_reduced_thread)
	mrThread.setDaemon(True)
	mrThread.start()
	thread_list.append(mrThread)
	
	rThread = threading.Thread(target = recv_thread)
	rThread.setDaemon(True)
	rThread.start()
	thread_list.append(rThread)
	
	aThread = threading.Thread(target = mpu6050_thread)
	aThread.setDaemon(True)
	aThread.start()
	thread_list.append(aThread)

	for thread in thread_list:
		thread.join()


if __name__ == '__main__':
	main()
