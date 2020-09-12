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
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

pwm_ENA = None
pwm_ENB = None
left_pwm = 0
right_pwm = 0

current_pwm  = 0   
goal_direction = 0
direction_step = 5

left_pwm_lock = threading.Lock()
right_pwm_lock = threading.Lock()

pwm_step = 1
left_pwm_step = 20
right_pwm_step = 20
pwm_dead_threshold = 30

IN1_status = False
IN2_status = False
IN3_status = False
IN4_status = False

ENA = 16
IN1 = 21
IN2 = 20

ENB = 13
IN3 = 19
IN4 = 26

var = None
entry = None

camera_1_init = 90
camera_2_init = 90

camera_channel = [0, 8]
last_angle = [camera_1_init, camera_2_init]

camera_minimum = [0, 80]
camera_maximum = [180, 130]

# 角度变量  
aax = 0
aay = 0
aaz = 0
agx = 0
agy = 0
agz = 0

#加速度计偏移量
axo = 0
ayo = 0
azo = 0; 
#陀螺仪偏移量            
gxo = 0
gyo = 0
gzo = 0           

last_time = 0
AcceRatio = 16384.0 # 加速度计比例系数  
GyroRatio = 131.0 # 陀螺仪比例系数  
n_sample = 8 # 加速度计滤波算法采样个数 

# x,y轴采样队列
aaxs = [0] * 8
aays = [0] * 8
aazs = [0] * 8

sensor = None

def get_raw_data():

	accel_data = sensor.get_accel_data()
	gyro_data = sensor.get_gyro_data()
    
	gx = gyro_data['x']
	gy = gyro_data['y']
	gz = gyro_data['z']
	ax = accel_data['x']
	ay = accel_data['y']
	az = accel_data['z']  
    
	return gx, gy, gz, ax, ay, az 

def mpu6050_init():
	global sensor
	global axo
	global ayo
	global azo
	global gxo
	global gyo
	global gzo
	global bus
	global goal_direction
    
	sensor = mpu6050(0x68)
	time.sleep(0.1)

	# 采样次数
	sampling_times = 500
	for i in range(0, sampling_times):
		gx, gy, gz, ax, ay, az = get_raw_data()
		# 采样和
		axo += ax
		ayo += ay
		azo += az
		gxo += gx
		gyo += gy
		gzo += gz  
        
	# 计算加速度计偏移
	axo /= sampling_times
	ayo /= sampling_times
	azo /= sampling_times 
	# 计算陀螺仪偏移    
	gxo /= sampling_times
	gyo /= sampling_times
	gzo /= sampling_times 
      
	goal_direction = get_angle()
    
def get_angle(): 
	global aax
	global aay
	global aaz
	global last_time
	global aaxs
	global aays
	global aazs
	global agx
	global agy
	global agz
    
	current_time = datetime.now().microsecond  
	diff_time = (current_time - last_time) / 1000.0
	last_time = current_time
  
	gx, gy, gz, ax, ay, az = get_raw_data()  
  
	'''
	accx = ax / AcceRatio # x轴加速度  
	accy = ay / AcceRatio # y轴加速度  
	accz = az / AcceRatio # z轴加速度  
  
	if accz != 0:
		aax = math.atan(accy / accz) * (-180) / math.pi # y轴对于z轴的夹角  
		aay = math.atan(accx / accz) * 180 / math.pi    # x轴对于z轴的夹角  
	if accy != 0:
		aaz = math.atan(accz / accy) * 180 / math.pi    # z轴对于y轴的夹角  
  
	# 对于加速度计原始数据的滑动加权滤波算法
	aax_sum = 0;               
	aay_sum = 0;  
	aaz_sum = 0;  
   
	for i in range(1, n_sample):
		aaxs[i-1] = aaxs[i]  
		aax_sum += aaxs[i] * i  
		aays[i-1] = aays[i]  
		aay_sum += aays[i] * i 
		aazs[i-1] = aazs[i]  
		aaz_sum += aazs[i] * i  
  
	aaxs[n_sample-1] = aax  
	aax_sum += aax * n_sample 
	aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0 # 角度调幅至0-90°  
	aays[n_sample-1] = aay                        # 此处应用实验法取得合适的系数  
	aay_sum += aay * n_sample                     # 本例系数为9/7  
	aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0 
	aazs[n_sample-1] = aaz  
	aaz_sum += aaz * n_sample 
	aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0
	'''
  
	gyrox = -(gx-gxo) / GyroRatio * diff_time   # x轴角速度  
	gyroy = -(gy-gyo) / GyroRatio * diff_time   # y轴角速度  
	gyroz = -(gz-gzo) / GyroRatio * diff_time   # z轴角速度  
	agx += gyrox                                # x轴角速度积分  
	agy += gyroy                                # y轴角速度积分  
	agz += gyroz                                # z轴角速度积分

	return agx

def mpu6050_thread():
	mpu6050_init()
	print("mpu6050_thread start")
	while True:
		angle = get_angle()
		#print(angle)
		angle_queue.put(angle)


def set_servo_angle(channel, angle):
	data = 0.5 + (angle/180.0)*(2.5 - 0.5)
	data = int(data / 20 * 4096)

	pwm.set_pwm(channel, 0, int(data))

def per_move(camera, new_dAngle):
	channel = camera_channel[camera] 
	angle = last_angle[camera] + new_dAngle

	if (angle < camera_minimum[camera]) or (angle > camera_maximum[camera]):
		print("angle exceed threadhold")
		return
        
	last_angle[camera] = angle
	set_servo_angle(channel, angle)

last_move_x = 0
last_move_y = 0

def camera_move(camera_move_x, camera_move_y):
	global last_move_x
	global last_move_y
    
	Kp0 = -1
	Kp1 = 1
    
	if abs(last_move_x - camera_move_x) <= 4:
		camera_move_x = 0
	if abs(last_move_y - camera_move_y) <= 4:
		camera_move_y = 0    
        
	last_move_x = camera_move_x
	last_move_y = camera_move_y
    
	new_dAngle_0 = Kp0 * camera_move_x
	new_dAngle_1 = Kp1 * camera_move_y
	per_move(0, new_dAngle_0)
	per_move(1, new_dAngle_1)


def get_current_direction():
	sampling_times = 5

	sum = 0
	for i in range(sampling_times):
		sum += angle_queue.get()

	return sum / sampling_times

def motor_thread():
	global current_pwm
	global goal_direction
	global last_pwm

	minimum_pwm = 10
	maximum_pwm = 100
	Kp = -0.1

	while True:
		current_direction = get_current_direction()
		diff_direction = current_direction - goal_direction
		if abs(diff_direction) < 5:
			diff_direction = 0

		if current_pwm > 0:
			left_pwm = current_pwm + Kp * diff_direction
		elif current_pwm < 0:
			left_pwm = current_pwm - Kp * diff_direction
		else:
			left_pwm = 0

		if abs(left_pwm) <= minimum_pwm:
			left_pwm = 0
			IN1_status = False
			IN2_status = False
		elif left_pwm > minimum_pwm:
			if left_pwm > maximum_pwm:
				left_pwm = maximum_pwm
			IN1_status = True
			IN2_status = False
		elif left_pwm < -minimum_pwm:
			left_pwm = abs(left_pwm)
			if left_pwm > maximum_pwm:
				left_pwm = maximum_pwm
			IN1_status = False
			IN2_status = True


		if current_pwm > 0:
			right_pwm = current_pwm - Kp * diff_direction
		elif current_pwm < 0:
			right_pwm = current_pwm + Kp * diff_direction
		else:
			right_pwm = 0

		if abs(right_pwm) <= minimum_pwm:
			right_pwm = 0
			IN3_status = False
			IN4_status = False
		elif right_pwm > minimum_pwm:
			if right_pwm > maximum_pwm:
				right_pwm = maximum_pwm
			IN3_status = True
			IN4_status = False
		elif right_pwm < -minimum_pwm:
			right_pwm = abs(right_pwm)
			if right_pwm > maximum_pwm:
				right_pwm = maximum_pwm
			IN3_status = False
			IN4_status = True
		
		
		left_pwm = min(100, left_pwm)
		right_pwm = min(100, right_pwm)
		print(current_pwm, left_pwm, right_pwm, IN1_status, IN2_status, IN3_status, IN4_status, diff_direction, goal_direction, current_direction)
		pwm_ENA.ChangeDutyCycle(left_pwm)
		pwm_ENB.ChangeDutyCycle(right_pwm)
		
		GPIO.output(IN1, IN1_status)  
		GPIO.output(IN2, IN2_status)  
		GPIO.output(IN3, IN3_status)  
		GPIO.output(IN4, IN4_status)       

def motor_reduced_thread():
	global current_pwm

	while True:
		if current_pwm == 0:
			continue
		if current_pwm > 0:
			current_pwm -= 1
		elif current_pwm < 0:
			current_pwm += 1
		time.sleep(0.5)

def isForward():
	return current_pwm > 0

def isBackward():
	return current_pwm < 0

def upCallBack():
	global current_pwm
	global goal_direction
	if current_pwm == 0:
		current_pwm = 15
		goal_direction = angle_queue.get();
	else:
		current_pwm = min(current_pwm + 1, 100)
	time.sleep(0.05)


def downCallBack():
	global current_pwm
	if current_pwm == 0:
		current_pwm = -15
		goal_direction = angle_queue.get();
	else:
		current_pwm = max(current_pwm - 1, -100)
	time.sleep(0.05)

def leftCallBack():
	global goal_direction
	goal_direction -= direction_step
	time.sleep(0.05)

def rightCallBack():
	global goal_direction
	goal_direction += direction_step
	time.sleep(0.05)
	
def haltCallBack():
	global current_pwm
	global goal_direction
	global IN1_status
	global IN2_status
	global IN3_status
	global IN4_status

	current_pwm = 0
	goal_direction = angle_queue.get()
	IN1_status = False
	IN2_status = False
	IN3_status = False
	IN4_status = False

def motor_ouput_init():
	global pwm_ENA
	global pwm_ENB
	global current_pwm

	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)		  # 使用BCM编号方式
	GPIO.setup(ENA, GPIO.OUT)	
	GPIO.setup(IN1, GPIO.OUT)	   
	GPIO.setup(IN2, GPIO.OUT)	 
	GPIO.setup(ENB, GPIO.OUT)	   
	GPIO.setup(IN3, GPIO.OUT)	   
	GPIO.setup(IN4, GPIO.OUT)	  

	init_freq = 50  # initial frequency in Hz
	pwm_ENA = GPIO.PWM(ENA, init_freq)
	pwm_ENB = GPIO.PWM(ENB, init_freq)	
	init_dc = 0  
	pwm_ENA.start(init_dc)
	pwm_ENB.start(init_dc)
    
	current_pwm = 0

def camera_init():
	set_servo_angle(camera_channel[0], camera_1_init)
	set_servo_angle(camera_channel[0], camera_2_init)

def get_host_ip():
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		s.connect(('8.8.8.8', 80))
		ip = s.getsockname()[0]
	finally:
		s.close()

	return ip

def recv_data_thread(socket):
	start_str = '*'
	end_str = '&'
	split_str = '~'
	uncomplete_str = str()
	while True:
		recv_data = socket.recv(64).decode("utf-8")
		data_array = recv_data.split(split_str)
		length = len(data_array)

		for data in data_array:
			if len(data) < 1:
				continue
			if data[0] != start_str:
				tmp_str = uncomplete_str + data
				data_queue.put(tmp_str[1:-1])
				uncomplete_str = str()
			elif data[-1] != end_str:
				uncomplete_str = data
			else:
				data_queue.put(data[1:-1])
	 
def recv_thread():
	left = 0
	right = 1
	up = 2
	down = 3
	mouseX = 4
	mouseY = 5
	halt = 6

	ip = get_host_ip()
	tcp_server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	tcp_server_socket.bind((ip, 10999))
	tcp_server_socket.listen(120)
	new_client_socket, client_addr = tcp_server_socket.accept()
	threading.Thread(target = recv_data_thread, args = (new_client_socket,)).start()
    
	while True:
		direction_str = data_queue.get().split(',')
		if direction_str[left] == '1':
			print('left')
			leftCallBack()
		if direction_str[right] == '1':
			print('right')
			rightCallBack()
		if direction_str[up] == '1':
			print('up')
			upCallBack()
		if direction_str[down] == '1':
			print('down')
			downCallBack()
		if direction_str[halt] == '1':
			print('halt')
			haltCallBack()
            
		camera_move_x = int(direction_str[mouseX])        
		camera_move_y = int(direction_str[mouseY])        
		camera_move(camera_move_x, camera_move_y)         

	server_socket.close()
	
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
