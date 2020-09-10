import tkinter as tk
import Adafruit_PCA9685	
import RPi.GPIO as GPIO		 # 引入GPIO模块
import time					 # 引入time模块
import threading
import socket
import threading
import queue

data_queue = queue.Queue()

channel_angle = [None, None, None, None, None, None, None, None, None]
channel_speed = [0, 0, 0, 0, 0, 0, 0, 0, 0]
g_channel = 0
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

pwm_ENA = None
pwm_ENB = None
left_pwm = 0
right_pwm = 0

left_pwm_lock = threading.Lock()
right_pwm_lock = threading.Lock()

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
    
	if abs(last_move_x - camera_move_x) <= 2:
		camera_move_x = 0
	if abs(last_move_y - camera_move_y) <= 2:
		camera_move_y = 0    
        
	last_move_x = camera_move_x
	last_move_y = camera_move_y
    
	new_dAngle_0 = Kp0 * camera_move_x
	new_dAngle_1 = Kp1 * camera_move_y
	per_move(0, new_dAngle_0)
	per_move(1, new_dAngle_1)
    
def motor_thread():
	while True:
		pwm_ENA.ChangeDutyCycle(left_pwm)
		pwm_ENB.ChangeDutyCycle(right_pwm)
		
		GPIO.output(IN1, IN1_status)  
		GPIO.output(IN2, IN2_status)  
		GPIO.output(IN3, IN3_status)  
		GPIO.output(IN4, IN4_status)  

def motor_reduced_thread():
	global left_pwm
	global right_pwm

	while True:
		if left_pwm > 0:
			left_pwm = left_pwm - 1

		if right_pwm > 0:
			right_pwm = right_pwm - 1
		
		time.sleep(0.1)

def left_turn(left_pwm_down):
	global left_pwm
	left_pwm = left_pwm - left_pwm_down

def right_turn(right_pwm_down):
	global right_pwm
	right_pwm = right_pwm - right_pwm_down

def upCallBack():
	global left_pwm
	global right_pwm
	global IN1_status
	global IN2_status
	global IN3_status
	global IN4_status

	if left_pwm < 100 - 5:
		left_pwm = left_pwm + 5
	elif left_pwm >= 100 - 5:
		left_pwm = 100

	IN1_status = True
	IN2_status = False

	if right_pwm < 100 - 5:
		right_pwm = right_pwm + 5
	elif right_pwm >= 100 - 5:
		right_pwm = 100
	IN3_status = True
	IN4_status = False

def downCallBack():
	global left_pwm
	global right_pwm
	global IN1_status
	global IN2_status
	global IN3_status
	global IN4_status

	if left_pwm > 0 + 5:
		left_pwm = left_pwm - 5
		IN1_status = False
		IN2_status = True

	if right_pwm > 0 + 5:
		right_pwm = right_pwm - 5
		IN3_status = False
		IN4_status = True

def motor_ouput_init():
	global pwm_ENA
	global pwm_ENB
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

	ip = get_host_ip()
	tcp_server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	tcp_server_socket.bind((ip, 10999))
	tcp_server_socket.listen(120)
	new_client_socket, client_addr = tcp_server_socket.accept()
	threading.Thread(target = recv_data_thread, args = (new_client_socket,)).start()
    
	while True:
		print(data_queue.qsize())
		direction_str = data_queue.get().split(',')
		print(direction_str, left_pwm, right_pwm)
		if direction_str[left] == '1':
			print('left')
		if direction_str[right] == '1':
			print('right')
		if direction_str[up] == '1':
			print('up')
			upCallBack()
		if direction_str[down] == '1':
			print('down')
			downCallBack()
            
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

	mrThread = threading.Thread(target = motor_reduced_thread)
	mrThread.setDaemon(True)
	mrThread.start()
	
	rThread = threading.Thread(target = recv_thread)
	rThread.setDaemon(True)
	rThread.start()
	
	mThread.join()
	mrThread.join()
	rThread.join()

if __name__ == '__main__':
	main()
