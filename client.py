import socket
import tkinter as tk
import threading
import time

direction_lock = threading.Lock()
direction_str = ['0', '0', '0', '0']
left = 0
right = 1
up = 2
down = 3
mouseX = 4
mouseY = 5

ip = '192.168.0.124'

def reset_direction_str():
	global direction_str
	direction_lock.acquire()
	direction_str[left] = '0'
	direction_str[right] = '0'
	direction_str[up] = '0'
	direction_str[down] = '0'
	direction_lock.release()

def upCall(value):
	global direction_str
	direction_lock.acquire()
	direction_str[up] = '1'
	direction_lock.release()

def downCall(value):
	global direction_str
	direction_lock.acquire()
	direction_str[down] = '1'
	direction_lock.release()

def leftCall(value):
	global direction_str
	direction_lock.acquire()
	direction_str[left] = '1'
	direction_lock.release()

def rightCall(value):
	global direction_str
	direction_lock.acquire()
	direction_str[right] = '1'
	direction_lock.release()

def gui_thread():
	window = tk.Tk()
	frame = tk.Frame(window, width=200, height=200)   
	frame.bind("<Up>", upCall) #触发的函数  
	frame.bind("<Down>", downCall) #触发的函数  
	frame.bind("<Left>", leftCall) #触发的函数  
	frame.bind("<Right>", rightCall) #触发的函数  
	frame.focus_set()  #必须获取焦点  
	frame.pack()  
	window.mainloop()  

def send_thread():
	tcp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	tcp_client_socket.connect(("10.32.17.1", 9999))
	while True:
		direction_lock.acquire()
		str = ','.join(direction_str)
		direction_lock.release()
		tcp_client_socket.send(bytes(str, encoding="utf-8"))
		
		reset_direction_str()
		time.sleep(0.1)
		
	tcp_client_socket.close()

def main():
	gTread = threading.Thread(target = gui_thread)
	gTread.setDaemon(True)
	gTread.start()

	sTread = threading.Thread(target = send_thread)
	sTread.setDaemon(True)
	sTread.start()

	gTread.join()
	sTread.join()

if __name__ == '__main__':
	main()
