import Adafruit_PCA9685
from motor import Motor
from camera import ServoCamera
from recvdata import DataReceiver
from m6050 import MPU6050

	
def main():

	pwm = Adafruit_PCA9685.PCA9685()
	pwm.set_pwm_freq(50)

	m6050 = MPU6050()
	motor = Motor(pwm, m6050)
	camera = ServoCamera(pwm)
	dataRecv = DataReceiver(motor, camera)

	dataRecv.start()

if __name__ == '__main__':
	main()
