import Adafruit_PCA9685
from motor import Motor
from servo import ServoCamera
from recvdata import DataReceiver
from m6050 import MPU6050

	
def main():

	pwm = Adafruit_PCA9685.PCA9685()
	pwm.set_pwm_freq(50)

	m6050 = MPU6050()
	motor = Motor(m6050)
	servo = ServoCamera(pwm)
	dataRecv = DataReceiver(motor, servo)

	dataRecv.start()

if __name__ == '__main__':
	main()
