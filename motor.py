import RPi.GPIO as GPIO # 引入GPIO模块
import threading
import time


class Motor:
    def __init__(self, m6050):

        self.pwm_ENA = None
        self.pwm_ENB = None
        self.pwm_step = 1
        self.left_pwm_step = 20
        self.right_pwm_step = 20
        self.pwm_dead_threshold = 30

        self.IN1_status = False
        self.IN2_status = False
        self.IN3_status = False
        self.IN4_status = False

        self.ENA = 16
        self.IN1 = 21
        self.IN2 = 20

        self.ENB = 13
        self.IN3 = 19
        self.IN4 = 26

        self.speedPWMStep = 10
        self.speedPWMMax = 200
        self.speedPWMMin = -200
        self.directionStep = 5

        self.goalDirection = 0
        self.speed = 0
        self.m6050 = m6050

        self.directionLock = threading.Lock()
        self.speedLock = threading.Lock()

        self.motorOuputInit()

    def motorOuputInit(self):
        '''
        电机输出初始化
        '''
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)  # 使用BCM编号方式
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)

        init_freq = 50  # initial frequency in Hz
        self.pwm_ENA = GPIO.PWM(self.ENA, init_freq)
        self.pwm_ENB = GPIO.PWM(self.ENB, init_freq)
        init_dc = 0
        self.pwm_ENA.start(init_dc)
        self.pwm_ENB.start(init_dc)

        controlThread = threading.Thread(target=Motor.motorControlThread, args=(self,))
        controlThread.setDaemon(True)
        controlThread.start()

        reduceSpeedThread = threading.Thread(target=Motor.motorReducedThread, args=(self,))
        reduceSpeedThread.setDaemon(True)
        reduceSpeedThread.start()


    def motorControlThread(self):
        """
        电机控制线程
        根据小车速度和目标方向，计算电机的转速
        """
        Kp = -0.1 # p控制

        while True:
            # 计算实际方向和目标方向的偏差
            currentDirection = self.m6050.getCurrDirection()
            diffDirection = currentDirection - self.goalDirection

            # 偏差小于阈值则认为偏差为0，防止小车抖动
            directionThreshold = 5
            if abs(diffDirection) < directionThreshold:
                diffDirection = 0

            leftPwm = self.speed
            if leftPwm > 0:
                leftPwm += Kp * diffDirection
            elif leftPwm < 0:
                leftPwm -= Kp * diffDirection

            if abs(leftPwm) <= self.speedPWMMin:
                leftPwm = 0
                self.IN1_status = False
                self.IN2_status = False
            elif leftPwm > self.speedPWMMin:
                if leftPwm > self.speedPWMMax:
                    leftPwm = self.speedPWMMax
                self.IN1_status = True
                self.IN2_status = False
            elif leftPwm < self.speedPWMMin:
                leftPwm = abs(leftPwm)
                if leftPwm > self.speedPWMMax:
                    leftPwm = self.speedPWMMin
                self.IN1_status = False
                self.IN2_status = True

            rightPwm = self.speed
            if rightPwm > 0:
                rightPwm -= Kp * diffDirection
            elif rightPwm < 0:
                rightPwm += Kp * diffDirection

            if abs(rightPwm) <= self.speedPWMMin:
                rightPwm = 0
                self.IN3_status = False
                self.IN4_status = False
            elif rightPwm > self.speedPWMMin:
                if rightPwm > self.speedPWMMax:
                    right_pwm = self.speedPWMMax
                self.IN3_status = True
                self.IN4_status = False
            elif rightPwm < -self.speedPWMMin:
                rightPwm = abs(rightPwm)
                if rightPwm > self.speedPWMMax:
                    rightPwm = self.speedPWMMax
                self.IN3_status = False
                self.IN4_status = True

            # print(left_pwm, right_pwm, IN1_status, IN2_status, IN3_status, IN4_status, diff_direction)
            self.pwm_ENA.ChangeDutyCycle(leftPwm)
            self.pwm_ENB.ChangeDutyCycle(rightPwm)

            GPIO.output(self.IN1, self.IN1_status)
            GPIO.output(self.IN2, self.IN2_status)
            GPIO.output(self.IN3, self.IN3_status)
            GPIO.output(self.IN4, self.IN4_status)

    def motorReducedThread(self):
        """
        不按油门，速度下降
        """
        while True:
            newSpeed = 0
            self.speedLock.acquire()
            if self.speed > 0:
                newSpeed = self.speed - self.speedPWMStep
                if newSpeed <= 0:
                    newSpeed = 0
            elif self.speed < 0:
                newSpeed = self.speed + self.speedPWMStep
                if newSpeed >= 0:
                    newSpeed = 0
            self.speedLock.release()

            self.setSpeed(newSpeed)

    def goUp(self):
        """
        前进，加速
        """
        self.setSpeed(self.speed + self.speedPWMStep)

    def goDown(self):
        """
        后退，减速
        """
        self.setSpeed(self.speed - self.speedPWMStep)

    def goLeft(self):
        """
        左转
        """
        self.setDirection(self.goalDirection - self.directionStep)

    def goRight(self):
        """
        右转
        """
        self.setDirection(self.goalDirection + self.directionStep)

    def stop(self):

        self.setSpeed(0)
        self.setDirection(self.m6050.getCurrDirection())

    def setSpeed(self, speed):

        self.speedLock.acquire()

        if speed >= self.speedPWMMax:
            self.speed = self.speedPWMMax
        elif speed <= self.speedPWMMin:
            self.speed = self.speedPWMMin
        else:
            self.speed = speed

        self.speedLock.release()

        time.sleep(0.05)

    def setDirection(self, goalDirection):

        self.directionLock.acquire()
        self.goalDirection = goalDirection
        self.directionLock.release()

        time.sleep(0.05)


