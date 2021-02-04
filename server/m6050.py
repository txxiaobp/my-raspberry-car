from mpu6050 import mpu6050
import queue
from datetime import datetime
import time
import threading


GyroRatio = 131.0  # 陀螺仪比例系数


class MPU6050:
    def __init__(self):
        self.sensor = None

        # 加速度计偏移量
        self.axo = 0
        self.ayo = 0
        self.azo = 0

        # 陀螺仪偏移量
        self.gxo = 0
        self.gyo = 0
        self.gzo = 0

        # 角度变量
        self.aax = 0
        self.aay = 0
        self.aaz = 0
        self.agx = 0
        self.agy = 0
        self.agz = 0

        # 上一次获取角度的时刻
        self.lastTime = 0
        self.angleQueue = queue.Queue()

        self.mpu6050Init()


    def mpu6050Init(self):
        """
        mpu6050初始化
        """
        self.sensor = mpu6050(0x68)
        time.sleep(0.1) # 等待mpu6050初始化完成

        # 采样次数
        samplingTimes = 500
        for i in range(0, samplingTimes):
            gx, gy, gz, ax, ay, az = self.getRawData()
            # 采样和
            self.axo += ax
            self.ayo += ay
            self.azo += az
            self.gxo += gx
            self.gyo += gy
            self.gzo += gz

        # 计算加速度计偏移
        self.axo /= samplingTimes
        self.ayo /= samplingTimes
        self.azo /= samplingTimes
        # 计算陀螺仪偏移
        self.gxo /= samplingTimes
        self.gyo /= samplingTimes
        self.gzo /= samplingTimes

        # goal_direction = self.getAngle()

        mpu6050Thread = threading.Thread(target=MPU6050.mpu6050Thread, args=(self,))
        mpu6050Thread.setDaemon(True)
        mpu6050Thread.start()

    def getCurrDirection(self):
        """
        通过mpu6050获取当前方向
        """
        samplingTimes = 5

        sum = 0
        for i in range(samplingTimes):
            sum += self.angleQueue.get()

        return sum / samplingTimes

    def getRawData(self):
        """
        获取mpu6050原始数据
        """
        accel_data = self.sensor.get_accel_data()
        gyro_data = self.sensor.get_gyro_data()

        gx = gyro_data['x']
        gy = gyro_data['y']
        gz = gyro_data['z']
        ax = accel_data['x']
        ay = accel_data['y']
        az = accel_data['z']

        return gx, gy, gz, ax, ay, az

    def getAngle(self):
        """
        获取当前角度
        """
        currentTime = datetime.now().microsecond
        diffTime = (currentTime - self.lastTime) / 1000.0
        self.lastTime = currentTime

        gx, gy, gz, ax, ay, az = self.getRawData()
        gyrox = -(gx - self.gxo) / GyroRatio * diffTime  # x轴角速度
        self.agx += gyrox  # x轴角速度积分

        return self.agx

    def mpu6050Thread(self):
        while True:
            angle = self.getAngle()
            self.angleQueue.put(angle)