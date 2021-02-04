
class ServoCamera:
    def __init__(self, pwm):
        self.pwm = pwm
        self.lastMoveX = 0
        self.lastMoveY = 0
        self.pca9685ChannelMap = [0, 8] # 云台上的舵机1和舵机2分别对应PCA9685模块的0和8channel
        self.lastAngle = None
        self.cameraMinimum = [0, 80]  # 云台限位
        self.cameraMaximum = [180, 130]  # 云台限位

        self.servoInit()

    def servoInit(self):
        """
        摄像头云台初始化，设置云台初始角度
        """
        servoInit1 = 90 # 云台舵机1初始位置
        servoInit2 = 90 # 云台舵机2初始位置

        self.setServoAngle(self.pca9685ChannelMap[0], servoInit1)
        self.setServoAngle(self.pca9685ChannelMap[0], servoInit2)

        self.lastAngle = [servoInit1, servoInit2]

    def servoMove(self, moveX, moveY):
        """
        云台转动
        """
        # 云台防抖
        if abs(self.lastMoveX - moveX) <= 4:
            camera_move_x = 0
        if abs(self.lastMoveY - moveY) <= 4:
            camera_move_y = 0

        self.lastMoveX = moveX
        self.lastMoveY = moveY

        # P控制
        Kp0 = -1
        Kp1 = 1
        newAngle0 = Kp0 * moveX
        newAngle1 = Kp1 * moveY

        self.singleChannelMove(0, newAngle0)
        self.singleChannelMove(1, newAngle1)

    def singleChannelMove(self, servoId, newRelatedAngle):
        """
        驱动单个舵机转动
        """
        channelId = self.pca9685ChannelMap[servoId]
        absolutAngle = self.lastAngle[channelId] + newRelatedAngle

        # 云台限位
        if (absolutAngle < self.cameraMinimum[channelId]) or (absolutAngle > self.cameraMaximum[channelId]):
            print("angle exceed threadhold")
            return

        self.lastAngle[channelId] = absolutAngle
        self.setServoAngle(channelId, absolutAngle)

    def setServoAngle(self, channelId, absolutAngle):
        data = 0.5 + (absolutAngle / 180.0) * (2.5 - 0.5)
        data = int(data / 20 * 4096)

        self.pwm.set_pwm(channelId, 0, int(data))



