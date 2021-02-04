# 角度变量
aax = 0
aay = 0
aaz = 0
agx = 0
agy = 0
agz = 0

# 加速度计偏移量
axo = 0
ayo = 0
azo = 0;
# 陀螺仪偏移量
gxo = 0
gyo = 0
gzo = 0

last_time = 0
AcceRatio = 16384.0  # 加速度计比例系数
GyroRatio = 131.0  # 陀螺仪比例系数
n_sample = 8  # 加速度计滤波算法采样个数

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
    gyrox = -(gx - gxo) / GyroRatio * diff_time  # x轴角速度
    agx += gyrox  # x轴角速度积分

    return agx


def mpu6050_thread():
    mpu6050_init()
    print("mpu6050_thread start")
    while True:
        angle = get_angle()
        # print(angle)
        angle_queue.put(angle)