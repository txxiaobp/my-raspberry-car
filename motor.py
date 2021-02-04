
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

pwm_ENA = None
pwm_ENB = None

left_pwm = 0
right_pwm = 0

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

def motor_thread():
    global goal_direction
    global left_pwm
    global right_pwm

    minimum_pwm = 10
    maximum_pwm = 100
    Kp = -0.1

    while True:
        current_direction = get_current_direction()
        diff_direction = current_direction - goal_direction
        if abs(diff_direction) < 5:
            diff_direction = 0

        if left_pwm > 0:
            left_pwm += Kp * diff_direction
        elif left_pwm < 0:
            left_pwm -= Kp * diff_direction

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

        if right_pwm > 0:
            right_pwm -= Kp * diff_direction
        elif right_pwm < 0:
            right_pwm += Kp * diff_direction

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
        print(left_pwm, right_pwm, IN1_status, IN2_status, IN3_status, IN4_status, diff_direction)
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
        if left_pwm != 0:
            left_pwm = max(abs(left_pwm) - 1, 0)
        if right_pwm != 0:
            right_pwm = max(abs(right_pwm) - 1, 0)
        time.sleep(0.5)


def upCallBack():
    global left_pwm
    global right_pwm
    global goal_direction
    global IN1_status
    global IN2_status
    global IN3_status
    global IN4_status

    if left_pwm == 0 and right_pwm == 0:
        left_pwm = 15
        right_pwm = 15
        goal_direction = angle_queue.get();

        IN1_status = True
        IN2_status = False
        IN3_status = True
        IN4_status = False
        return

    if IN1_status == True and IN2_status == False:
        left_pwm = min(left_pwm + 1, 100)
    elif IN1_status == False and IN2_status == True:
        left_pwm = max(left_pwm - 1, 0)

    if IN3_status == True and IN4_status == False:
        right_pwm = min(right_pwm + 1, 100)
    elif IN3_status == False and IN4_status == True:
        right_pwm = max(right_pwm - 1, 0)

    time.sleep(0.05)


def downCallBack():
    global left_pwm
    global right_pwm
    global goal_direction
    global IN1_status
    global IN2_status
    global IN3_status
    global IN4_status

    if left_pwm == 0 and right_pwm == 0:
        left_pwm = 15
        right_pwm = 15
        goal_direction = angle_queue.get();

        IN1_status = False
        IN2_status = True
        IN3_status = False
        IN4_status = True
        return

    if IN1_status == False and IN2_status == True:
        left_pwm = min(left_pwm + 1, 100)
    elif IN1_status == True and IN2_status == False:
        left_pwm = max(left_pwm - 1, 0)

    if IN3_status == False and IN4_status == True:
        right_pwm = min(right_pwm + 1, 100)
    elif IN3_status == True and IN4_status == False:
        right_pwm = max(right_pwm - 1, 0)

    time.sleep(0.05)


def leftCallBack():
    global left_pwm
    global goal_direction
    left_pwm = max(0, left_pwm - 10)
    time.sleep(0.01)
    left_pwm = min(100, left_pwm + 10)
    time.sleep(0.01)
    goal_direction = angle_queue.get()


def rightCallBack():
    global right_pwm
    global goal_direction
    right_pwm = max(0, right_pwm - 10)
    time.sleep(0.01)
    right_pwm = min(100, right_pwm + 10)
    time.sleep(0.01)
    goal_direction = angle_queue.get()


def haltCallBack():
    global left_pwm
    global right_pwm
    global goal_direction
    global IN1_status
    global IN2_status
    global IN3_status
    global IN4_status

    left_pwm = 0
    right_pwm = 0
    goal_direction = angle_queue.get()
    IN1_status = False
    IN2_status = False
    IN3_status = False
    IN4_status = False


def motor_ouput_init():
    global pwm_ENA
    global pwm_ENB

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)  # 使用BCM编号方式
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