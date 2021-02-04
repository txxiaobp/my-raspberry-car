
camera_1_init = 90
camera_2_init = 90

camera_channel = [0, 8]
last_angle = [camera_1_init, camera_2_init]

camera_minimum = [0, 80]
camera_maximum = [180, 130]


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

def set_servo_angle(channel, angle):
	data = 0.5 + (angle/180.0)*(2.5 - 0.5)
	data = int(data / 20 * 4096)

	pwm.set_pwm(channel, 0, int(data))

def camera_init():
	set_servo_angle(camera_channel[0], camera_1_init)
	set_servo_angle(camera_channel[0], camera_2_init)