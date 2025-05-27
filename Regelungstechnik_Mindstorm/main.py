#!/usr/bin/env pybricks-micropython

from ucollections import namedtuple
import urandom

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, ImageFile, SoundFile
from pybricks.tools import wait, StopWatch

# Initialize the EV3 brick.
ev3 = EV3Brick()

# Initialize the motors connected to the drive wheels.
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

# Initialize the gyro sensor. It is used to provide feedback for balancing the robot.
gyro_sensor = GyroSensor(Port.S2)

# Initialize the timers.
fall_timer = StopWatch()
single_loop_timer = StopWatch()
control_loop_timer = StopWatch()
# action_timer = StopWatch()

# Defining Constants
GYRO_CALIBRATION_LOOP_COUNT = 200
GYRO_OFFSET_FACTOR = 0.0005
TARGET_LOOP_PERIOD = 15  # ms
K_P = 45
K_I = 0.36
K_D = 2.4
K_G = -0.01

while True:
    # During Callibration show Sleeping face to let user know callibration is still ongoing
    ev3.light.on(Color.ORANGE)

    # Reset the sensors and variables.
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    fall_timer.reset()

    motor_position_sum = 0
    wheel_angle = 0
    motor_position_change = [0, 0, 0, 0]
    drive_speed, steering = 0, 0
    control_loop_count = 0
    robot_body_angle = -0.25

    # wait til Robot is not moving anymore and standing in an upgright position before callibrating the Gyro
    while True:
        gyro_minimum_rate, gyro_maximum_rate = 440, -440
        gyro_sum = 0
        for _ in range(GYRO_CALIBRATION_LOOP_COUNT):
            gyro_sensor_value = gyro_sensor.speed()
            gyro_sum += gyro_sensor_value
            if gyro_sensor_value > gyro_maximum_rate:
                gyro_maximum_rate = gyro_sensor_value
            if gyro_sensor_value < gyro_minimum_rate:
                gyro_minimum_rate = gyro_sensor_value
            wait(5)
        if gyro_maximum_rate - gyro_minimum_rate < 2:
            break
    gyro_offset = gyro_sum / GYRO_CALIBRATION_LOOP_COUNT

    # Awake eyes and green light shows that callibration has ended and Robot is good to go
    ev3.light.on(Color.GREEN)

    # Main control loop for balancing the robot.
    while True:
        # Timer to messure loop time and keep it consistent
        single_loop_timer.reset()

        # Calculate the average controle loop periode, helps filtering out random fluctuations during controll feedback callculation
        if control_loop_count == 0:
            # Assign Value in first loop to avoid dividing by zero
            average_control_loop_period = TARGET_LOOP_PERIOD / 1000
            control_loop_timer.reset()
        else:
            average_control_loop_period = (control_loop_timer.time() / 1000 /
                                           control_loop_count)
        control_loop_count += 1

        # calculate robot body angle and speed
        gyro_sensor_value = gyro_sensor.speed()
        gyro_offset *= (1 - GYRO_OFFSET_FACTOR)
        gyro_offset += GYRO_OFFSET_FACTOR * gyro_sensor_value
        robot_body_rate = gyro_sensor_value - gyro_offset
        robot_body_angle += robot_body_rate * average_control_loop_period

        # calculate wheel angle and speed
        left_motor_angle = left_motor.angle()
        right_motor_angle = right_motor.angle()
        previous_motor_sum = motor_position_sum
        motor_position_sum = left_motor_angle + right_motor_angle
        change = motor_position_sum - previous_motor_sum
        motor_position_change.insert(0, change)
        del motor_position_change[-1]
        wheel_angle += change - drive_speed * average_control_loop_period
        wheel_rate = sum(motor_position_change) / 40 / average_control_loop_period

        # This is the main control feedback calculation.
        output_power = (K_G * drive_speed) + (K_D * (robot_body_rate + wheel_rate) + K_P * robot_body_angle + K_I * wheel_angle)
        print(wheel_rate)
        
        if output_power > 100:
            output_power = 100
        if output_power < -100:
            output_power = -100

        # Drive the motors.
        left_motor.dc(output_power - 0.1 * steering)
        right_motor.dc(output_power + 0.1 * steering)

        # If output speed is +/-100% for longer than one second the roboter most likely fell over
        if abs(output_power) < 100:
            fall_timer.reset()
        elif fall_timer.time() > 1000:
            break

        # Enshuring the loop is long enough, so all actions have enough time to act
        wait(TARGET_LOOP_PERIOD - single_loop_timer.time())

    # Handling of Roboter falling over

    # Stop all of the motors.
    left_motor.stop()
    right_motor.stop()

    # Kocked out eyes and red light let user know the robot fell over
    ev3.light.on(Color.RED)

    # Wait for 3 seconds before trying to balance again.
    wait(3000)