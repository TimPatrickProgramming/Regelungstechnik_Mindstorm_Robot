#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Color
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

# Defining Constants
GYRO_CALIBRATION_LOOP_COUNT = 200
GYRO_OFFSET_FACTOR = 0.0005
TARGET_LOOP_PERIOD = 15  # ms

# PID Parameters (adjusted for stable behavior)
K_P = 15
K_I = 0.05
K_D = 0.8

while True:
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

    # wait for upright still position to calibrate Gyro
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

    ev3.light.on(Color.GREEN)

    integral_error = 0
    previous_error = 0

    while True:
        single_loop_timer.reset()

        if control_loop_count == 0:
            average_control_loop_period = TARGET_LOOP_PERIOD / 1000
            control_loop_timer.reset()
        else:
            average_control_loop_period = (control_loop_timer.time() / 1000 /
                                           control_loop_count)
        control_loop_count += 1

        # compute robot body angle and rate
        gyro_sensor_value = gyro_sensor.speed()
        gyro_offset *= (1 - GYRO_OFFSET_FACTOR)
        gyro_offset += GYRO_OFFSET_FACTOR * gyro_sensor_value
        robot_body_rate = gyro_sensor_value - gyro_offset
        robot_body_angle += robot_body_rate * average_control_loop_period

        # PID Controller
        error = -robot_body_angle
        integral_error += error * average_control_loop_period
        derivative_error = -robot_body_rate

        output_power = (K_P * error) + (K_I * integral_error) + (K_D * derivative_error)

        if output_power > 100:
            output_power = 100
        if output_power < -100:
            output_power = -100

        left_motor.dc(output_power)
        right_motor.dc(output_power)

        if abs(output_power) < 100:
            fall_timer.reset()
        elif fall_timer.time() > 1000:
            break

        wait(TARGET_LOOP_PERIOD - single_loop_timer.time())

    # When the robot falls over
    left_motor.stop()
    right_motor.stop()
    ev3.light.on(Color.RED)
    wait(3000)
