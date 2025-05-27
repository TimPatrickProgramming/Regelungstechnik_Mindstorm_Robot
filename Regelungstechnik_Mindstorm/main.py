#!/usr/bin/env pybricks-micropython

from ucollections import namedtuple
import urandom

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, ImageFile, SoundFile
from pybricks.tools import wait, StopWatch

# Initialization

# Initialising of Peripherals and Brick
ev3 = EV3Brick()
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
gyro = GyroSensor(Port.S2)

# Timers
loop_timer = StopWatch()
fall_timer = StopWatch()
control_timer = StopWatch()

# Constants

GYRO_CALIBRATION_COUNT = 200
GYRO_SMOOTHING = 0.0005
TARGET_LOOP_MS = 15

# PID control constants
Kp = 45
Ki = 0.36
Kd = 2.4

# Main balancing loop

while True:
    # Show orange light during gyro calibration
    ev3.light.on(Color.ORANGE)

    # Reset motor positions and timers
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    fall_timer.reset()

    # Initial internal state
    motor_angle_sum = 0
    wheel_angle = 0
    motor_delta_history = [0, 0, 0, 0]
    forward_speed = 0
    steering = 0
    loop_count = 0
    body_angle = -0.25

    # Gyro calibration: wait until robot is still 
    while True:
        min_gyro, max_gyro = 440, -440
        gyro_total = 0

        for _ in range(GYRO_CALIBRATION_COUNT):
            gyro_speed = gyro.speed()
            gyro_total += gyro_speed
            max_gyro = max(max_gyro, gyro_speed)
            min_gyro = min(min_gyro, gyro_speed)
            wait(5)

        # When range is narrow enough, assume stable
        if max_gyro - min_gyro < 2:
            break

    # Final gyro offset after calibration
    gyro_offset = gyro_total / GYRO_CALIBRATION_COUNT

    # Show green light to indicate ready
    ev3.light.on(Color.GREEN)

    # Balancing control loop 
    while True:
        loop_timer.reset()

        # Calculate smoothed loop duration
        if loop_count == 0:
            avg_loop_s = TARGET_LOOP_MS / 1000
            control_timer.reset()
        else:
            avg_loop_s = control_timer.time() / (1000 * loop_count)

        loop_count += 1

        # Gyro feedback 
        gyro_speed = gyro.speed()
        gyro_offset = (1 - GYRO_SMOOTHING) * gyro_offset + GYRO_SMOOTHING * gyro_speed
        body_speed = gyro_speed - gyro_offset
        body_angle += body_speed * avg_loop_s
        gyro_correction = -0.01 * forward_speed

        # Wheel feedback 
        left_angle = left_motor.angle()
        right_angle = right_motor.angle()
        prev_motor_sum = motor_angle_sum
        motor_angle_sum = left_angle + right_angle
        delta = motor_angle_sum - prev_motor_sum

        motor_delta_history.insert(0, delta)
        motor_delta_history.pop()

        wheel_angle += delta - forward_speed * avg_loop_s
        wheel_speed = sum(motor_delta_history) / (40 * avg_loop_s)
        body_speed += wheel_speed

        # Controller calculation
        power = gyro_correction + (Kd * body_speed + Kp * body_angle + Ki * wheel_angle)
        power = max(min(power, 100), -100)

        # Apply power with steering offset
        left_motor.dc(power - 0.1 * steering)
        right_motor.dc(power + 0.1 * steering)

        # Reset fall timer if under control
        if abs(power) < 100:
            fall_timer.reset()
        elif fall_timer.time() > 1000:
            break  # Robot likely fell over

        # Enforce consistent loop duration
        wait(TARGET_LOOP_MS - loop_timer.time())

    # Fall recovery

    left_motor.stop()
    right_motor.stop()

    # Show red light to indicate failure
    ev3.light.on(Color.RED)

    # Wait before restarting
    wait(3000)
