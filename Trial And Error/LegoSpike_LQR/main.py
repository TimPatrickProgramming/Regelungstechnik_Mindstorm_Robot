#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Color
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.tools import wait, StopWatch

# Initialisierung
ev3 = EV3Brick()
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
gyro_sensor = GyroSensor(Port.S2)

# Timer
single_loop_timer = StopWatch()
control_loop_timer = StopWatch()
fall_timer = StopWatch()

# Konstanten
GYRO_CALIBRATION_LOOP_COUNT = 200
GYRO_OFFSET_FACTOR = 0.0005
TARGET_LOOP_PERIOD = 20  # ms

# PID-Reglerwerte
Kp = 35
Ki = 4
Kd = 1.2

# Globale Variablen
integral = 0
prev_error = 0

while True:
    try:

        # Startanzeige
        ev3.screen.load_image(ImageFile.SLEEPING)
        ev3.light.off()
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        fall_timer.reset()

        control_loop_counter = 0
        robot_body_angle = -0.2

        # Gyro kalibrieren
        while True:
            gyro_min, gyro_max, gyro_sum = 440, -440, 0
            for _ in range(GYRO_CALIBRATION_LOOP_COUNT):
                val = gyro_sensor.speed()
                gyro_sum += val
                gyro_min = min(gyro_min, val)
                gyro_max = max(gyro_max, val)
                wait(5)
            if gyro_max - gyro_min < 2:
                break
        gyro_offset = gyro_sum / GYRO_CALIBRATION_LOOP_COUNT

        # Ready!
        ev3.speaker.play_file(SoundFile.SPEED_UP)
        ev3.screen.load_image(ImageFile.AWAKE)
        ev3.light.on(Color.GREEN)
        wait(500)

        # Balancing-Schleife mit einfachem PID
        while True:
            single_loop_timer.reset()

            # Durchschnittliche Loopzeit berechnen
            if control_loop_counter == 0:
                avg_loop_time = TARGET_LOOP_PERIOD / 1000
                control_loop_timer.reset()
            else:
                avg_loop_time = control_loop_timer.time() / 1000 / control_loop_counter
            control_loop_counter += 1

            # Kippwinkel berechnen
            gyro_val = gyro_sensor.speed()
            print(gyro_val)
            gyro_offset = (1 - GYRO_OFFSET_FACTOR) * gyro_offset + GYRO_OFFSET_FACTOR * gyro_val
            body_rate = gyro_val - gyro_offset
            robot_body_angle += body_rate * avg_loop_time

            # === PID-Regelung auf robot_body_angle ===
            error = 0 - robot_body_angle
            integral += error * avg_loop_time
            derivative = (error - prev_error) / avg_loop_time

            output_power = Kp * error + Ki * integral + Kd * derivative
            prev_error = error

            # Begrenzung
            output_power = max(min(output_power, 100), -100)

            # Motoren steuern
            left_motor.dc(output_power)
            right_motor.dc(output_power)

            # Fallerkennung
            # if abs(output_power) < 100:
            #     fall_timer.reset()
            # elif fall_timer.time() > 1000:
            #     break

            wait(TARGET_LOOP_PERIOD - single_loop_timer.time())

        # Fallreaktion
        # left_motor.stop()
        # right_motor.stop()
        # ev3.light.on(Color.RED)
        # ev3.screen.load_image(ImageFile.KNOCKED_OUT)
        # ev3.speaker.play_file(SoundFile.SPEED_DOWN)
        # wait(3000)

    except KeyboardInterrupt:
        left_motor.stop()
        right_motor.stop()
    break