#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait, StopWatch

# Initialisiere EV3 Brick und Geräte
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
gyro = GyroSensor(Port.S2)

# Kalibriere den Gyrosensor
gyro.reset_angle(0)

# PID-Regler-Parameter
k_p = 35
k_i = 4
k_d = 1.2

# Zielwinkel
target_angle = 0

# Initialisiere PID-Variablen
integral = 0
previous_error = 0

# Steuerparameter
power = 1
fs = 70  # Abtastrate in Hz
dt = 1000 // fs  # Abtastzeit in ms

# Hauptregelkreis
while True:
    # Aktuellen Winkel vom Gyrosensor lesen
    current_angle = gyro.angle()
    print(current_angle)

    # Fehler berechnen
    error = target_angle - current_angle

    # Integral berechnen
    integral += error * (dt / 1000)

    # Ableitung berechnen
    derivative = (error - previous_error) / (dt / 1000)

    # PID-Regelung berechnen
    result_power = k_p * error + k_i * integral + k_d * derivative

    # Motorleistung berechnen
    movement_speed = int(-result_power * power)

    # Motoren ansteuern
    left_motor.run(movement_speed)
    right_motor.run(movement_speed)

    # Vorherigen Fehler aktualisieren
    previous_error = error

    # Wartezeit
    wait(dt)
