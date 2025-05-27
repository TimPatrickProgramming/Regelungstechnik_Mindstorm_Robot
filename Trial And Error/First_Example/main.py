#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait

# Initialisierung
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
gyro = GyroSensor(Port.S2)

# Kalibrierung
gyro.reset_angle(0)

# PID-Parameter
Kp = 20
Kd = 40

# Zielwinkel
target_angle = 1

# Hauptregelkreis
while True:
    current_angle = gyro.angle()
    rate = gyro.speed()
    print(current_angle)
    
    error = current_angle - target_angle
    derivative = rate
    
    correction = -(Kp * error)
    
    left_motor.dc(correction)
    right_motor.dc(correction)
    
    wait(10)
