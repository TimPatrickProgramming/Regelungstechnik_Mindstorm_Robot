import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

# Strecke definieren
M = 0.04   # Masse des Wagens (kg)
m = 0.6    # Masse des Pendels (kg)
b = 1      # Dämpfung am Wagen (Ns/m)
I = 0.0031 # Trägheitsmoment des Pendels (kg·m²)
l = 0.1    # Länge zum Schwerpunkt des Pendels (m)
g = 9.81   # Erdbeschleunigung (m/s²)

# Hilfsgröße
denominator_factor = (M + m)*(I + m*l**2) - (m*l)**2

# Nennerkoeffizienten (s³ + a*s² + b*s + c)
a = b*(I + m*l**2) / denominator_factor
b_coeff = - (M + m)*m*g*l / denominator_factor
c = - b*m*g*l / denominator_factor

# Zählerkoeffizienten
numerator = [0, m*l, 0, 0]

# Nenner
denominator = [1, a, b_coeff, c]

# Übertragungsfunktion
plant = ctrl.TransferFunction(numerator, denominator)

# PID-Regler definieren
Kp = 45
Ki = 0.36
Kd = 2.4
controller = ctrl.TransferFunction([Kd, Kp, Ki], [1, 0])

# Offene Strecke
open_loop = ctrl.series(controller, plant)

# Bode-Plot
mag, phase, omega = ctrl.bode(open_loop, dB=True, deg=True, omega_limits=(1e-2, 1e5), plot=True)
plt.suptitle("Open Loop Bode Plot", fontsize=14)
plt.show()