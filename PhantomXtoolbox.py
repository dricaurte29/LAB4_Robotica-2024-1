import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH

# Definir los parámetros de los eslabones
params = [
    [11.3, 0, np.pi/2, 0],
    [0, 10.07, 0, np.pi/2],
    [0, 10.07, 0, 0],
    [0, 8.50, np.pi/2, 0]
]

# Lista para almacenar los eslabones
links = []

# Crear cada eslabón y añadirlo a la lista
for param in params:
    links.append(RevoluteDH(d=param[0], a=param[1], alpha=param[2], offset=param[3]))

# Crear el robot serial
robot = DHRobot(links)

# Asignar herramienta al robot
tool = np.array([[0, 0, 1, params[3][0]],
                 [-1, 0, 0, 0],
                 [0, -1, 0, 0],
                 [0, 0, 0, 1]])
robot.tool = tool

while True:
    # Pedir ángulos por consola
    joint_values = []
    for i in range(1, 5):
        while True:
            try:
                angle_input = float(input(f"Enter angle for joint {i} (in degrees): "))
                scaled_angle = np.deg2rad(angle_input)  # Convertir a radianes
                joint_values.append(scaled_angle)
                break
            except ValueError:
                print("Please enter a valid number.")

    # Mover el robot a las posiciones objetivo
    robot.q = joint_values
    robot.plot(robot.q)

