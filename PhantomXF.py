#!/usr/bin/env python

# Librerias
import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand, DynamixelCommandResponse
from dynamixel_workbench_msgs.msg import DynamixelStateList
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
import tkinter as tk
from tkinter import messagebox, StringVar, OptionMenu

# DEFINIR MODELO DEL ROBOT -------------------------------------------
# Definir parámetros de los eslabones
params = [
    [11.3, 0, -np.pi/2, 0],
    [0, 10.07, 0, -np.pi/2],
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
tool = np.array([[1, 0, 0, params[3][0]],
                 [0, 0, 1, 0],
                 [0, -1, 0, 0],
                 [0, 0, 0, 1]])
robot.tool = tool

# Definir límites articulares
joint_limits = {
    1: (10, 1000),
    2: (204, 820),
    3: (100, 920),
    4: (155, 860),
    5: (50, 450)
}

# Posiciones base
base_positions = [
    [0, 0, 0, 0, 0],
    [25, 25, 20, -20, 0],
    [-35, 35, -30, 30, 0],
    [85, -20, 55, 25, 0],
    [80, -35, 55, -45, 0]
]

# FUNCIONES ---------------------------------------------------------
# Servicios del robot
def call_service(command, id, addr_name, value):
    rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
    try:
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        response = dynamixel_command('', id, addr_name, value)
        return response.comm_result
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

# Checkear limites articulares
def check_limits(id, value):
    if joint_limits[id][0] <= value <= joint_limits[id][1]:
        return True
    else:
        print(f"Error: Value {value} for motor {id} is out of limits {joint_limits[id]}")
        return False

# Obtener posiciones actuales de las articulaciones
def get_joint_positions():
    rospy.wait_for_message('/dynamixel_workbench/dynamixel_state', DynamixelStateList)
    state_msg = rospy.wait_for_message('/dynamixel_workbench/dynamixel_state', DynamixelStateList)
    positions = {state.id: state.present_position for state in state_msg.dynamixel_state}
    return positions

# Funcion principal
def main():
    rospy.init_node('dynamixel_control', anonymous=True)
    
    # Definir posiciones de inicio (home)
    home_positions = [512, 512, 512, 512, 450]
    
    # Habilitar Torques
    ids = [1, 2, 3, 4]
    for motor_id in ids:
        call_service('', motor_id, 'Torque_Enable', 1)
    
    # Crear ventana de Tkinter
    root = tk.Tk()
    root.title("Control de Robot")
    
    # Variable para seleccionar posición base
    selected_base_position = StringVar(root)
    selected_base_position.set("Seleccionar posición base")

    # Entradas para los ángulos de las articulaciones
    entries = []
    for i in range(1, 6):
        frame = tk.Frame(root)
        frame.pack(padx=10, pady=5)
        label = tk.Label(frame, text=f"Ángulo para articulación {i}:")
        label.pack(side=tk.LEFT)
        entry = tk.Entry(frame)
        entry.pack(side=tk.LEFT)
        entries.append(entry)
    
    def update_entries(*args):
        selection = selected_base_position.get()
        if selection != "Ingresar posición propia":
            position_index = int(selection.split()[1]) - 1
            position_values = base_positions[position_index]
            for i, value in enumerate(position_values):
                entries[i].delete(0, tk.END)
                entries[i].insert(0, value)
        else:
            for entry in entries:
                entry.delete(0, tk.END)

    selected_base_position.trace('w', update_entries)

    option_menu = OptionMenu(root, selected_base_position, "Seleccionar posición base",
                             "Posición 1", "Posición 2", "Posición 3", "Posición 4", "Posición 5",
                             "Ingresar posición propia")
    option_menu.pack(pady=20)

    def update_current_angles():
        positions = get_joint_positions()
        for i in range(5):
            if i + 1 in positions:
                current_angle = positions[i + 1] / 3.41332 - home_positions[i]
                if i == 4:
                    current_angle -= 42
                if i == 3:
                    current_angle += 2
                current_angles_labels[i].config(text=f"Ángulo actual de articulación {i + 1}: {int(current_angle) + 360}")
            else:
                current_angles_labels[i].config(text=f"Ángulo actual de articulación {i + 1}: No disponible")

    def on_submit():
        target_positions = {}
        joint_values = []
        
        try:
            for i in range(5):
                angle_input = int(entries[i].get())
                model_angle = np.deg2rad(angle_input) 
                scaled_angle = (angle_input * 3.41332) + home_positions[i]
                if check_limits(i + 1, scaled_angle):
                    joint_values.append(model_angle)
                    target_positions[i + 1] = scaled_angle
                else:
                    messagebox.showerror("Error", f"El valor {scaled_angle} para la articulación {i + 1} está fuera de los límites.")
                    return
        except ValueError:
            messagebox.showerror("Error", "Por favor, ingrese un entero válido.")
            return
        
        joint_values.pop()
        # Mover modelo a las posiciones objetivo
        robot.q = joint_values
        robot.plot(robot.q)
        
        # Mover robot a las posiciones objetivo
        for motor_id, position in target_positions.items():
            speed = 250 if motor_id == 5 else 200
            position = int(position)
            if check_limits(motor_id, position):
                call_service('', motor_id, 'Moving_Speed', speed)
                call_service('', motor_id, 'Goal_Position', position)
                rospy.sleep(2)
                update_current_angles()  # Actualizar ángulos actuales después de cada movimiento individual

    # Botón para enviar los datos
    submit_button = tk.Button(root, text="Enviar", command=on_submit)
    submit_button.pack(pady=20)
    
    # Sección para mostrar ángulos actuales
    current_angles_frame = tk.Frame(root)
    current_angles_frame.pack(padx=10, pady=10)
    current_angles_labels = []
    for i in range(1, 6):
        label = tk.Label(current_angles_frame, text=f"Ángulo actual de articulación {i}: Desconocido")
        label.pack()
        current_angles_labels.append(label)

    root.mainloop()

if __name__ == '__main__':
    main()