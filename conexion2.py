#!/usr/bin/env python

import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from sensor_msgs.msg import JointState

# Definir límites articulares
joint_limits = {
    1: (10, 1000),
    2: (204, 820),
    3: (100, 920),
    4: (155, 860),
    5: (50, 450)
}

def call_service(id, addr_name, value):
    rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
    try:
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        response = dynamixel_command('', id, addr_name, value)
        if not response.comm_result:
            print(f"Failed to send command to motor {id}, address {addr_name}, value {value}")
        return response.comm_result
    except rospy.ServiceException as e:
        print(f"Service call failed for motor {id}, address {addr_name}, value {value}: {e}")
        return False

def check_limits(id, value):
    if joint_limits[id][0] <= value <= joint_limits[id][1]:
        return True
    else:
        print(f"Error: Value {value} for motor {id} is out of limits {joint_limits[id]}")
        return False

def joint_states_callback(msg):
    # Imprimir el nombre de la articulación y su posición
    for i, name in enumerate(msg.name):
        print(f"Joint {name}: {msg.position[i]:.2f}")

def subscribe_to_joint_topics():
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, joint_states_callback)

def main():
    rospy.init_node('dynamixel_control', anonymous=True)
    
    # Suscribirse a los tópicos de cada articulación
    subscribe_to_joint_topics()

    # Habilitar Torques
    ids = [1, 2, 3, 4, 5]
    for motor_id in ids:
        if not call_service(motor_id, 'Torque_Enable', 1):
            rospy.logerr(f"Failed to enable torque for motor {motor_id}")
            return
    
    # HOME----------------------------------------------
    home_positions = {
        1: 512,
        2: 512,
        3: 512,
        4: 512,
        5: 450  # Cambiado de 512 a 450
    }
    for motor_id, position in home_positions.items():
        if check_limits(motor_id, position):
            if not call_service(motor_id, 'Moving_Speed', 200):
                rospy.logerr(f"Failed to set moving speed for motor {motor_id}")
                return
            if not call_service(motor_id, 'Goal_Position', position):
                rospy.logerr(f"Failed to set goal position for motor {motor_id}")
                return
            rospy.sleep(2)
    
    # OBJETIVO --------------------------------------------------------------
    target_positions = {
        1: 820,
        4: 650,
        2: 650,
        3: 820,
        5: 204
    }
    for motor_id, position in target_positions.items():
        speed = 250 if motor_id == 5 else 200
        if check_limits(motor_id, position):
            if not call_service(motor_id, 'Moving_Speed', speed):
                rospy.logerr(f"Failed to set moving speed for motor {motor_id}")
                return
            if not call_service(motor_id, 'Goal_Position', position):
                rospy.logerr(f"Failed to set goal position for motor {motor_id}")
                return
            rospy.sleep(2)
    
    # Mantener el nodo vivo para seguir recibiendo datos de los tópicos
    rospy.spin()

if __name__ == '__main__':
    main()

