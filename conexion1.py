#!/usr/bin/env python

import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand

# Definir límites articulares
joint_limits = {
    1: (10, 1000),
    2: (204, 820),
    3: (100, 920),
    4: (155, 860),
    5: (50, 450)
}

def call_service(command, id, addr_name, value):
    rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
    try:
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        response = dynamixel_command('', id, addr_name, value)
        return response.comm_result
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def check_limits(id, value):
    if joint_limits[id][0] <= value <= joint_limits[id][1]:
        return True
    else:
        print(f"Error: Value {value} for motor {id} is out of limits {joint_limits[id]}")
        return False

def main():
    rospy.init_node('dynamixel_control', anonymous=True)
    
    # Habilitar Torques
    ids = [1, 2, 3, 4]
    for motor_id in ids:
        call_service('', motor_id, 'Torque_Enable', 1)
    
    # HOME----------------------------------------------
    home_positions = {
        1: 512,
        2: 512,
        3: 512,
        4: 512,
        5: 450
    }
    for motor_id, position in home_positions.items():
        if check_limits(motor_id, position):
            call_service('', motor_id, 'Moving_Speed', 200)
            call_service('', motor_id, 'Goal_Position', position)
            rospy.sleep(2)
    
    # OBJETIVO --------------------------------------------------------------
    target_positions = {
        1: 820,
        4: 650,
        2: 650,
        3: 820,
        5: 300
    }
    for motor_id, position in target_positions.items():
        speed = 250 if motor_id == 5 else 200
        if check_limits(motor_id, position):
            call_service('', motor_id, 'Moving_Speed', speed)
            call_service('', motor_id, 'Goal_Position', position)
            rospy.sleep(2)

if __name__ == '__main__':
    main()

