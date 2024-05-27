#!/bin/bash

#Habilitar Torques
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 1
addr_name: 'Torque_Enable'
value: 1"
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 2
addr_name: 'Torque_Enable'
value: 1"
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 3
addr_name: 'Torque_Enable'
value: 1"
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 4
addr_name: 'Torque_Enable'
value: 1"

#HOME----------------------------------------------


#Motor 1
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 1
addr_name: 'Moving_Speed'
value: 200"
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 1
addr_name: 'Goal_Position'
value: 512"
sleep 2

#Motor 2
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 2
addr_name: 'Moving_Speed'
value: 200"
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 2
addr_name: 'Goal_Position'
value: 512"
sleep 2

#Motor 3
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 3
addr_name: 'Moving_Speed'
value: 200"
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 3
addr_name: 'Goal_Position'
value: 512"
sleep 2

#Motor 4
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 4
addr_name: 'Moving_Speed'
value: 200"
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 4
addr_name: 'Goal_Position'
value: 512"
sleep 2

#Motor 5
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 5
addr_name: 'Moving_Speed'
value: 200"
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 5
addr_name: 'Goal_Position'
value: 512"
sleep 2

#OBJETIVO --------------------------------------------------------------

#Motor 1
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 1
addr_name: 'Moving_Speed'
value: 200"
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 1
addr_name: 'Goal_Position'
value: 820"
sleep 2

#Motor 4
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 4
addr_name: 'Moving_Speed'
value: 200"
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 4
addr_name: 'Goal_Position'
value: 650"
sleep 2

#Motor 2
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 2
addr_name: 'Moving_Speed'
value: 200"
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 2
addr_name: 'Goal_Position'
value: 650"
sleep 2

#Motor 3
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 3
addr_name: 'Moving_Speed'
value: 200"
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 3
addr_name: 'Goal_Position'
value: 820"
sleep 2

#Motor 5
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 5
addr_name: 'Moving_Speed'
value: 250"
rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 5
addr_name: 'Goal_Position'
value: 204"
sleep 2

