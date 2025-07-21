# User instruction
## Manual mode

'''
ros2 run core_msg teleporter # necessary one
ros2 run keyboard keyboard_node  #keyboard control
ros2 run server server #webcontroller #then choose MANUAL on server
#joystick TBD
'''

## Auto mode
'''
ros2 run ros2 run server server #choose AUTO on server #task TBD
ros2 run core_msg teleporter
'''

## Follow mode
'''
ros2 run ros2 run server server #choose FOLLOW on server #task TBD
ros2 run core_msg teleporter
'''