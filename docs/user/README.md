# User instruction
## Manual mode
'''
### necessary ones
ros2 run core_msg teleporter
### choose one of below
1. 
ros2 run keyboard keyboard_node  #keyboard control
2. 
ros2 run server server #webcontroller #then choose MANUAL on server
3.
joystick TBD
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