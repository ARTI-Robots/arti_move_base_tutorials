# arti_move_base_tutorials

## Prerquisits
Arti move base ( https://github.com/ARTI-Robots/arti_move_base )

Arti public stanley controller ( https://github.com/ARTI-Robots/stanley_control_path_follower)

teb-local-planer (sudo apt-get install ros-$(rosversion -d)-teb-local-planner)

sudo apt-get install ros-$(rosversion -d)-turtlesim


## start

roslaunch arti_move_base_example example_project_launch.launch 


## Troubleshooting
check that the script/odom_republisher.py is executable
