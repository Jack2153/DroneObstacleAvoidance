#!usr/bin/env python3
# Importing the required libraries
import subprocess
import time

# Using the system.Popen() method to execute shell commands

print("Starting the Simulator...")
subprocess.Popen("gz sim -v 4 -r iris_empty.sdf", shell=True)
# time.sleep() function delays the programm in seconds
time.sleep(3)
print("Starting the SITL...")
print("This may take a few moments")

# Launching SITL in a seperate shell
My_Cmmnd = "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --debug"

process = subprocess.Popen(
    "gnome-terminal -e 'bash -c \""+My_Cmmnd+";bash\"'",
    stdout=subprocess.PIPE,
    stderr=None,
    shell=True
)

# Launching the ros_gz_bridge for the front sensor in a seperate shell
My_Cmmnd = " ros2 run ros_gz_bridge parameter_bridge /lidar_front@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"

process = subprocess.Popen(
    "gnome-terminal -e 'bash -c \""+My_Cmmnd+";bash\"'",
    stdout=subprocess.PIPE,
    stderr=None,
    shell=True
)

# Launching the ros_gz_bridge for the back sensor in a seperate shell
My_Cmmnd = " ros2 run ros_gz_bridge parameter_bridge /lidar_back@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"

process = subprocess.Popen(
    "gnome-terminal -e 'bash -c \""+My_Cmmnd+";bash\"'",
    stdout=subprocess.PIPE,
    stderr=None,
    shell=True
)

# Launching the ros_gz_bridge for the right sensor in a seperate shell
My_Cmmnd = " ros2 run ros_gz_bridge parameter_bridge /lidar_right@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"

process = subprocess.Popen(
    "gnome-terminal -e 'bash -c \""+My_Cmmnd+";bash\"'",
    stdout=subprocess.PIPE,
    stderr=None,
    shell=True
)

# Launching the ros_gz_bridge for the left sensor in a seperate shell
My_Cmmnd = " ros2 run ros_gz_bridge parameter_bridge /lidar_left@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"

process = subprocess.Popen(
    "gnome-terminal -e 'bash -c \""+My_Cmmnd+";bash\"'",
    stdout=subprocess.PIPE,
    stderr=None,
    shell=True
)
