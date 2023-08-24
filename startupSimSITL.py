#Importing the required libraries
import subprocess
import time

#Using the system.Popen() method to execute shell commands

print("Starting the Simulator...")

My_Cmmnd1 = "gz sim -v 4 -r iris_empty.sdf"

process2 = subprocess.Popen(
    "gnome-terminal -e 'bash -c \""+My_Cmmnd1+";bash\"'",
    stdout=subprocess.PIPE,
    stderr=None,
    shell=True
)

#time.sleep() function delays the programm in seconds
time.sleep(3)

print("Starting the SITL...")
print("This may take a few moments")

My_Cmmnd2 = "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console"

process2 = subprocess.Popen(
    "gnome-terminal -e 'bash -c \""+My_Cmmnd2+";bash\"'",
    stdout=subprocess.PIPE,
    stderr=None,
    shell=True
)

print("Starting the ROS_gz bridge)

My_Cmmnd3 = "ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"

process2 = subprocess.Popen(
    "gnome-terminal -e 'bash -c \""+My_Cmmnd3+";bash\"'",
    stdout=subprocess.PIPE,
    stderr=None,
    shell=True
)
