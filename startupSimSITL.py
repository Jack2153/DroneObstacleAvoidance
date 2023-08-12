#Importing the required libraries
import subprocess
import time

 #Using the system.Popen() method to execute shell commands

print("Starting the Simulator...")
subprocess.Popen("gz sim -v 4 -r iris_empty.sdf", shell=True)
#time.sleep() function delays the programm in seconds
time.sleep(3)
print("Starting the SITL...")
print("This may take a few moments")

My_Cmmnd = "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console"

process = subprocess.Popen(
    "gnome-terminal -e 'bash -c \""+My_Cmmnd+";bash\"'",
    stdout=subprocess.PIPE,
    stderr=None,
    shell=True
)