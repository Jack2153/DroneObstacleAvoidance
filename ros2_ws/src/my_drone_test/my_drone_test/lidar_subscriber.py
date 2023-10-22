#!usr/bin/env python3
import random
import rclpy
import argparse
import math
import time
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550') # Specify IP-address for the simulator
args = parser.parse_args()

# Connect to the vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=False)

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print ("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True


    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Check that vehicle has reached takeoff altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Function to get the new location (longitude and latitude) based on an original location and a north and east movement given in meters
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 # Radius of "spherical" earth
    
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    # Give back the same object as the input was
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon, original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon, original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
    
    # Return target location, the new longitude and latitude for the target Location
    return targetlocation, newlon, newlat

# Function to get the distance in meters between two locations
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

# Function to move the vehicle north a certain amount of meters north and east
def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """

    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)[0]
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance
    
    time.sleep(20)



# Print the location for debugging purposes
def printLocation():
    print ("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print ("Altitude relative to home_location: %s" % vehicle.location.global_relative_frame.alt)

# ROS Node that implements the floodfill algorithm and movement of the drone
class FloodFillNode(Node):

    # Drone step to count the movements in the movement map
    robot_step_count = 1
    def __init__(self):
        super().__init__('flood_fill_node')

        # Create a subscriber that listens to the /sensor_right topic of ROS2 where the right lidar sesnor publishes the data 
        self.subscription_right = self.create_subscription(
            LaserScan,
            '/sensor_right',
            self.lidar_right_callback,
            10)

        # Create a subscriber that listens to the /sensor_left topic of ROS2 where the left lidar sesnor publishes the data 
        self.subscription_left = self.create_subscription(
            LaserScan,
            '/sensor_left',
            self.lidar_left_callback,
            10)
        
        # Create a subscriber that listens to the /sensor_back topic of ROS2 where the back lidar sesnor publishes the data 
        self.subscription_back = self.create_subscription(
            LaserScan,
            '/sensor_back',
            self.lidar_back_callback,
            10)
        
        # Create a subscriber that listens to the /sensor_front topic of ROS2 where the front lidar sesnor publishes the data 
        self.subscription_front = self.create_subscription(
            LaserScan,
            '/sensor_front',
            self.lidar_front_callback,
            10)
        
        # Initialize all the maps
        self.map_distance = np.zeros((9, 9), dtype=int)
        self.map_movement = np.zeros((9, 9), dtype=int)
        self.map_obstacles = np.zeros((9, 9), dtype=int)

        # Position the robot in the middle of the map
        self.robot_x = 4 
        self.robot_y = 4 

        # Set the target altitude to 4 meters
        aTargetAltitude = float(4)

        # Only execute arm_and_takeoff if the current altitude of the drone ist below 90% of the target altitude
        if vehicle.location.global_relative_frame.alt <= aTargetAltitude * 0.9:
            arm_and_takeoff(aTargetAltitude)

        # Save the y and y coordinates of the target
        self.target_x, self.target_y = self.set_target()


    # Set the target via use input 
    def set_target(self):
        target_x = int(input("Bitte geben Sie die X-Koordinate des Zielpunkts ein: "))
        target_y = int(input("Bitte geben Sie die Y-Koordinate des Zielpunkts ein: "))
        return target_x, target_y

    # Callback function that is executed everytime the front lidar sensor meassures a distance
    def lidar_front_callback(self, msg):
        distance_to_obstacle = msg.ranges[0] # Take the first meassurement as the lidar sensor behaves the same way as a proximity sensor that only gives back one distance value for one meassurement
    
        if distance_to_obstacle != float('inf'):  # Check for infinite values, when the value is not infinite there is an bstacle
            distance_to_obstacle = int(round(distance_to_obstacle)) # Convert the distance to obstacle into an integer if it is not 'inf'
            self.add_obstacle(distance_to_obstacle, sensor="front") # Add an ostacle to the map with the right distance
        self.main_program()

    # Callback function that is executed everytime the right lidar sensor meassures a distance
    def lidar_right_callback(self, msg):
        distance_to_obstacle = msg.ranges[0] # Take the first meassurement as the lidar sensor behaves the same way as a proximity sensor that only gives back one distance value for one meassurement
    
        if distance_to_obstacle != float('inf'):  # Check for infinite values, when the value is not infinite there is an bstacle
            distance_to_obstacle = int(round(distance_to_obstacle)) # Convert the distance to obstacle into an integer if it is not 'inf'
            self.add_obstacle(distance_to_obstacle, sensor="right") # Add an ostacle to the map with the right distance

    # Callback function that is executed everytime the left lidar sensor meassures a distance
    def lidar_left_callback(self, msg):
        distance_to_obstacle = msg.ranges[0] # Take the first meassurement as the lidar sensor behaves the same way as a proximity sensor that only gives back one distance value for one meassurement
    
        if distance_to_obstacle != float('inf'):  # Check for infinite values, when the value is not infinite there is an bstacle
            distance_to_obstacle = int(round(distance_to_obstacle)) # Convert the distance to obstacle into an integer if it is not 'inf'
            self.add_obstacle(distance_to_obstacle, sensor="left") # Add an ostacle to the map with the right distance

    # Callback function that is executed everytime the back lidar sensor meassures a distance
    def lidar_back_callback(self, msg):
        distance_to_obstacle = msg.ranges[0] # Take the first meassurement as the lidar sensor behaves the same way as a proximity sensor that only gives back one distance value for one meassurement
    
        if distance_to_obstacle != float('inf'):  # Check for infinite values, when the value is not infinite there is an bstacle
            distance_to_obstacle = int(round(distance_to_obstacle + 0.1)) # Convert the distance to obstacle into an integer if it is not 'inf'
            self.add_obstacle(distance_to_obstacle, sensor="back") # Add an ostacle to the map with the right distance

    # Function for the main program
    def main_program(self):
        temp_map_distance = np.zeros((9, 9), dtype=int) # Initialize a new temporary distance map to recalculate the distance of each cell in every step
        # Copy the obstacle array into the temporary distance array
        temp_map_distance[self.map_obstacles == -1] = -1

        # Perform the FloodFill algorithm
        self.flood_fill(self.target_x, self.target_y, temp_map_distance)
        
        # Refresh the main distance map
        self.map_distance = temp_map_distance
        self.map_distance[self.target_x][self.target_y] = 0 # Set the target to 0 in the main distance map

        # Print all the maps ecxept the temporary map
        print(f"Distance map: \n {self.map_distance}")
        print(f"Movement map: \n {self.map_movement}")
        print(f"Obstacle map: \n {self.map_obstacles}")
        
        # Check if the drone is already at the target
        if self.robot_x == self.target_x and self.robot_y == self.target_y:
            print("Der Roboter ist am Ziel angekommen.")
            rclpy.shutdown() # Shutdown the ROS 2 client library
            vehicle.close() # Close the vehicle object of dronekit
            

        # Move the drone in the map and in real life/simulator
        self.move_robot()
        time.sleep(2)

    # Function to add an obstacle in front of the drone at a certain distance in meters from the drone based on whcih sensor meassures a distance
    def add_obstacle(self, distance, sensor):
        # If the front sensor meassures a distance, there is an obstacle in the x-axis in front of the drone
        if sensor == "front":
            obstacle_x = self.robot_x + distance 
            obstacle_y = self.robot_y
        # If the right sensor meassures a distance, there is an obstacle in the y-axis
        elif sensor == "right":
            obstacle_x = self.robot_x
            obstacle_y = self.robot_y - distance
        # If the left sensor meassures a distance, there is an obstacle in the y-axis
        elif sensor == "left":
            obstacle_x = self.robot_x
            obstacle_y = self.robot_y + distance
        # If the back sensor meassures a distance, there is an obstacle in the x-axis in the back of the drone
        elif sensor == "back":
            obstacle_x = self.robot_x - distance
            obstacle_y = self.robot_y

        # Checking if the obstacle is in the map or not
        if 0 <= obstacle_x < 9 and 0 <= obstacle_y < 9:
            self.map_obstacles[obstacle_x][obstacle_y] = -1 # Setting the obstacle to -1 in the obstacle map

    # Function to perform the stack based FloodFill algorithm in a given array with a given target
    def flood_fill(self, target_x, target_y, array_to_fill):
        n, m = array_to_fill.shape # Get the dimensions of the 2D array in which the FloodFill should be performed in

        # Define arrays to specify 4-way (non-diagonal) movement directions
        dx = [0, 0, 1, -1] # Movement directions of the x-axis
        dy = [1, -1, 0, 0] # Movement directions of the y-axis
        
        # Initialize stack with the target coordinates and distance 0
        stack = [(target_x, target_y, 0)]

        # Continue as long there are elements in the stack
        while stack:
            x, y, dist = stack.pop() # Remove the next cell and its distance from the stack

            # Check if the cell should be updated
            if array_to_fill[x][y] == 0 or array_to_fill[x][y] > dist:
                array_to_fill[x][y] = dist

                # Loop to explore in all 4 directions
                for i in range(4):
                    new_x, new_y = x + dx[i], y + dy[i] # Calculate the new coordinates based on the movement direction
                    # Check that the new cell is inside the array and that it is not an obstacle (indicated by -1)
                    if 0 <= new_x < n and 0 <= new_y < m and array_to_fill[new_x][new_y] != -1:
                        stack.append((new_x, new_y, dist + 1)) # Add the new cell to the stack with incremented distance

    # Function to move the drone
    def move_robot(self):
        n, m = self.map_distance.shape # Get the dimensions of the 2D map_distance array
        dx = [0, 0, 1, -1] # Movement directions of the x-axis
        dy = [1, -1, 0, 0] # Movement directions of the y-axis
        
        min_distance = float('inf') # Initialize the minimum distance
        min_positions = [] # Initialize a list to store positions with the minimum distance value

        
        # Loop to find the neighbor cell with the minimum distance
        for i in range(4):
            new_x, new_y = self.robot_x + dx[i], self.robot_y + dy[i] # Calculate new potential positions based on the movement direction
            if 0 <= new_x < n and 0 <= new_y < m and self.map_distance[new_x][new_y] >= 0: # Check if new position is within the map and that it is not an obstacle
                if self.map_distance[new_x][new_y] < min_distance: # Check if a new minimum distance is found
                    min_distance = self.map_distance[new_x][new_y] # Update the minimum distance
                    min_positions = [(new_x, new_y)] # Update the position list with the new position
                elif self.map_distance[new_x][new_y] == min_distance: # Check if the distance is the same as the current distance
                    min_positions.append((new_x, new_y)) # Add the cell to the position list

        if min_positions: # Check if there are any cells to move to
            new_x, new_y = random.choice(min_positions) # Randomly choose one of the cells in the list min_positions


            # Print the current and the new location for debugging purposes
            print(f"self.robot_x: {self.robot_x}")
            print(f"self.robot_y: {self.robot_y}")
            print(f"new_x: {new_x}")
            print(f"new_y: {new_y}")

            # Calculate the movement in x and y directions
            movement_x = new_x - self.robot_x
            movement_y = new_y - self.robot_y

            self.robot_x, self.robot_y = new_x, new_y # Update the robots current position
            self.robot_step_count += 1 # Increment the step count to isplay how many steps the drone already moved
            self.map_movement[new_x][new_y] = self.robot_step_count # Set the new position of the drone on the movement map with the value of the drone step to indicate how many steps the drone already moved
            
        # Code to actually move the drone based on the calculated x and y movement (movement_x,  movement_y)
        if  movement_x == 0 and movement_y == -1:
            goto(-1, 0) # Move the drone 1m to the west
        elif movement_x == 0 and movement_y == 1:
            goto(1, 0) # Move the drone 1m to the east
        elif movement_x == -1 and movement_y == 0:
            goto(0, -1) # Move the drone 1m to the north
        elif movement_x == 1 and movement_y == 0:
            goto(0, 1) # Move the drone 1m to the south
        else:
            raise Exception("Invalid Location") # Raise an exception if the new location is invalid

# Main function
def main(args=None):
    rclpy.init(args=args) # Initialize the ROS 2 client library
    flood_fill_node = FloodFillNode() # Create an instance of the FloodFillNode class

    rclpy.spin(flood_fill_node)  # Spin the ROS 2 node to keep it alive and responsive to callbacks

    flood_fill_node.destroy_node() # Destroy the node to end the program
    rclpy.shutdown() # Shutdown the ROS 2 client library
    vehicle.close() # Close the vehicle object of dronekit

# Check if this script is executed as a standalone script
if __name__ == '__main__':
    main() # Call the main function
