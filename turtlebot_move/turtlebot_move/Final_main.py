#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from pyquaternion import Quaternion as PyQuaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper  
import numpy as np

 
def pick_and_place(arm, gripper, pre_pick_pose, pick_pose, pre_put_pose1, pre_put_pose2, put_pose1, put_pose2, put_pose3):
    arm.inverse_kinematic_movement(pre_pick_pose)
    time.sleep(0.5)
    gripper.move_to_position(0.0)
    arm.inverse_kinematic_movement(pick_pose)
    gripper.move_to_position(0.8)
    print("got the cube")
    time.sleep(0.5)
    arm.inverse_kinematic_movement(pre_put_pose1)
    time.sleep(0.5)
    arm.inverse_kinematic_movement(pre_put_pose2)
    time.sleep(0.5)
    arm.inverse_kinematic_movement(put_pose1)
    time.sleep(0.5)
    arm.inverse_kinematic_movement(put_pose2)
    time.sleep(0.5)
    arm.inverse_kinematic_movement(put_pose3)
    time.sleep(0.5)
    gripper.move_to_position(0.0)
    print("finished!")

def set_pose(position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w):
    return_pose = Pose()
    return_pose.position.x = position_x
    return_pose.position.y = position_y
    return_pose.position.z = position_z
    return_pose.orientation.x = orientation_x
    return_pose.orientation.y = orientation_y
    return_pose.orientation.z = orientation_z
    return_pose.orientation.w = orientation_w
    return return_pose

class TurtleBotController(Node):
    def __init__(self, speed=0.1, distance=1.0):
        super().__init__('turtlebot_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = speed  # Set the speed (positive for moving forward)
        self.target_distance = distance  # Set the target distance to move
        self.movement_complete = False

 

    # Function to move the TurtleBot and display progress in meters
    def move(self):
        print("Starting to move forward...")

 

        twist = Twist()
        twist.linear.x = self.speed  # Set the forward speed

 

        # Calculate total duration and setup for progress display
        duration = self.target_distance / abs(self.speed)
        start_time = time.time()

 

        # Initialize distance tracking
        traveled_distance = 0.0

 

        # Publish speed command and display progress
        while not self.movement_complete and (time.time() - start_time) < duration:
            self.cmd_vel_publisher.publish(twist)
            elapsed_time = time.time() - start_time
            traveled_distance = abs(self.speed) * elapsed_time

 

            # Print progress in meters
            print(f"Moved: {traveled_distance:.2f} meters", end='\r')

            # Add a small delay to avoid overloading the command buffer
            time.sleep(0.1)

 

        # Stop the TurtleBot
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.movement_complete = True

        print(f"\nFinished! Moved a total of {traveled_distance:.2f} meters.")



def main():
    rclpy.init()
    
    node = TurtleBotController(speed=0.1, distance=3.3)
    node.move()
    gripper = Gen3LiteGripper()
    arm = Gen3LiteArm()
    arm.go_vertical()
    pre_pick_pose = set_pose(0.370, -0.092, 0.371, 0.817, 0.574, -0.036, 0.028)
    # bonus_pre_pick_pose = set_pose(0.380, -0.030, 0.371, 0.817, 0.574, -0.036, 0.028)
    #pick_pose = set_pose(0.370, -0.092, 0.130, 0.758, 0.652, 0.008, 0.017)
    pick_pose = set_pose(0.370, -0.070, 0.130, 0.758, 0.652, 0.008, 0.017)

    # bonus_pick_pose = set_pose(0.394, -0.031, 0.161, 0.751, 0.659, -0.025, 0.017)
    pre_put_pose1 = set_pose(-.293, 0.081, 0.228, 0.752, 0.658, 0.023, -0.027)
    pre_put_pose2 = set_pose(-0.174, -0.002, 0.191, 0.494, 0.867, -0.007, -0.066)
    put_pose1 = set_pose(-0.195, 0.034, -0.164, 0.494, 0.867, -0.008, -0.066)
    put_pose2 = set_pose(-0.173, 0.0, -0.342, 0.542, 0.838, -0.04, -0.042)
    put_pose3 = set_pose(-0.159, 0.085, -0.336, -0.018, 0.997, 0.026, -0.076)

    # standard arm movement takes 2min 5s
    

    pick_and_place(arm, gripper, pre_pick_pose, pick_pose, pre_put_pose1, pre_put_pose2, put_pose1, put_pose2, put_pose3)

    # Create a node instance to move the TurtleBot forward by 3.3 meters
    node2 = TurtleBotController(speed=-0.1, distance=3.2)

    # Execute the move command
    node2.move()

    # Clean up and shut down
    node.destroy_node()
    #rclpy.shutdown()
    
        # Clean up and shut down
    node2.destroy_node()
    #rclpy.shutdown()

 
    # Shutdown ROS2 and clean up
    rclpy.shutdown()
    gripper.shutdown()
    arm.shutdown()


if __name__ == '__main__':
    main()
