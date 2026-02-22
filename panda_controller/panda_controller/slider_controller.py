#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# This node subscribes to "joint_commands" topic and 
# publishes to "arm_controller/joint_trajectory" and "gripper_controller/joint_trajectory"
#ros2_control connects to the robot controllers and
#executes the trajectories sent by this node in gazebo & rviz
class SliderControl(Node):
    def __init__(self):
        super().__init__("slider_control")
        self.arm_pub_ = self.create_publisher(JointTrajectory, "arm_controller/joint_trajectory", 10)
        self.gripper_pub_ = self.create_publisher(JointTrajectory, "gripper_controller/joint_trajectory", 10)
        self.sub_ = self.create_subscription(JointState, "joint_commands", self.sliderCallback, 10)
        self.get_logger().info("Slider Control Node started")

    def sliderCallback(self, msg):
        arm_controller = JointTrajectory()
        gripper_controller = JointTrajectory()
        
        arm_controller.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
        gripper_controller.joint_names = ["panda_finger_joint1"]
        
        arm_goal = JointTrajectoryPoint()
        gripper_goal = JointTrajectoryPoint()
        
        arm_goal.positions = msg.position[:7]
        gripper_goal.positions = [msg.position[7]]
        
        arm_controller.points.append(arm_goal)
        gripper_controller.points.append(gripper_goal)
        
        self.arm_pub_.publish(arm_controller)
        self.gripper_pub_.publish(gripper_controller)

def main(args=None):
    rclpy.init(args=args)
    slider_control = SliderControl()
    rclpy.spin(slider_control)
    slider_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()