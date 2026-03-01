
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
//for diff message types
#include <example_interfaces/msg/bool.hpp>
#include "my_robot_interfaces/msg/joint_target.hpp"
#include "my_robot_interfaces/msg/pose_target.hpp"
#include "my_robot_interfaces/msg/color_coordinates.hpp"
#include "std_msgs/msg/string.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using JointTarget = my_robot_interfaces::msg::JointTarget;
using PoseTarget = my_robot_interfaces::msg::PoseTarget;
using Coords = my_robot_interfaces::msg::ColorCoordinates;
using String = std_msgs::msg::String;

using namespace std::placeholders;//for the _1


//this node subscribes to /color_coordinates, pulls the coordinate data, writes a 
//loop of [robot to pre_pick,robot gripper open, robot tocoordinate + some z above, slow descent, close gripper, slow ascent, pre drop, gripper open]
//for each color 


class ColorCoordinates{
    public:
        ColorCoordinates(std::shared_ptr<rclcpp::Node> node)
        {
            node_ = node;

            arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");//create MoveGroupInterface for the "arm" planning group
            gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
            arm_->setMaxVelocityScalingFactor(1.0);
            arm_->setMaxAccelerationScalingFactor(1.0);

            //the message from subscription will replace _1, this specifies whic object executes member function ColorCoordinatesCallback
            color_coordinates_sub_ = node ->create_subscription<Coords>("color_coordinates", 1, std::bind(&ColorCoordinates::ColorCoordinatesCallback, this, _1));


        }

        void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path=false)
        {
            //converting from roll, pitch, yaw to quaternion
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            q.normalize();

            //creating a pose target
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "panda_link0";
            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;
            target_pose.pose.orientation.x = q.getX();
            target_pose.pose.orientation.y = q.getY();
            target_pose.pose.orientation.z = q.getZ();
            target_pose.pose.orientation.w = q.getW();
             
            arm_->setStartStateToCurrentState();//set the start state to the current state

            if (cartesian_path)
            {
                //Cartesian path
                std::vector<geometry_msgs::msg::Pose> waypoints;//create a list of waypoints
                waypoints.push_back(target_pose.pose);

                moveit_msgs::msg::RobotTrajectory trajectory;//create a trajectory object to hold the trajectory

                double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);//compute the Cartesian path 0.01 represents max allowed deviation
                if (fraction > 0.9)
                {
                    //execute the trajectory
                    arm_->execute(trajectory);

                }
            }
            else
            {
                arm_->setPoseTarget(target_pose);

                moveit::planning_interface::MoveGroupInterface::Plan plan;//create a plan object to hold the plan
                bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

                if (success)
                {
                    auto result = arm_->execute(plan);//execute the plan
                    rclcpp::sleep_for(std::chrono::seconds(1));

                    if (result != moveit::core::MoveItErrorCode::SUCCESS)
                    {
                        RCLCPP_ERROR(node_->get_logger(), "Execution failed");
                    }
                } 
            }
        
        }
    
    private:
        void planAndExecute( const std::shared_ptr<MoveGroupInterface> &interface)
        {
            MoveGroupInterface::Plan plan;//create a plan object to hold the plan
            bool success = (interface->plan(plan)== moveit::core::MoveItErrorCode::SUCCESS);//plan the motion

            if (success)
            {
                auto result = interface->execute(plan);//execute the plan
                rclcpp::sleep_for(std::chrono::seconds(1));

                if (result != moveit::core::MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Execution failed");
                }

                                

            }
        }

        //CALLBACKS
        void ColorCoordinatesCallback(const Coords &msg)
        {
            //robot to pre-pick
            arm_->setStartStateToCurrentState();//set the start state to the current state
            arm_->setNamedTarget("pre_pick");//set the named target
            planAndExecute(arm_);
            

            //gripper open
            gripper_->setStartStateToCurrentState();//set the start state to the current state
            gripper_->setNamedTarget("gripper_open");//set the named target
            planAndExecute(gripper_);

            //define params
            double x = msg.coordinates[0];
            double y = msg.coordinates[1];
            double z = msg.coordinates[2];
            double roll = 3.14;
            double pitch = 0;
            double yaw = 0.78;

            //move to above coordinate(Z+0.2)
            goToPoseTarget(x, y, z+0.2, roll, pitch, yaw, false);
            rclcpp::sleep_for(std::chrono::seconds(2));


            //descent
            goToPoseTarget(x, y, z, roll, pitch, yaw, false);
            rclcpp::sleep_for(std::chrono::seconds(2));


            //close gripper
            gripper_->setStartStateToCurrentState();//set the start state to the current state
            gripper_->setNamedTarget("gripper_closed");//set the named target
            planAndExecute(gripper_);

            //ascent
            goToPoseTarget(x, y, z+0.3, roll, pitch, yaw, false);

            //go to pre-drop
            arm_->setStartStateToCurrentState();//set the start state to the current state
            goToPoseTarget(-0.6, 0.0, 0.2, roll, pitch, yaw, false);
            rclcpp::sleep_for(std::chrono::seconds(4));

            //open gripper
            gripper_->setStartStateToCurrentState();//set the start state to the current state
            gripper_->setNamedTarget("gripper_open");//set the named target
            planAndExecute(gripper_);


           
        }
        
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> arm_;
        std::shared_ptr<MoveGroupInterface> gripper_;

        rclcpp::Subscription<Coords>::SharedPtr color_coordinates_sub_;

};

int main(int argc, char** argv)//standard program entry point
{
    rclcpp::init(argc, argv);//initialize ROS 2
    auto node = std::make_shared<rclcpp::Node>("pick_and_place"); // Create an empty ROS 2 node named "/pick_and_place"
    auto pick_and_place = ColorCoordinates(node);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;

}