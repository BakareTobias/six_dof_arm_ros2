#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/bool.hpp>
#include "my_robot_interfaces/msg/joint_target.hpp"
#include "my_robot_interfaces/msg/pose_target.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_robot_interfaces/msg/circle_target.hpp"


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using namespace std::placeholders;//for the _1
using JointTarget = my_robot_interfaces::msg::JointTarget;
using PoseTarget = my_robot_interfaces::msg::PoseTarget;
using String = std_msgs::msg::String;
using CircleTarget = my_robot_interfaces::msg::CircleTarget;

class Commander{//class uses composition, not inheritance
    public:
        Commander(std::shared_ptr<rclcpp::Node> node)
        {
            node_ = node;

            arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");//create MoveGroupInterface for the "arm" planning group
            gripper_ = std::make_shared<MoveGroupInterface>(node_, "claw");
            arm_->setMaxVelocityScalingFactor(1.0);
            arm_->setMaxAccelerationScalingFactor(1.0);

            open_gripper_sub_ = node_->create_subscription<Bool>("open_gripper", 10, std::bind(&Commander::OpenGripperCallback, this, _1));

            joint_target_sub_ = node_->create_subscription<JointTarget>("joint_target", 10, std::bind(&Commander::JointTargetCallback, this, _1));

            pose_target_sub_ =  node_->create_subscription<PoseTarget>("pose_target", 10, std::bind(&Commander::PoseTargetCallback, this, _1));

            named_target_sub_ = node_->create_subscription<String>("named_target", 10, std::bind(&Commander::NamedTargetCallback, this, _1));

            circle_target_sub_ = node_->create_subscription<CircleTarget>("circle_target", 10, std::bind(&Commander::CircleTargetCallback, this, _1));

        }


        void goToNamedTarget(const std::string &name)
        {
            arm_->setStartStateToCurrentState();//set the start state to the current state
            arm_->setNamedTarget(name);//set the named target
            planAndExecute(arm_);
        }

        void goToJointTarget(const std::vector<double> &joints)
        {
            arm_->setStartStateToCurrentState();
            arm_->setJointValueTarget(joints);//set the joint value target
            planAndExecute(arm_);
        }
 
        void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path=false)
        {
            //converting from roll, pitch, yaw to quaternion
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            q.normalize();

            //creating a pose target
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "base_link";
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
                    arm_->execute(plan);//execute the plan
                } 
            }
        }
         
       void circularPath(double radius, int points)
        {
            arm_->setStartStateToCurrentState();//set the start state to the current state
            std::vector<geometry_msgs::msg::Pose> waypoints;//create a list of waypoints

            arm_->startStateMonitor(2.0); // 2 seconds tolerance

            
            geometry_msgs::msg::Pose center_pose = arm_->getCurrentPose().pose;

            for (int i = 1; i <= points; i++)
            {
                geometry_msgs::msg::Pose pose = center_pose;
                
                  float angle = 2 * M_PI * i / points;
                    pose.position.x = center_pose.position.x + (radius * cos(angle));
                    pose.position.y = center_pose.position.y + (radius * sin(angle));
                    waypoints.push_back(pose);//add the waypoint to the list
            }
            moveit_msgs::msg::RobotTrajectory trajectory;//create a trajectory object to hold the trajectory
            double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);//compute the Cartesian path 0.01 represents max allowed deviation
            //returns a fraction of the path that was followed successfully

            if (fraction >= 0.9)
            {
                arm_->execute(trajectory);//execute the trajectory
            }
        }
     
        void openGripper()
        {
            gripper_->setStartStateToCurrentState();//set the start state to the current state
            gripper_->setNamedTarget("claw_open");//set the named target
            planAndExecute(gripper_);

        }

        void closeGripper()
        {
            gripper_->setStartStateToCurrentState();//set the start state to the current state
            gripper_->setNamedTarget("claw_closed");//set the named target
            planAndExecute(gripper_);
        }
    
        //for methods only used inside the class
    private:
        void planAndExecute( const std::shared_ptr<MoveGroupInterface> &interface)
        {
            MoveGroupInterface::Plan plan;//create a plan object to hold the plan
            bool success = (interface->plan(plan)== moveit::core::MoveItErrorCode::SUCCESS);//plan the motion

            if (success)
            {
                interface->execute(plan);//execute the plan
            }
        }

        //CALLBACKS

        void OpenGripperCallback(const Bool &msg)
        {
            if (msg.data)
            {
                openGripper();
            }
            else
            {
                closeGripper();
            }
        }

        void JointTargetCallback(const JointTarget &msg)
        {
            goToJointTarget(msg.joint_angles);
        }

        void PoseTargetCallback(const PoseTarget &msg)
        {
            goToPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.cartesian);
        }

        void NamedTargetCallback(const String &msg)
        {
            goToNamedTarget(msg.data);
        }

        void CircleTargetCallback(const CircleTarget &msg)
        {
            circularPath(msg.radius, msg.points);
        }

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> arm_;
        std::shared_ptr<MoveGroupInterface> gripper_;

        rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
        rclcpp::Subscription<JointTarget>::SharedPtr joint_target_sub_;
        rclcpp::Subscription<PoseTarget>::SharedPtr pose_target_sub_;
        rclcpp::Subscription<String>::SharedPtr named_target_sub_;
        rclcpp::Subscription<CircleTarget>::SharedPtr circle_target_sub_;




};





int main(int argc, char** argv)//standard program entry point
{
    rclcpp::init(argc, argv);//initialize ROS 2
    auto node = std::make_shared<rclcpp::Node>("commander_template"); // Create an empty ROS 2 node named "/commander_template"
    auto commander = Commander(node);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;

}