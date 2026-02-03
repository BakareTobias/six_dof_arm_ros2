#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char** argv)//standard program entry point
{
    rclcpp::init(argc, argv);//initialize ROS 2

    //Node setup
    auto node = std::make_shared<rclcpp::Node>("test_moveit"); // Create an empty ROS 2 node named "/test_moveit"
    rclcpp::executors::SingleThreadedExecutor executor;//this executor will handle the node's callbacks, subscriptions, and services
    executor.add_node(node);

    auto spinner = std::thread([&executor]() { executor.spin(); });//start spinning in a separate thread

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");//create MoveGroupInterface for the "arm" planning group
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    /* //Named goal
    arm.setStartStateToCurrentState();//set the start state to the current state
    arm.setNamedTarget("pose_1");//set the named target "pose_1"

    moveit::planning_interface::MoveGroupInterface::Plan plan1;//create a plan object to hold the plan
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1)
    {
        arm.execute(plan1);//execute the plan
    }

    //Return to home
    arm.setStartStateToCurrentState();//set the start state to the current state
    arm.setNamedTarget("home");//set the named target "home"

    moveit::planning_interface::MoveGroupInterface::Plan plan2;//create a plan object to hold the plan
    bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success2)
    {
        arm.execute(plan2);//execute the plan
    } */

    //--------------------------------------------------------------------------------------------------
    
    //Joint goal

    /* std::vector<double> joints = {1.5, 0.5, 0.0, 1.5, 0.0, -0.7};//define joint values for the goal

    arm.setStartStateToCurrentState();
    arm.setJointValueTarget(joints);//set the joint value target

    moveit::planning_interface::MoveGroupInterface::Plan plan3;//create a plan object to hold the plan
    bool success3 = (arm.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success3)
    {
        arm.execute(plan3);//execute the plan
    } 
 */
    //Pose goal

    //converting from roll, pitch, yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(3.14,0.0,1.57);
    q.normalize();
    
    //creating a pose target
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.4;
    target_pose.pose.position.y = 0.7;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm.setStartStateToCurrentState();//set the start state to the current state
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan4;//create a plan object to hold the plan
    bool success4 = (arm.plan(plan4) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success4)
    {
        arm.execute(plan4);//execute the plan
    } 

    //Cartesian path
    /* std::vector<geometry_msgs::msg::Pose> waypoints;//create a list of waypoints
    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;//get the current pose
    pose1.position.z += -0.2;//move down in z direction
    waypoints.push_back(pose1);//add the waypoint to the list
    geometry_msgs::msg::Pose pose2 = pose1;
    pose2.position.x += 0.4;
    waypoints.push_back(pose2);
    geometry_msgs::msg::Pose pose3 = pose2;
    pose3.position.z += 0.2;
    waypoints.push_back(pose3);
    geometry_msgs::msg::Pose pose4 = pose3;
    pose4.position.x += -0.4;
    waypoints.push_back(pose4);
    

    moveit_msgs::msg::RobotTrajectory trajectory;//create a trajectory object to hold the trajectory

    double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory);//compute the Cartesian path 0.01 represents max allowed deviation
    //returns a fraction of the path that was followed successfully

    if (fraction == 1.0)
    {
        arm.execute(trajectory);//execute the trajectory
    }
 */
     
    //Circular path
    std::vector<geometry_msgs::msg::Pose> waypoints;//create a list of waypoints
    float radius = 0.15;
    int points = 32;

    geometry_msgs::msg::Pose center_pose = arm.getCurrentPose().pose;//get the current pose
    
    for (int i = 1; i <= points; i++)
    {

        geometry_msgs::msg::Pose pose = center_pose;//start from the center pose

        float angle = 2 * M_PI * i / points;
        pose.position.x = center_pose.position.x + (radius * cos(angle));
        pose.position.y = center_pose.position.y + (radius * sin(angle));
        waypoints.push_back(pose);//add the waypoint to the list
        
    }

    


    moveit_msgs::msg::RobotTrajectory trajectory;//create a trajectory object to hold the trajectory

    double fraction = arm.computeCartesianPath(waypoints, 0.02, trajectory);//compute the Cartesian path 0.01 represents max allowed deviation
    //returns a fraction of the path that was followed successfully

    if (fraction >= 0.9)
    {
        arm.execute(trajectory);//execute the trajectory
    }
    








    rclcpp::shutdown();
    spinner.join();//wait for the spinner thread to finish
    return 0;
}