#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// Helper: plan and execute with error checking (mirrors commander_template.cpp)
bool planAndExecute(std::shared_ptr<MoveGroupInterface> group) {
  group->setStartStateToCurrentState();

  MoveGroupInterface::Plan plan;
  bool success = (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(LOGGER, "  Plan succeeded, executing...");
    group->execute(plan);
    return true;
  } else {
    RCLCPP_ERROR(LOGGER, "  Planning failed!");
    return false;
  }
}

// Helper: move to a pose target
bool goToPose(std::shared_ptr<MoveGroupInterface> arm, double x, double y,
              double z, double roll, double pitch, double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.pose.position.x = x;
  target_pose.pose.position.y = y;
  target_pose.pose.position.z = z;
  target_pose.pose.orientation.x = q.getX();
  target_pose.pose.orientation.y = q.getY();
  target_pose.pose.orientation.z = q.getZ();
  target_pose.pose.orientation.w = q.getW();

  arm->setStartStateToCurrentState();
  arm->setPoseTarget(target_pose);
  return planAndExecute(arm);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pick_and_place", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create MoveGroupInterface for arm and gripper
  auto arm = std::make_shared<MoveGroupInterface>(node, "arm");
  auto gripper = std::make_shared<MoveGroupInterface>(node, "gripper");

  arm->setMaxVelocityScalingFactor(1.0);
  arm->setMaxAccelerationScalingFactor(1.0);
  arm->setPlanningTime(10.0);     // Give the planner more time
  arm->setNumPlanningAttempts(5); // Allow multiple attempts

  // Gripper helpers
  auto open_gripper = [&]() {
    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    gripper->setStartStateToCurrentState();
    gripper->setNamedTarget("gripper_open");
    planAndExecute(gripper);
  };

  auto close_gripper = [&]() {
    RCLCPP_INFO(LOGGER, "Closing Gripper...");
    gripper->setStartStateToCurrentState();
    gripper->setNamedTarget("gripper_closed");
    planAndExecute(gripper);
  };

  // ---- Pick and Place Sequence ----

  // 1. Move to Home
  RCLCPP_INFO(LOGGER, "=== Step 1: Moving to Home ===");
  arm->setStartStateToCurrentState();
  arm->setNamedTarget("home");
  planAndExecute(arm);
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 2. Open Gripper
  RCLCPP_INFO(LOGGER, "=== Step 2: Opening Gripper ===");
  open_gripper();
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 3. Approach Pick Position (above object)
  RCLCPP_INFO(LOGGER, "=== Step 3: Approaching Pick Position ===");
  goToPose(arm, 0.5, 0.0, 0.4, 3.14159, 0.0, 0.0);
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 4. Move Down to Pick
  RCLCPP_INFO(LOGGER, "=== Step 4: Moving Down to Pick ===");
  goToPose(arm, 0.5, 0.0, 0.2, 3.14159, 0.0, 0.0);
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 5. Close Gripper (grasp)
  RCLCPP_INFO(LOGGER, "=== Step 5: Closing Gripper ===");
  close_gripper();
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 6. Lift Object
  RCLCPP_INFO(LOGGER, "=== Step 6: Lifting Object ===");
  goToPose(arm, 0.5, 0.0, 0.4, 3.14159, 0.0, 0.0);
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 7. Move to Place Position
  RCLCPP_INFO(LOGGER, "=== Step 7: Moving to Place Position ===");
  goToPose(arm, 0.5, 0.5, 0.4, 3.14159, 0.0, 0.0);
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 8. Lower to Place
  RCLCPP_INFO(LOGGER, "=== Step 8: Lowering to Place ===");
  goToPose(arm, 0.5, 0.5, 0.2, 3.14159, 0.0, 0.0);
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 9. Open Gripper (release)
  RCLCPP_INFO(LOGGER, "=== Step 9: Releasing Object ===");
  open_gripper();
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 10. Retract
  RCLCPP_INFO(LOGGER, "=== Step 10: Retracting ===");
  goToPose(arm, 0.5, 0.5, 0.4, 3.14159, 0.0, 0.0);
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 11. Return Home
  RCLCPP_INFO(LOGGER, "=== Step 11: Returning Home ===");
  arm->setStartStateToCurrentState();
  arm->setNamedTarget("home");
  planAndExecute(arm);

  RCLCPP_INFO(LOGGER, "=== Pick and Place Completed! ===");

  rclcpp::shutdown();
  return 0;
}
