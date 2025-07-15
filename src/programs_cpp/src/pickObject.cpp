#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <memory>
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

#include "position_tracker/srv/get_position.hpp"  
#include </home/karroyabreu/ar4/src/programs_cpp/saved_poses/ar4Poses.hpp>

using GetPosition = position_tracker::srv::GetPosition;

enum class MotionType {
  Joint,
  Cartesian,
  GripperOpen,
  GripperClose
};

struct PoseStep {
  geometry_msgs::msg::Pose pose;
  double velocity_scaling;
  double acceleration_scaling;
  bool use_cartesian;
  int wait_time_ms;
  MotionType motion_type;

  PoseStep(const geometry_msgs::msg::Pose& p,
    double vel, double acc, bool cartesian, int wait_ms, MotionType type)
    : pose(p), velocity_scaling(vel), acceleration_scaling(acc),
      use_cartesian(cartesian), wait_time_ms(wait_ms), motion_type(type) {}

  // Constructor for gripper actions (no pose required)
  PoseStep(MotionType gripper_action)
    : velocity_scaling(0), acceleration_scaling(0),
      use_cartesian(false), wait_time_ms(1000), motion_type(gripper_action) {}
};

geometry_msgs::msg::Pose create_pose_from_xyz_yaw(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x / 1000.0;
  pose.position.y = y / 1000.0;
  pose.position.z = z / 1000.0;

  double corrected_yaw = yaw - M_PI_2;
  double pitch_angle = M_PI;
  double roll_angle = 0;

  tf2::Quaternion q_yaw;
  q_yaw.setRPY(0, 0, corrected_yaw);

  tf2::Quaternion q_pitch;
  q_pitch.setRPY(0, pitch_angle, 0);

  tf2::Quaternion q_roll;
  q_roll.setRPY(roll_angle, 0, 0);

  tf2::Quaternion q_final = q_yaw * q_pitch * q_roll;
  q_final.normalize();

  pose.orientation.x = q_final.x();
  pose.orientation.y = q_final.y();
  pose.orientation.z = q_final.z();
  pose.orientation.w = q_final.w();
  return pose;
}

class ProgramsNode : public rclcpp::Node
{
public:
  ProgramsNode()
    : Node("programs_node")
  {
    client_ = this->create_client<GetPosition>("get_position");
  }

  void init_move_group()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ar_manipulator");
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);
  }

  void init_gripper_group()
  {
    gripper_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ar_gripper");
    gripper_group_->setMaxVelocityScalingFactor(1.0);
    gripper_group_->setMaxAccelerationScalingFactor(1.0);
  }

  void openGripper() {
  gripper_group_->setNamedTarget("open");
  if (gripper_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("Gripper"), "Failed to open gripper.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("Gripper"), "Gripper opened.");
  }
  }

  void closeGripper() {
    gripper_group_->setNamedTarget("closed");
    if (gripper_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("Gripper"), "Failed to close gripper.");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("Gripper"), "Gripper closed.");
    }
  }

  void run_motion_sequence() {
  // ... (same code to get object_pose from service)
  if (!client_->wait_for_service(std::chrono::seconds(3))) {
  RCLCPP_ERROR(get_logger(), "GetPosition service not available.");
  return;
  }

  auto request = std::make_shared<GetPosition::Request>();
  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to call GetPosition service.");
    return;
  }

  const auto& response = future.get();
  geometry_msgs::msg::Pose object_pose = create_pose_from_xyz_yaw(
    response->x_position, response->y_position, response->z_position, 0.0);


  // THIS IS THE CODE FOR THE SEQUENCE OF POSES =============================================
  //=========================================================================================
  std::vector<PoseStep> sequence = {
       PoseStep(MotionType::GripperOpen),
      //  PoseStep(ready_pose, 0.8, 0.3, false, 1000, MotionType::Joint),
      //  PoseStep(arm_pre_dock, 0.8, 0.3, false, 1000, MotionType::Joint),
      //  PoseStep(arm_dock, 0.8, 0.3, true, 1000, MotionType::Cartesian),
      //  PoseStep(MotionType::GripperClose),
      //  PoseStep(arm_pre_dock, 0.8, 0.3, true, 1000, MotionType::Cartesian),
      //  PoseStep(dock_ready, 0.8, 0.3, false, 1000, MotionType::Joint)


     PoseStep(ready_pose, 0.8, 0.3, false, 1000, MotionType::Joint),

     PoseStep(object_pose, 0.5, 0.3, false, 100, MotionType::Joint),

     PoseStep(MotionType::GripperClose),

     PoseStep(example_drop_pose, 0.8, 0.3, true, 100, MotionType::Cartesian),

     PoseStep(MotionType::GripperOpen)
  };
  //=========================================================================================
  //=========================================================================================

  for (size_t i = 0; i < sequence.size(); ++i) {
    const auto& step = sequence[i];

    if (step.motion_type == MotionType::GripperOpen) {
      openGripper();
      rclcpp::sleep_for(std::chrono::milliseconds(step.wait_time_ms));
      continue;
    } else if (step.motion_type == MotionType::GripperClose) {
      closeGripper();
      rclcpp::sleep_for(std::chrono::milliseconds(step.wait_time_ms));
      continue;
    }

    move_group_->setPoseTarget(step.pose);
    move_group_->setMaxVelocityScalingFactor(step.velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(step.acceleration_scaling);
    move_group_->setPlanningTime(5.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;

    if (step.motion_type == MotionType::Cartesian) {
      std::vector<geometry_msgs::msg::Pose> waypoints = { step.pose };
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double eef_step = 0.01;
      const double jump_threshold = 0.0;
      double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

      if (fraction > 0.9) {
        plan.trajectory = trajectory;
        success = true;
      }
    } else {
      success = (move_group_->plan(plan) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    }

    if (success) {
      move_group_->execute(plan);
      RCLCPP_INFO(get_logger(), "Executed step %zu", i);
    } else {
      RCLCPP_WARN(get_logger(), "Planning failed at step %zu", i);
      break;
    }

    move_group_->clearPoseTargets();
    rclcpp::sleep_for(std::chrono::milliseconds(step.wait_time_ms));
  }
}
private:
  rclcpp::Client<GetPosition>::SharedPtr client_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProgramsNode>();
  node->init_move_group();
  node->init_gripper_group();  // ✅ Initialize the gripper MoveGroup
  node->run_motion_sequence();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
