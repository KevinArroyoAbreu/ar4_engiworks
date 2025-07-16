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
  std::string target_color; 

  // Constructor for static pose-based steps
  PoseStep(const geometry_msgs::msg::Pose& p,
           double vel, double acc, bool cartesian, int wait_ms, MotionType type)
    : pose(p), velocity_scaling(vel), acceleration_scaling(acc),
      use_cartesian(cartesian), wait_time_ms(wait_ms),
      motion_type(type), target_color("") {}  // Initialize target_color as empty

  // Constructor for gripper actions (open/close)
  PoseStep(MotionType gripper_action)
    : velocity_scaling(0), acceleration_scaling(0),
      use_cartesian(false), wait_time_ms(1000),
      motion_type(gripper_action), target_color("") {}

  // Constructor for dynamic color-based pose
  PoseStep(const std::string& color,
           double vel, double acc, bool cartesian, int wait_ms, MotionType type)
    : velocity_scaling(vel), acceleration_scaling(acc),
      use_cartesian(cartesian), wait_time_ms(wait_ms),
      motion_type(type), target_color(color) {}
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
  std::vector<PoseStep> pick_place_sequence;
  std::vector<PoseStep> mount_arm_sequence;
  std::vector<PoseStep> move_tray_sequence;
  

  
  // Initialization function to be called after shared_ptr creation
  void init()
  {
    init_move_group();
    init_gripper_group();
    auto node = std::make_shared<ProgramsNode>();

    // ======================================================================================================================
    // ================================== PROGRAM SEQUENCES =================================================================
    // ======================================================================================================================
    pick_place_sequence = {
      PoseStep(MotionType::GripperOpen),
      PoseStep(ready_pose, 1, 0.8, false, 1000, MotionType::Joint),
      PoseStep("red", 0.8, 0.3, false, 100, MotionType::Joint),
      PoseStep(MotionType::GripperClose),
      PoseStep(example_drop_pose, 1, 0.3, true, 100, MotionType::Cartesian)
    };

    mount_arm_sequence = {
      PoseStep(MotionType::GripperOpen),
      PoseStep(ready_pose, 1, 0.8, false, 100, MotionType::Joint),
      PoseStep(arm_pre_dock, 0.1, 0.8, false, 100, MotionType::Joint),
      PoseStep(arm_dock, 0.8, 0.3, true, 100, MotionType::Cartesian),
      PoseStep(MotionType::GripperClose),
      PoseStep(arm_pre_dock, 0.8, 0.3, true, 100, MotionType::Cartesian),
      PoseStep(dock_ready, 1, 0.8, false, 100, MotionType::Joint)
    };

    move_tray_sequence = { 
      // PoseStep(ready_pose, 1, 0.8, false, 100, MotionType::Joint),
      // PoseStep(arm_pre_dock, 0.1, 0.8, false, 100, MotionType::Joint)
    };
  }

  void init_move_group()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(this, "ar_manipulator");
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);
  }

  void init_gripper_group()
  {
    gripper_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(this, "ar_gripper");
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
  
  geometry_msgs::msg::Pose get_object_pose_by_color(
    const std::shared_ptr<rclcpp::Client<GetPosition>>& client,
    const std::string& color,
    rclcpp::Logger logger,
    const std::shared_ptr<rclcpp::Node>& node)
  {
    if (!client->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_ERROR(logger, "GetPosition service not available.");
      return geometry_msgs::msg::Pose{};
    }

    auto request = std::make_shared<GetPosition::Request>();
    request->color = color;

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Failed to call GetPosition service.");
      return geometry_msgs::msg::Pose{};
    }

    const auto& response = future.get();
    RCLCPP_INFO(logger, "Received pose for color %s: [%.2f, %.2f, %.2f], yaw=%.2f",
                color.c_str(), response->x_position, response->y_position, response->z_position, response->yaw);

    return create_pose_from_xyz_yaw(
      response->x_position,
      response->y_position,
      response->z_position,
      response->yaw
    );
  }

  void run_sequence(const std::vector<PoseStep>& sequence) {
    for (size_t i = 0; i < sequence.size(); ++i) {
      auto step = sequence[i];

      if (step.motion_type == MotionType::GripperOpen) {
        openGripper();
        rclcpp::sleep_for(std::chrono::milliseconds(step.wait_time_ms));
        continue;
      } else if (step.motion_type == MotionType::GripperClose) {
        closeGripper();
        rclcpp::sleep_for(std::chrono::milliseconds(step.wait_time_ms));
        continue;
      }

      // Dynamically fetch pose if this step uses a color
      if (!step.target_color.empty()) {
        step.pose = get_object_pose_by_color(client_, step.target_color, get_logger(), this->shared_from_this());
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

  // ===================================================================================================================
  // ================================ INITIALIZE EACH POSE HERE ========================================================
  // ===================================================================================================================
  geometry_msgs::msg::Pose red_pose;
  geometry_msgs::msg::Pose green_pose;
  geometry_msgs::msg::Pose blue_pose;
  
  geometry_msgs::msg::Pose ready_pose;
  geometry_msgs::msg::Pose example_drop_pose;
  geometry_msgs::msg::Pose arm_pre_dock;
  geometry_msgs::msg::Pose arm_dock;
  geometry_msgs::msg::Pose dock_ready;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProgramsNode>();

  // Call init AFTER shared_ptr creation so shared_from_this() works
  node->init();

  node->run_sequence(node->pick_place_sequence);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
