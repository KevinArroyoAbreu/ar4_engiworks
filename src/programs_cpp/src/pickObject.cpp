#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <memory>
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

#include "position_tracker/srv/get_position.hpp"  

using GetPosition = position_tracker::srv::GetPosition;

enum class MotionType { Joint, Cartesian };

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

  void run_motion_sequence()
  {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "Waiting for get_position service...");
    }

    auto request = std::make_shared<GetPosition::Request>();
    auto future = client_->async_send_request(request);

    geometry_msgs::msg::Pose object_pose;

    if (rclcpp::spin_until_future_complete(shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      RCLCPP_INFO(get_logger(), "Received: x=%.3f y=%.3f z=%.3f yaw=%.3f",
                  response->x_position, response->y_position, response->z_position, response->yaw);

      object_pose = create_pose_from_xyz_yaw(
        response->x_position, response->y_position, response->z_position, response->yaw);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to call get_position service.");
      return;
    } 

    // ➤ Open the gripper
    gripper_group_->setNamedTarget("open");
    if (gripper_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to open gripper.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Gripper opened.");

    //Pose to reach the object
    RCLCPP_INFO(get_logger(), "Adding pose to sequence: x=%.3f, y=%.3f, z=%.3f, quat=[%.3f, %.3f, %.3f, %.3f]",
                object_pose.position.x, object_pose.position.y, object_pose.position.z,
                object_pose.orientation.x, object_pose.orientation.y,
                object_pose.orientation.z, object_pose.orientation.w);

    std::vector<PoseStep> sequence = {
     PoseStep(object_pose, 0.5, 0.3, false, 1000, MotionType::Joint),
    };

    for (size_t i = 0; i < sequence.size(); ++i)
    {
      const auto& step = sequence[i];

      move_group_->setPoseTarget(step.pose);
      move_group_->setMaxVelocityScalingFactor(step.velocity_scaling);
      move_group_->setMaxAccelerationScalingFactor(step.acceleration_scaling);
      move_group_->setPlanningTime(5.0);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = false;

      if (step.motion_type == MotionType::Cartesian)
      {
        std::vector<geometry_msgs::msg::Pose> waypoints = { step.pose };
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;
        const double jump_threshold = 0.0;
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.9)
        {
          plan.trajectory = trajectory;
          success = true;
        }
      }
      else
      {
        success = (move_group_->plan(plan) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
      }

      if (success)
      {
        move_group_->execute(plan);
        RCLCPP_INFO(get_logger(), "Executed step %zu", i);
      }
      else
      {
        RCLCPP_WARN(get_logger(), "Planning failed at step %zu", i);
        break;
      }

      move_group_->clearPoseTargets();
      rclcpp::sleep_for(std::chrono::milliseconds(step.wait_time_ms));
    }

    // ➤ Close the gripper after reaching the pose
    gripper_group_->setNamedTarget("closed");
    if (gripper_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to close gripper.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Gripper closed.");
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
