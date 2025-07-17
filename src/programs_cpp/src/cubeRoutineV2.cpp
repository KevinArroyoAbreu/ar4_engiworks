#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <memory>
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

#include "position_tracker/srv/get_position.hpp"  
#include </home/karroyabreu/ar4/src/programs_cpp/saved_poses/ar4PosesV2.hpp>

//THIS IS A COPY OF CubeRoutine.cpp, please edit prior to running

using GetPosition = position_tracker::srv::GetPosition;
using namespace std::chrono_literals;

enum class MotionType {
  Joint,
  Cartesian,
  GripperOpen,
  GripperClose,
  LoweredFromColor
};

struct PoseStep {
 // std::vector<double> joint_pose;
  geometry_msgs::msg::Pose pose;
  double velocity_scaling;
  double acceleration_scaling;
  bool use_cartesian;
  int wait_time_ms;
  MotionType motion_type;
  std::string target_color;
  double desired_z = 250.0; 

  // Static pose
  PoseStep(const geometry_msgs::msg::Pose& p,
           double vel, double acc, bool cartesian, int wait_ms, MotionType type)
    : pose(p), velocity_scaling(vel), acceleration_scaling(acc),
      use_cartesian(cartesian), wait_time_ms(wait_ms),
      motion_type(type), target_color("") {}

  // Gripper
  PoseStep(MotionType gripper_action)
    : velocity_scaling(0), acceleration_scaling(0),
      use_cartesian(false), wait_time_ms(1000),
      motion_type(gripper_action), target_color(""){}

  // Dynamic pose (base color)
  PoseStep(const std::string& color,
           double desired_z_input,
           double vel, double acc, bool cartesian, int wait_ms, MotionType type)
    : velocity_scaling(vel), acceleration_scaling(acc),
      use_cartesian(cartesian), wait_time_ms(wait_ms),
      motion_type(type), target_color(color), 
      desired_z(desired_z_input) {}
  // Joint pose
 /* PoseStep(const std::vector<double>& joints,
           double vel, double acc, int wait_ms)
    : joint_pose(joints), velocity_scaling(vel), acceleration_scaling(acc),
      use_cartesian(false), wait_time_ms(wait_ms),
      motion_type(MotionType::Joint) {} */
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
  // ======================================================================================================================
  // ================================== INITIALIZE SEQUENCES =================================================================
  // ======================================================================================================================
  std::vector<PoseStep> pick_red;
  std::vector<PoseStep> pick_blue;
  std::vector<PoseStep> pick_green;
  std::vector<PoseStep> place_cube_red;
  std::vector<PoseStep> place_cube_blue;
  std::vector<PoseStep> place_cube_green;
  std::vector<PoseStep> home;

  std::vector<PoseStep> mount_arm_sequence;
  std::vector<PoseStep> dock_arm_sequence;
  std::vector<PoseStep> move_container_sequence;

  
  

  
  // Initialization function to be called after shared_ptr creation
  void init()
  {
    init_move_group();
    init_gripper_group();
    //auto node = std::make_shared<ProgramsNode>();

    // ======================================================================================================================
    // ================================== PROGRAM SEQUENCES =================================================================
    // ======================================================================================================================
    home = {
      PoseStep(MotionType::GripperOpen),
      PoseStep(ready_pose, 1, 0.8, false, 0, MotionType::Joint),
    };

    // Define the hover and drop poses for red cube
    geometry_msgs::msg::Pose high_pose_r = get_object_pose_by_color(client_, "red", 100.0, get_logger(), this->shared_from_this());
    geometry_msgs::msg::Pose low_pose_r = high_pose_r;
    low_pose_r.position.z = 0.08;

    pick_red = {
      PoseStep(MotionType::GripperOpen),
      PoseStep(high_pose_r, 0.8, 0.3, false, 0, MotionType::Joint),
      PoseStep(low_pose_r, 0.5, 0.5, true, 0, MotionType::Cartesian),
      PoseStep(MotionType::GripperClose)
    };

    // Define the hover and drop poses for green cube
    geometry_msgs::msg::Pose high_pose_g = get_object_pose_by_color(client_, "green", 100.0, get_logger(), this->shared_from_this());
    geometry_msgs::msg::Pose low_pose_g = high_pose_g;
    low_pose_g.position.z = 0.08;

    pick_green = {
      PoseStep(MotionType::GripperOpen),
      PoseStep(high_pose_g, 0.8, 0.3, false, 0, MotionType::Joint),
      PoseStep(low_pose_g, 0.5, 0.5, true, 0, MotionType::Cartesian),
      PoseStep(MotionType::GripperClose)
    };

    // Define the hover and drop poses for blue cube
    geometry_msgs::msg::Pose high_pose_b = get_object_pose_by_color(client_, "blue", 100.0, get_logger(), this->shared_from_this());
    geometry_msgs::msg::Pose low_pose_b = high_pose_b;
    low_pose_b.position.z = 0.08;  

    pick_blue = {
      PoseStep(MotionType::GripperOpen),
      PoseStep(high_pose_b, 0.8, 0.3, false, 0, MotionType::Joint),
      PoseStep(low_pose_b, 0.5, 0.5, false, 0, MotionType::Joint),
      PoseStep(MotionType::GripperClose)
    };

    // Sequences for placing red cube on red target
    geometry_msgs::msg::Pose high_pose_rpl = get_object_pose_by_color(client_, "red_target", 170.0, get_logger(), this->shared_from_this());
    geometry_msgs::msg::Pose low_pose_rpl = high_pose_rpl;
    low_pose_rpl.position.z = 0.100;

    place_cube_red = {
      PoseStep(pre_cube_drop, 1, 1, false, 0, MotionType::Joint),
      PoseStep(high_pose_rpl, 0.6, 1, false, 0, MotionType::Joint),
      PoseStep(low_pose_rpl, 0.6, 1, true, 0, MotionType::Cartesian),
      PoseStep(MotionType::GripperOpen)
    };

    // Sequences for placing blue cube on blue target
    geometry_msgs::msg::Pose high_pose_bpl = get_object_pose_by_color(client_, "blue_target", 170.0, get_logger(), this->shared_from_this());
    geometry_msgs::msg::Pose low_pose_bpl = high_pose_bpl;
    low_pose_bpl.position.z = 0.100;

    place_cube_blue = {
      PoseStep(pre_cube_drop, 1, 1, false, 0, MotionType::Joint),
      PoseStep(high_pose_bpl, 1, 1, false, 0, MotionType::Joint),
      PoseStep(low_pose_bpl, 0.6, 1, true, 0, MotionType::Cartesian),
      PoseStep(MotionType::GripperOpen)
    };

    // Sequences for placing green cube on green target
    geometry_msgs::msg::Pose high_pose_gpl = get_object_pose_by_color(client_, "green_target", 170.0, get_logger(), this->shared_from_this());
    geometry_msgs::msg::Pose low_pose_gpl = high_pose_gpl;
    low_pose_gpl.position.z = 0.100;

    place_cube_green = {
      PoseStep(pre_cube_drop, 1, 1, false, 0, MotionType::Joint),
      PoseStep(high_pose_gpl, 0.6, 1, false, 0, MotionType::Joint),
      PoseStep(low_pose_gpl, 0.6, 1, true, 0, MotionType::Cartesian),
      PoseStep(MotionType::GripperOpen)
    };

    // Sequences for mounting and docking the arm
    mount_arm_sequence = {
      PoseStep(MotionType::GripperOpen),
      PoseStep(ready_pose, 1, 0.8, false, 0, MotionType::Joint),
      PoseStep(arm_pre_dock_v1, 1, 0.8, false, 0, MotionType::Joint),
      PoseStep(arm_dock_v1, 0.8, 0.3, true, 0, MotionType::Cartesian),
      PoseStep(arm_pre_dock_v1, 0.8, 0.3, true, 0, MotionType::Cartesian),
      PoseStep(MotionType::GripperClose),
      PoseStep(dock_ready, 1, 0.8, false, 0, MotionType::Joint)
    };
    dock_arm_sequence = {
      PoseStep(MotionType::GripperClose),
      PoseStep(ready_pose, 1, 0.8, false, 0, MotionType::Joint), 
      PoseStep(arm_pre_dock_v1, 1, 0.8, false, 0, MotionType::Joint), 
      PoseStep(MotionType::GripperOpen),
      PoseStep(arm_dock_v1, 0.8, 0.3, true, 0, MotionType::Cartesian),
      PoseStep(arm_disengage, 1, 0.6, true, 0, MotionType::Cartesian), 
      PoseStep(ready_pose, 1, 0.8, false, 0, MotionType::Joint) 
    };
    // Sequence for moving the container
    move_container_sequence = { 
       PoseStep(align_with_container_v1, 1, 1, false, 0, MotionType::Joint),
       PoseStep(pickup_container_v1, 1, 1, false, 0, MotionType::Joint),
       PoseStep(lift_container_v1, 1, 1, false, 0, MotionType::Joint),
       PoseStep(container_deliver_hover_v1, 1, 1, false, 0, MotionType::Joint),
       PoseStep(container_deliver_dropoff_v1, 1, 1, false, 0, MotionType::Joint),
       PoseStep(container_deliver_backoff_v1, 1, 1, false, 0, MotionType::Joint)

    };
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

  geometry_msgs::msg::Pose get_object_pose_by_color(
    const std::shared_ptr<rclcpp::Client<GetPosition>>& client,
    const std::string& color,
    double desired_z,
    rclcpp::Logger logger,
    const std::shared_ptr<rclcpp::Node>& node)
{
    if (!client->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_ERROR(logger, "GetPosition service not available.");
      return geometry_msgs::msg::Pose{};
    }

    auto request = std::make_shared<GetPosition::Request>();
    request->color = color;
    request->desired_z = desired_z;

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

  void run_sequence(std::vector<PoseStep>& sequence) {
    for (size_t i = 0; i < sequence.size(); ++i) {
      auto& step = sequence[i]; 

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
      geometry_msgs::msg::Pose base_pose = get_object_pose_by_color(
        client_, step.target_color, step.desired_z, get_logger(), this->shared_from_this());

      step.pose = base_pose;
    }

      RCLCPP_INFO(get_logger(), "Target pose: x=%.3f y=%.3f z=%.3f (cartesian=%s)",
            step.pose.position.x, step.pose.position.y, step.pose.position.z,
            step.use_cartesian ? "true" : "false");

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
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProgramsNode>();

  // Call init AFTER shared_ptr creation so shared_from_this() works
  node->init();

   node->run_sequence(node->home);
   node->run_sequence(node->pick_red);
   node->run_sequence(node->place_cube_blue);
   node->run_sequence(node->home);
   node->run_sequence(node->pick_blue);
   node->run_sequence(node->place_cube_red);
   node->run_sequence(node->home);
   node->run_sequence(node->pick_green);
   node->run_sequence(node->place_cube_green); 
   node->run_sequence(node->mount_arm_sequence);
   node->run_sequence(node->move_container_sequence);
   node->run_sequence(node->dock_arm_sequence);
  

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
