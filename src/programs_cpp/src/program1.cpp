#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <memory>
#include </home/karroyabreu/ar4/src/programs_cpp/saved_poses/poses7_12.hpp>
#include <chrono>

class ProgramsNode : public rclcpp::Node
{
public:
  ProgramsNode() : Node("programs_node")
  {
    // constructor empty
  }

  void init_move_group(const rclcpp::Node::SharedPtr & node_handle)
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_handle, "ar_manipulator");
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
  }


void run_motion_sequence()
{
  std::vector<PoseStep> sequence = {
    PoseStep(pose1, 0.5, 0.3, false, 1000, MotionType::Joint),
    PoseStep(pose2, 0.2, 0.2, true, 3000, MotionType::Cartesian),
    PoseStep(pose3, 1.0, 1.0, false, 2000, MotionType::Joint),
  };

  for (size_t i = 0; i < sequence.size(); ++i)
  {
    const auto& step = sequence[i];

    move_group_->setPoseTarget(step.pose);
    move_group_->setMaxVelocityScalingFactor(step.velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(step.acceleration_scaling);
    move_group_->setPlanningTime(5.0);  // optional

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;

    if (step.motion_type == MotionType::Cartesian)
    {
      std::vector<geometry_msgs::msg::Pose> waypoints = { step.pose };
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double eef_step = 0.01;  // adjust as needed
      const double jump_threshold = 0.0;
      double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

      if (fraction > 0.9)
      {
        plan.trajectory = trajectory;
        success = true;
      }
    }
    else  // Joint space motion
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
}

enum class MotionType { Joint, Cartesian };

//THE FOLLOWING ARE FOR ADDING GRIPPER ACTIONS, OUTS INS AND CAMERA
enum class ActionType
{
  Move,
  GripperOpen,
  GripperClose,
  WaitForCamera,
};
struct TaskStep
{
  ActionType action;
  geometry_msgs::msg::Pose pose;   // for Move
  double velocity_scaling;
  double acceleration_scaling;
  bool cartesian;
  int wait_time_ms;                // wait after this step
  // For gripper open/close, pose and velocity are ignored
};

// FOR POSES
struct PoseStep
{
  geometry_msgs::msg::Pose pose;
  double velocity_scaling = 1.0;
  double acceleration_scaling = 1.0;
  bool use_cartesian = false;
  int wait_time_ms = 0;
  MotionType motion_type = MotionType::Joint;

  PoseStep(const geometry_msgs::msg::Pose& p,
           double vel = 1.0,
           double acc = 1.0,
           bool cartesian = false,
           int wait_ms = 0,
           MotionType type = MotionType::Joint)
    : pose(p),
      velocity_scaling(vel),
      acceleration_scaling(acc),
      use_cartesian(cartesian),
      wait_time_ms(wait_ms),
      motion_type(type) {}
};


private:
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ProgramsNode>();

  // Pass the shared_ptr to init_move_group to avoid shared_from_this issues
  node->init_move_group(node);

  node->run_motion_sequence();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
