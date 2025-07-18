#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <memory>
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include "position_tracker/srv/get_position.hpp"  
#include </home/karroyabreu/ar4/src/programs_cpp/saved_poses/ar4PosesV3.hpp>



//THIS IS A COPY OF CubeRoutine.cpp, but for target detection
// THE FOLLOWING ROUTINE USES CACHED PLANS FOR FASTER EXECUTION
// ---- TARGETS ARE USED ONLY FOR KNOWING WHICH CUBE TO PLACE WHERE

using GetPosition = position_tracker::srv::GetPosition;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

//Getter function --> gets the base inferface for the executor
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

//Initialize node with specified options
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_program_node", options) }
{
}

// Method planning scene setup
void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

//Interface with the MoveIt Task Constructor

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}


// Object - sets some initial properties for the task
mtc::Task MTCTaskNode::createTask()
{
  moveit::task_constructor::Task task;
  task.stages()->setName("cube routine");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "ar_manipulator";
  const auto& hand_group_name = "ar_gripper";
  const auto& hand_frame = "gripper_base_link";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);


  //Add a stage to the node
  // -- create pointers to reuse stage info 
  // -- create current state
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));
  
  // SOLVER OPTIONS:
  // 1) PipelinePlanner (OMPL)
  // 2) CartesianPath (straight line)
  // 3) JointInterpolation (simple motions, fast)
  // --------------------------------------------

  //Properties of Cartesian Planner
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // Stage --> move the robot
  // Move to stage (propagator state)
  auto stage_open_hand =
    std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("open");
    task.add(std::move(OPEN HAND STATE));

  // Connector Stage (bridges before and after)
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  //Pointer to MoveIt Task Constructor
  //stage object ==> to save a stage
  mtc::Stage* attach_object_stage =
    nullptr;  // Forward attach_object_stage to place pose generator

  // SerialContainer --> added to a task to hold
  // -- several substages (picking-action)
  {
  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });
  
  // -- under the container: approach object
  // -- move relative to current position
  // -- Vector3Stamped to indicate direction
  {
  auto stage =
      std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
    stage->properties().set("marker_ns", "approach_object");
    stage->properties().set("link", hand_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.1, 0.15);

    // Set hand forward direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame;
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
  }
  //not in container
  // -- stage to grasp pose
  // generator stage (no linking or bridging)
  // stage properties / pose before / angle delta
  // many orientations will be made (angle delta defined the variation
  // -- between pose orientations)
  {
  // Sample grasp pose
  auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "grasp_pose");
  stage->setPreGraspPose("open");
  stage->setObject("object");
  stage->setAngleDelta(M_PI / 12);
  stage->setMonitoredStage(current_state_ptr);  // Hook into current state

  //StampedPose before IK computing (Eigen transform)
  Eigen::Isometry3d grasp_frame_transform;
  Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  grasp_frame_transform.linear() = q.matrix();
  grasp_frame_transform.translation().z() = 0.1;

  // Compute Ik stage - added to serial container
  auto wrapper =
      std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(8);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame(grasp_frame_transform, hand_frame);
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  grasp->insert(std::move(wrapper));
  }
  // Allow collisions
  {
  auto stage =
      std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
  stage->allowCollisions("object",
                        task.getRobotModel()
                            ->getJointModelGroup(hand_group_name)
                            ->getLinkModelNamesWithCollisionGeometry(),
                        true);
  grasp->insert(std::move(stage));
  
}
//Close hand gripper
{
  auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
  stage->setGroup(hand_group_name);
  stage->setGoal("close"); // UPDATE THIS
  grasp->insert(std::move(stage));
}
//Modify planning scene (attach object)
{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  stage->attachObject("object", hand_frame);
  attach_object_stage = stage.get();
  grasp->insert(std::move(stage));
}

//Lift object (move relative )
{
  auto stage =
      std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.3);
  stage->setIKFrame(hand_frame);
  stage->properties().set("marker_ns", "lift_object");

  // Set upward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.z = 1.0;
  stage->setDirection(vec);
  grasp->insert(std::move(stage));
}
  task.add(std::move(grasp));
}
// CONTAINER FINISHED --> Object GRASPED


// --------------- PLACE STAGES -----------------------------
//===========================================================
{
  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                { hand_group_name, sampling_planner } });
  stage_move_to_place->setTimeout(5.0);
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_place));
}

{//CONTAINER FOR PLACING
  auto place = std::make_unique<mtc::SerialContainer>("place object");
  task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  place->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });
{
  // Sample place pose
  auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "place_pose");
  stage->setObject("object");

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = "object";
  target_pose_msg.pose.position.y = 0.5;
  target_pose_msg.pose.orientation.w = 1.0;
  stage->setPose(target_pose_msg);
  stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

  // Compute IK
  auto wrapper =
      std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(2);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame("object");
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  place->insert(std::move(wrapper));
}
//Open hand
{
  auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage->setGroup(hand_group_name);
  stage->setGoal("open");
  place->insert(std::move(stage));
}
//Enable collisions
{
  auto stage =
      std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
  stage->allowCollisions("object",
                        task.getRobotModel()
                            ->getJointModelGroup(hand_group_name)
                            ->getLinkModelNamesWithCollisionGeometry(),
                        false);
  place->insert(std::move(stage));
}
//Detach object
{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  stage->detachObject("object", hand_frame);
  place->insert(std::move(stage));
}
//MoveRelative -- retreat from object placement
{
  auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.3);
  stage->setIKFrame(hand_frame);
  stage->properties().set("marker_ns", "retreat");

  // Set retreat direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.x = -0.5;
  stage->setDirection(vec);
  place->insert(std::move(stage));
}
  task.add(std::move(place));
}//Container end

// RETURN HOME
{
  auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setGoal("ready");
  task.add(std::move(stage));
}

return task;
}


  
    //https://moveit.picknik.ai/humble/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html

// Main function to run the task
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}



    