#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <memory>
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
//#include <moveit/task_constructor/exceptions.h>
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



//THIS IS A COPY OF CubeRoutine.cpp, but for target detection - subscribes to v1 get_position_server
// THE FOLLOWING ROUTINE USES CACHED PLANS FOR FASTER EXECUTION
// ---- TARGETS ARE USED ONLY FOR KNOWING WHICH CUBE TO PLACE WHERE

//https://moveit.picknik.ai/humble/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html

using GetPosition = position_tracker::srv::GetPosition;
namespace mtc = moveit::task_constructor;


class MTCTaskNode 
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask(double x, double y, double z, double yaw, const std::string& color); 

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

//Interface with the MoveIt Task Constructor

void MTCTaskNode::doTask()// DO TASK DO TASK ===================//
{
  //task_ = createTask();
  //task for pick and place
  task_ = createTask(x, y, z, yaw, "red");


  //MTC setup
  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    //RCLCPP_ERROR_STREAM(LOGGER, e);
    RCLCPP_ERROR_STREAM(node_->get_logger(), e);

    return;
  }

  if (!task_.plan(5))
  {
    //RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    RCLCPP_ERROR_STREAM(node_->get_logger(), e);

    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
   // RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    RCLCPP_ERROR_STREAM(node_->get_logger(), e);

    return;
  }

  //LOOP PATTERN FOR COLOR CUBE PICKING
  //This is for the position server (using v1 first)
  std::vector<std::string> colors = { "green", "blue", "red" };

  // Define place positions for each color (PLACE POSES)
  std::map<std::string, geometry_msgs::msg::PoseStamped> place_poses;
  for (const auto& color : colors) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = 0.199;
    pose.pose.position.y =
      (color == "red") ? -0.41 :
      (color == "green") ? -0.44 :
      (color == "blue") ? -0.48 :
    pose.pose.position.z = 0.3;
    pose.pose.orientation.w = 0.0087;
    place_poses[color] = pose;
  }

  // ROS2 service client
  auto client = node_->create_client<GetPosition>("/get_position");

  for (const auto& color : colors) {
    // Request object pose
    auto req = std::make_shared<GetPosition::Request>();
    req->color = color;
    req->desired_z = 80.0;

    if (!client->wait_for_service(std::chrono::seconds(3))) {
      //RCLCPP_ERROR(node_->get_logger(), "Service unavailable");
      RCLCPP_ERROR_STREAM(node_->get_logger(), e);

      continue;
    }

    auto future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
      //RCLCPP_WARN(node_->get_logger(), "Failed to get position for %s", color.c_str());
      RCLCPP_ERROR_STREAM(node_->get_logger(), e);

      continue;
    }

    auto res = future.get();
    double x = res->x_position / 1000.0;  // Convert mm → m
    double y = res->y_position / 1000.0;
    double z = res->z_position / 1000.0;
    double yaw = res->yaw;

    // Build task for this object
    task_ = createTask(x, y, z, yaw, place_poses[color]);

    try {
      task_.init();
      if (!task_.plan(5)) {
      //  RCLCPP_ERROR(node_->get_logger(), "Planning failed for %s", color.c_str());
        RCLCPP_ERROR_STREAM(node_->get_logger(), e);

        continue;
      }
      task_.execute(*task_.solutions().front());
    } catch (const std::exception& e) {
     // RCLCPP_ERROR(node_->get_logger(), "Exception in task execution: %s", e.what());
      RCLCPP_ERROR_STREAM(node_->get_logger(), e);

    }
  }

  

  return;
}


// Object - sets some initial properties for the task
mtc::Task MTCTaskNode::createTask(double x, double y, double z, double yaw, const std::string& color)

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
    task.add(std::move(stage_open_hand));

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
  // GENERATING GRASP POSE WITH THE SERVER
  

  //auto pose_stage = std::make_unique<mtc::stages::MoveTo>("set pick pose");

  geometry_msgs::msg::PoseStamped pick_pose;
  pick_pose.header.frame_id = "world";  // SET FRAME( CHANGE)
  pick_pose.pose.position.x = x;
  pick_pose.pose.position.y = y;
  pick_pose.pose.position.z = z;

  // tf2::Quaternion q;
  // q.setRPY(0, 0, yaw);
  // pick_pose.pose.orientation = tf2::toMsg(q);
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q(yaw_angle);

  // Convert to geometry_msgs::msg::Quaternion
  pick_pose.pose.orientation = tf2::toMsg(q);

  auto pose_stage = std::make_unique<mtc::stages::MoveTo>("set pick pose");

 // pose_stage->setPose(pick_pose);
 // pose_stage->setMonitoredStage(current_state_ptr);

  move_to->setGoal(generate_pose->getGoal());

  pipeline->insert(std::move(generate_pose));
  pipeline->insert(std::move(move_to));

  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "grasp_pose");
  stage->setPreGraspPose("open");
  stage->setObject("object");
  stage->setAngleDelta(M_PI / 12);
  stage->setMonitoredStage(current_state_ptr);  // Hook into current state

  //StampedPose before IK computing (Eigen transform)
  Eigen::Isometry3d grasp_frame_transform;
  // Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
  //                       Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
  //                       Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  
/* The above code snippet is part of a C++ program that is likely part of a robotic manipulation task
planning system. Here is a breakdown of what the code is doing: */
  grasp_frame_transform.linear() = q.matrix();
  grasp_frame_transform.translation().z() = 0.1;

  // Compute Ik stage - added to serial container
  auto wrapper =
    std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(place_pose_stage));  
 
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
  stage->setGoal("closed"); // UPDATE THIS
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
  // Sample place pose - PLACE POSES LOOP
  auto generate_pose = std::make_unique<GeneratePose>("generate place pose");
  generate_pose->setPose(place_poses[color]);

  auto move_to = std::make_unique<MoveTo>("move to pick pose");
  move_to->setGroup("ar_manipulator");  // for example, set planning group

  pipeline->insert(std::move(generate_pose));
  pipeline->insert(std::move(move_to));
  //auto place_pose_stage = std::make_unique<mtc::stages::MoveTo>("set place pose");

  //====================================================




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



    