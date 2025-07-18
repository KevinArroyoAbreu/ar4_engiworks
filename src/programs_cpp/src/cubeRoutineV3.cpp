#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <memory>
#include <chrono>
#include <cmath>

#include <fstream>
#include <sstream>

#include <tf2/LinearMath/Quaternion.h>
#include <moveit/planning_scene/planning_scene.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
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

 // void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask(double x, double y, double z, double yaw, const std::string& color); 

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> place_poses;

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
 // task_ = createTask(x, y, z, yaw, "red");


  //MTC setup
  

  auto client = node_->create_client<GetPosition>("/get_position");

  //LOOP PATTERN FOR COLOR CUBE PICKING
  //This is for the position server (using v1 first)
  std::vector<std::string> colors = { "green", "blue", "red" };

  // Define place positions for each color (PLACE POSES)
  place_poses.clear();  // Optional: in case reused

  for (const auto& color : colors) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = 0.199;
    pose.pose.position.y =
      (color == "red") ? -0.41 :
      (color == "green") ? -0.44 :
      (color == "blue") ? -0.48 : 0.0;
    pose.pose.position.z = 0.3;
    pose.pose.orientation.w = 1.0;
    place_poses[color] = pose;
  }

  for (const auto& color : colors) {
  if (!client->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(), "Service unavailable");
    continue;
  }
  auto req = std::make_shared<GetPosition::Request>();
  req->color = color;
  req->desired_z = 80.0;
  auto fut = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node_, fut) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get position for %s", color.c_str());
    continue;
  }
  auto res = fut.get();
  double x = res->x_position / 1000.0;
  double y = res->y_position / 1000.0;
  double z = res->z_position / 1000.0;
  double yaw = res->yaw;

  auto task = createTask(x,y,z,yaw,color);
  try {
    task.init();
    if (!task.plan(5)) {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed for %s", color.c_str());
      continue;
    }
    task.execute(*task.solutions().front());
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(node_->get_logger(), "Exception: %s", ex.what());
  }
}

  

  return;
}


// Object - sets some initial properties for the task
mtc::Task MTCTaskNode::createTask(double x, double y, double z, double yaw, const std::string& color) {
  mtc::Task task;
  task.setName("pick_and_place_" + color);
  task.loadRobotModel(node_);
  task.setProperty("group", "ar_manipulator");
  task.setProperty("eef", "ar_gripper");
  task.setProperty("ik_frame", "gripper_base_link");
  //task.setProperty("planning_plugin", "ompl_interface/OMPLPlanner");
 // task.setProperty("planning_pipeline", "ompl");


  geometry_msgs::msg::PoseStamped place_pose = place_poses[color];



  // Current state
  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  // Open gripper
  task.add(std::make_unique<mtc::stages::MoveTo>("open",
           std::make_shared<mtc::solvers::JointInterpolationPlanner>()));

  // Move to pre-grasp
  task.add(std::make_unique<mtc::stages::Connect>("move to pick",
           mtc::stages::Connect::GroupPlannerVector{
             {"ar_manipulator", std::make_shared<mtc::solvers::PipelinePlanner>(node_)} }
  ));

  // Grasp container
  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");

  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });



  // grasp->properties().exposeTo(grasp->properties(), {"eef","group","ik_frame"});
  // Approach
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach",
      std::make_shared<mtc::solvers::CartesianPath>());
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = "gripper_base_link";
    dir.vector.z = 1.0;
    stage->setDirection(dir);
    stage->setMinMaxDistance(0.1, 0.15);
    grasp->insert(std::move(stage));
  }
  // Set pick pose
  {
    geometry_msgs::msg::PoseStamped pick;
    pick.header.frame_id = "world";
    pick.pose.position.x = x;
    pick.pose.position.y = y;
    pick.pose.position.z = z;
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q(yaw_angle);
    pick.pose.orientation = tf2::toMsg(q);


    auto gen = std::make_unique<mtc::stages::GenerateGraspPose>("gen_pick");
    gen->setPose(pick);
    auto m = std::make_unique<mtc::stages::MoveTo>("move to pick",
             std::make_shared<mtc::solvers::PipelinePlanner>(node_));
    grasp->insert(std::move(gen));
    grasp->insert(std::move(m));
  }
  // Close
  {
    // grasp->insert(std::make_unique<mtc::stages::MoveTo>("closed",
    //   std::make_shared<mtc::solvers::JointInterpolationPlanner>()));
   // auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach");
    //attach->attachObject("object", "gripper_base_link");


    // auto close = std::make_unique<mtc::stages::MoveTo>("closed", interpolation_planner);
    // close->setGroup("ar_gripper");
    // close->setGoal("closed");
    // grasp->insert(std::move(close));

    auto close = std::make_unique<mtc::stages::MoveTo>("close hand", std::make_shared<mtc::solvers::JointInterpolationPlanner>());
    close->setGroup("ar_gripper");
    close->setGoal("closed");
    grasp->insert(std::move(close));

    // task.add(std::make_unique<mtc::stages::MoveTo>("closed",
    //        std::make_shared<mtc::solvers::JointInterpolationPlanner>()));
  }
  // Lift object
  {
    auto lift = std::make_unique<mtc::stages::MoveRelative>("lift",
      std::make_shared<mtc::solvers::CartesianPath>());
    lift->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = "world";
    dir.vector.z = 1.0;
    lift->setDirection(dir);
    lift->setMinMaxDistance(0.1, 0.3);
    grasp->insert(std::move(lift));
  }
  task.add(std::move(grasp));

  // Move to place
  task.add(std::make_unique<mtc::stages::Connect>("move to place",
           mtc::stages::Connect::GroupPlannerVector{
             {"ar_manipulator", std::make_shared<mtc::solvers::PipelinePlanner>(node_)},
             {"ar_gripper", std::make_shared<mtc::solvers::PipelinePlanner>(node_)} }
  ));

  // Place container
  auto place = std::make_unique<mtc::SerialContainer>("place object");

  task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });


 
  place->properties().exposeTo(place->properties(), {"eef","group","ik_frame"});
  {
    // Place pose
    
    auto gen = std::make_unique<mtc::stages::GeneratePlacePose>("gen_place");
    gen->setPose(place_pose);
    auto m = std::make_unique<mtc::stages::MoveTo>("move to place",
             std::make_shared<mtc::solvers::PipelinePlanner>(node_));
    place->insert(std::move(gen));
    place->insert(std::move(m));
  }
  // Open, detach & retreat
  place->insert(std::make_unique<mtc::stages::MoveTo>("open",
    std::make_shared<mtc::solvers::JointInterpolationPlanner>()));
  {
    auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach");
    detach->detachObject("object", "gripper_base_link");
    place->insert(std::move(detach));
  }
  {
    auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat",
      std::make_shared<mtc::solvers::CartesianPath>());
    retreat->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = "world";
    dir.vector.x = -0.5;
    retreat->setDirection(dir);
    retreat->setMinMaxDistance(0.1, 0.3);
    place->insert(std::move(retreat));
  }
  task.add(std::move(place));

  // Return home
  task.add(std::make_unique<mtc::stages::MoveTo>("return home",
    std::make_shared<mtc::solvers::JointInterpolationPlanner>()));
  return task;
}



  
    

// Main function to run the task
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

  // No executor, just run directly
  mtc_task_node->doTask();

  rclcpp::shutdown();
  return 0;
}


 // mtc_task_node->setupPlanningScene();
 // mtc_task_node->doTask();


  // spin_thread->join();
  // rclcpp::shutdown();
  // return 0;




    