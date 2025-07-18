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
#include <moveit_task_constructor_msgs/msg/solution.hpp>

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
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Publisher<moveit_task_constructor_msgs::msg::Solution>::SharedPtr solution_pub_;
  rclcpp::Node::SharedPtr getNode() const {
    return node_;
  }
  std::string planning_plugin_name;
  


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
    // Always set a valid default
    planning_plugin_name = "ompl_interface/OMPLPlanner";
    

    RCLCPP_INFO(node_->get_logger(), "Using planning plugin: %s", planning_plugin_name.c_str());

    while (!node_->has_parameter("robot_description")) {
      RCLCPP_INFO(node_->get_logger(), "Waiting for robot_description parameter...");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, "ar_manipulator");
    solution_pub_ = node_->create_publisher<moveit_task_constructor_msgs::msg::Solution>(
      "/moveit_task_constructor/solution", 10);

    if (!move_group_) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to create MoveGroupInterface");
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "MoveGroupInterface created successfully");
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
    if (task.plan(5)) {
      auto solution = task.solutions().front();
      task.execute(*solution);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed");
    }
  } catch (const std::exception &ex) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Exception: " << ex.what());
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
  // Explicitly set both properties
  task.setProperty("planning_plugin", "ompl_interface/OMPLPlanner");
  task.setProperty("planning_pipeline", "ompl");



  geometry_msgs::msg::PoseStamped place_pose = place_poses[color];



  // Current state
  try {
    RCLCPP_INFO(node_->get_logger(), "Adding stage: CurrentState");
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));
    RCLCPP_INFO(node_->get_logger(), "Added stage: CurrentState");
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Error adding CurrentState: " << ex.what());
  }

  // Move to pre-grasp
  try {
    task.add(std::make_unique<mtc::stages::Connect>("move to pick",
             mtc::stages::Connect::GroupPlannerVector{
               {"ar_manipulator", std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl")} }
    ));
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Error adding Connect (move to pick): " << ex.what());
  }

  // Grasp container
  try {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

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

      auto gen = std::make_unique<mtc::stages::GeneratePose>("gen_pick");
      gen->setPose(pick);
      auto m = std::make_unique<mtc::stages::MoveTo>("move to pick",
               std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl"));
      m->setGroup("ar_manipulator");
      m->setGoal(pick);
      grasp->insert(std::move(gen));
      grasp->insert(std::move(m));
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
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Error adding SerialContainer (pick object): " << ex.what());
  }

  // Move to place
  try {
    task.add(std::make_unique<mtc::stages::Connect>("move to place",
             mtc::stages::Connect::GroupPlannerVector{
               {"ar_manipulator", std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl")}
              }
    ));
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Error adding Connect (move to place): " << ex.what());
  }

  // Place container
  try {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
    place->properties().exposeTo(place->properties(), {"eef","group","ik_frame"});
    {
      auto gen = std::make_unique<mtc::stages::GeneratePose>("gen_place");
      gen->setPose(place_pose);
      place->insert(std::move(gen));
      try {
        auto m = std::make_unique<mtc::stages::MoveTo>("move to place",
            std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl"));
        m->setGroup("ar_manipulator");
        m->setGoal(place_pose);
        place->insert(std::move(m));
      } catch (const std::exception& ex) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error initializing MoveTo stage 'move to place': " << ex.what());
      }
    }
    // Open & retreat (no detach)
    try {
      auto open = std::make_unique<mtc::stages::MoveTo>("open", std::make_shared<mtc::solvers::JointInterpolationPlanner>());
      open->setGroup("ar_gripper");
      open->setGoal("open");
      place->insert(std::move(open));
    } catch (const std::exception& ex) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Error initializing MoveTo stage 'open': " << ex.what());
    }
    try {
      auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat",
        std::make_shared<mtc::solvers::CartesianPath>());
      retreat->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = "world";
      dir.vector.x = -0.5;
      retreat->setDirection(dir);
      retreat->setMinMaxDistance(0.1, 0.3);
      place->insert(std::move(retreat));
    } catch (const std::exception& ex) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Error initializing MoveRelative stage 'retreat': " << ex.what());
    }
    task.add(std::move(place));
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Error adding SerialContainer (place object): " << ex.what());
  }

  // Return home
  try {
    auto home = std::make_unique<mtc::stages::MoveTo>("return home",
      std::make_shared<mtc::solvers::JointInterpolationPlanner>());
    home->setGroup("ar_manipulator");
    home->setGoal("home");
    task.add(std::move(home));
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Error initializing MoveTo stage 'return home': " << ex.what());
  }
  return task;
}



  
    

// Main function to run the task
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

  // Remove executor and add_node:
  // rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(mtc_task_node->getNode());

  // Run task (which calls spin_until_future_complete internally)
  mtc_task_node->doTask();

  rclcpp::shutdown();
  return 0;
}



 // mtc_task_node->setupPlanningScene();
 // mtc_task_node->doTask();


  // spin_thread->join();
  // rclcpp::shutdown();
  // return 0;




