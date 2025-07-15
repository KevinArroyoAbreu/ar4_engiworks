#ifndef SAVED_POSES_HPP
#define SAVED_POSES_HPP

#include <geometry_msgs/msg/pose.hpp>




//-----------ready_pose---------------
geometry_msgs::msg::Pose ready_pose = []() {
    geometry_msgs::msg::Pose ready_pose;
    ready_pose.position.x = -0.006843;
    ready_pose.position.y = -0.266051;
    ready_pose.position.z = 0.213080;
    ready_pose.orientation.x = 0.000223;
    ready_pose.orientation.y = 0.999578;
    ready_pose.orientation.z = -0.029037;
    ready_pose.orientation.w = 0.000099;
    return ready_pose;
}();


//-----------example_drop_pose---------------
geometry_msgs::msg::Pose example_drop_pose = []() {
    geometry_msgs::msg::Pose example_drop_pose;
    example_drop_pose.position.x = -0.196570;
    example_drop_pose.position.y = -0.256734;
    example_drop_pose.position.z = 0.104327;
    example_drop_pose.orientation.x = 0.676254;
    example_drop_pose.orientation.y = 0.736242;
    example_drop_pose.orientation.z = -0.018321;
    example_drop_pose.orientation.w = 0.017090;
    return example_drop_pose;
}();


//-----------arm_pre_dock---------------
geometry_msgs::msg::Pose arm_pre_dock = []() {
    geometry_msgs::msg::Pose arm_pre_dock;
    arm_pre_dock.position.x = 0.242123;//0.253007
    arm_pre_dock.position.y = -0.174629;//-0.180085
    arm_pre_dock.position.z = 0.120456;
    arm_pre_dock.orientation.x = 0.009927;
    arm_pre_dock.orientation.y = 0.999947;
    arm_pre_dock.orientation.z = -0.002772;
    arm_pre_dock.orientation.w = 0.000789;
    return arm_pre_dock;
}();


//-----------arm_dock---------------
geometry_msgs::msg::Pose arm_dock = []() {
    geometry_msgs::msg::Pose arm_dock;
    arm_dock.position.x = 0.242123;
    arm_dock.position.y = -0.174629;
    arm_dock.position.z = 0.088316;
    arm_dock.orientation.x = -0.001110;
    arm_dock.orientation.y = 0.999654;
    arm_dock.orientation.z = 0.026270;
    arm_dock.orientation.w = 0.000139;
    return arm_dock;
}();


//-----------dock_ready_v1---------------
geometry_msgs::msg::Pose dock_ready = []() {
    geometry_msgs::msg::Pose dock_ready;
    dock_ready.position.x = -0.006894;
    dock_ready.position.y = -0.264710;
    dock_ready.position.z = 0.166176;
    dock_ready.orientation.x = -0.698960;
    dock_ready.orientation.y = 0.714997;
    dock_ready.orientation.z = 0.010427;
    dock_ready.orientation.w = 0.011213;
    return dock_ready;
}();

#endif // SAVED_POSES_HPP
