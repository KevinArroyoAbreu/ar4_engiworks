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
    arm_pre_dock.position.y = -0.164629;//-0.180085
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
    arm_dock.position.x = 0.242123;//0.242123
    arm_dock.position.y = -0.164629;//-0.174628
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


//-----------arm_pre_dock_v1---------------
geometry_msgs::msg::Pose arm_pre_dock_v1 = []() {
    geometry_msgs::msg::Pose arm_pre_dock_v1;
    arm_pre_dock_v1.position.x = 0.253597;
    arm_pre_dock_v1.position.y = -0.178480;
    arm_pre_dock_v1.position.z = 0.130905;
    arm_pre_dock_v1.orientation.x = 0.000051;
    arm_pre_dock_v1.orientation.y = 1.000000;
    arm_pre_dock_v1.orientation.z = -0.000910;
    arm_pre_dock_v1.orientation.w = -0.000332;
    return arm_pre_dock_v1;
}();


//-----------arm_dock_v1---------------
geometry_msgs::msg::Pose arm_dock_v1 = []() {
    geometry_msgs::msg::Pose arm_dock_v1;
    arm_dock_v1.position.x = 0.254002;
    arm_dock_v1.position.y = -0.175544;
    arm_dock_v1.position.z = 0.100856;
    arm_dock_v1.orientation.x = 0.000134;
    arm_dock_v1.orientation.y = 0.999999;
    arm_dock_v1.orientation.z = -0.001108;
    arm_dock_v1.orientation.w = -0.000087;
    return arm_dock_v1;
}();


//-----------arm_disengage---------------
geometry_msgs::msg::Pose arm_disengage = []() {
    geometry_msgs::msg::Pose arm_disengage;
    arm_disengage.position.x = 0.253674;
    arm_disengage.position.y = -0.236622;
    arm_disengage.position.z = 0.100709;
    arm_disengage.orientation.x = 0.001071;
    arm_disengage.orientation.y = 0.999998;
    arm_disengage.orientation.z = 0.000294;
    arm_disengage.orientation.w = -0.001648;
    return arm_disengage;
}();


//-----------center_cube_drop---------------
geometry_msgs::msg::Pose center_cube_drop = []() {
    geometry_msgs::msg::Pose center_cube_drop;
    center_cube_drop.position.x = -0.003327;
    center_cube_drop.position.y = -0.547037;
    center_cube_drop.position.z = 0.087;//0.082654
    center_cube_drop.orientation.x = 0.000024;
    center_cube_drop.orientation.y = 0.786002;
    center_cube_drop.orientation.z = -0.618224;
    center_cube_drop.orientation.w = -0.000004;
    return center_cube_drop;
}();


//-----------left_cube_drop---------------
geometry_msgs::msg::Pose left_cube_drop = []() {
    geometry_msgs::msg::Pose left_cube_drop;
    left_cube_drop.position.x = 0.026459;
    left_cube_drop.position.y = -0.598134;
    left_cube_drop.position.z = 0.125508;
    left_cube_drop.orientation.x = -0.701051;
    left_cube_drop.orientation.y = 0.706404;
    left_cube_drop.orientation.z = -0.068880;
    left_cube_drop.orientation.w = -0.069104;
    return left_cube_drop;
}();


//-----------left_cube_drop_hover---------------
geometry_msgs::msg::Pose left_cube_drop_hover = []() {
    geometry_msgs::msg::Pose left_cube_drop_hover;
    left_cube_drop_hover.position.x = 0.026545;
    left_cube_drop_hover.position.y = -0.591886;
    left_cube_drop_hover.position.z = 0.169494;
    left_cube_drop_hover.orientation.x = -0.700844;
    left_cube_drop_hover.orientation.y = 0.706512;
    left_cube_drop_hover.orientation.z = -0.069258;
    left_cube_drop_hover.orientation.w = -0.069718;
    return left_cube_drop_hover;
}();


//-----------right_cube_drop_hover---------------
geometry_msgs::msg::Pose right_cube_drop_hover = []() {
    geometry_msgs::msg::Pose right_cube_drop_hover;
    right_cube_drop_hover.position.x = -0.039740;
    right_cube_drop_hover.position.y = -0.590998;
    right_cube_drop_hover.position.z = 0.171154;
    right_cube_drop_hover.orientation.x = -0.700767;
    right_cube_drop_hover.orientation.y = 0.706573;
    right_cube_drop_hover.orientation.z = -0.069333;
    right_cube_drop_hover.orientation.w = -0.069814;
    return right_cube_drop_hover;
}();


//-----------right_cube_drop---------------
geometry_msgs::msg::Pose right_cube_drop = []() {
    geometry_msgs::msg::Pose right_cube_drop;
    right_cube_drop.position.x = - 0.1;//-0.039560
    right_cube_drop.position.y = -0.596137;
    right_cube_drop.position.z = 0.133455;
    right_cube_drop.orientation.x = -0.700876;
    right_cube_drop.orientation.y = 0.706227;
    right_cube_drop.orientation.z = -0.070483;
    right_cube_drop.orientation.w = -0.071054;
    return right_cube_drop;
}();


//-----------center_cube_drop_hover---------------
geometry_msgs::msg::Pose center_cube_drop_hover = []() {
    geometry_msgs::msg::Pose center_cube_drop_hover;
    center_cube_drop_hover.position.x = -0.006907;
    center_cube_drop_hover.position.y = -0.440;//-0.539738
    center_cube_drop_hover.position.z = 0.175907;
    center_cube_drop_hover.orientation.x = 0.000083;
    center_cube_drop_hover.orientation.y = -0.706874;
    center_cube_drop_hover.orientation.z = 0.707340;
    center_cube_drop_hover.orientation.w = -0.000004;
    return center_cube_drop_hover;
}();

#endif // SAVED_POSES_HPP
