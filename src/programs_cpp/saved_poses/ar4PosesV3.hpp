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
    arm_pre_dock_v1.position.z = 0.170905;//0.130905
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




//-----------center_cube_drop_hover---------------
geometry_msgs::msg::Pose center_cube_drop_hover = []() {
    geometry_msgs::msg::Pose center_cube_drop_hover;
    center_cube_drop_hover.position.x = 0.040175;
    center_cube_drop_hover.position.y = -0.438371;
    center_cube_drop_hover.position.z = 0.235874;
    center_cube_drop_hover.orientation.x = -0.001677;
    center_cube_drop_hover.orientation.y = 0.999935;
    center_cube_drop_hover.orientation.z = -0.007149;
    center_cube_drop_hover.orientation.w = 0.008743;
    return center_cube_drop_hover;
}();

//-----------container_clearance_pose---------------
geometry_msgs::msg::Pose container_clearance_pose = []() {
    geometry_msgs::msg::Pose container_clearance_pose;
    container_clearance_pose.position.x = 0.040175;
    container_clearance_pose.position.y = -0.438371;
    container_clearance_pose.position.z = 0.24;
    container_clearance_pose.orientation.x = -0.001677;
    container_clearance_pose.orientation.y = 0.999935;
    container_clearance_pose.orientation.z = -0.007149;
    container_clearance_pose.orientation.w = 0.008743;
    return center_cube_drop_hover;
}();


//-----------left_cube_drop_hover---------------
geometry_msgs::msg::Pose left_cube_drop_hover = []() {
    geometry_msgs::msg::Pose left_cube_drop_hover;
    left_cube_drop_hover.position.x = 0.198705;
    left_cube_drop_hover.position.y = -0.405568;
    left_cube_drop_hover.position.z = 0.168523;
    left_cube_drop_hover.orientation.x = -0.002203;
    left_cube_drop_hover.orientation.y = 0.999943;
    left_cube_drop_hover.orientation.z = -0.006336;
    left_cube_drop_hover.orientation.w = 0.008344;
    return left_cube_drop_hover;
}();


//-----------right_cube_drop_hover---------------
geometry_msgs::msg::Pose right_cube_drop_hover = []() {
    geometry_msgs::msg::Pose right_cube_drop_hover;
    right_cube_drop_hover.position.x = 0.198750;
    right_cube_drop_hover.position.y = -0.478190;
    right_cube_drop_hover.position.z = 0.169590;
    right_cube_drop_hover.orientation.x = -0.001934;
    right_cube_drop_hover.orientation.y = 0.999944;
    right_cube_drop_hover.orientation.z = -0.006328;
    right_cube_drop_hover.orientation.w = 0.008236;
    return right_cube_drop_hover;
}();


//-----------right_cube_drop---------------
geometry_msgs::msg::Pose right_cube_drop = []() {
    geometry_msgs::msg::Pose right_cube_drop;
    right_cube_drop.position.x = 0.195307;
    right_cube_drop.position.y = -0.479149;
    right_cube_drop.position.z = 0.130680;
    right_cube_drop.orientation.x = -0.001874;
    right_cube_drop.orientation.y = 0.997576;
    right_cube_drop.orientation.z = -0.007487;
    right_cube_drop.orientation.w = 0.069156;
    return right_cube_drop;
}();


//-----------center_cube_drop---------------
geometry_msgs::msg::Pose center_cube_drop = []() {
    geometry_msgs::msg::Pose center_cube_drop;
    center_cube_drop.position.x = 0.196180;
    center_cube_drop.position.y = -0.442051;
    center_cube_drop.position.z = 0.131098;
    center_cube_drop.orientation.x = -0.002798;
    center_cube_drop.orientation.y = 0.997461;
    center_cube_drop.orientation.z = -0.009725;
    center_cube_drop.orientation.w = 0.070497;
    return center_cube_drop;
}();


//-----------left_cube_drop---------------
geometry_msgs::msg::Pose left_cube_drop = []() {
    geometry_msgs::msg::Pose left_cube_drop;
    left_cube_drop.position.x = 0.195087;
    left_cube_drop.position.y = -0.402590;
    left_cube_drop.position.z = 0.129333;
    left_cube_drop.orientation.x = -0.001673;
    left_cube_drop.orientation.y = 0.997610;
    left_cube_drop.orientation.z = -0.006439;
    left_cube_drop.orientation.w = 0.068779;
    return left_cube_drop;
}();


//-----------align_with_container---------------
geometry_msgs::msg::Pose align_with_container = []() {
    geometry_msgs::msg::Pose align_with_container;
    align_with_container.position.x = 0.031225;
    align_with_container.position.y = -0.434;//-0.440805
    align_with_container.position.z = 0.145949;
    align_with_container.orientation.x = 0.998884;
    align_with_container.orientation.y = 0.027504;
    align_with_container.orientation.z = -0.038359;
    align_with_container.orientation.w = -0.001473;
    return align_with_container;
}();


//-----------pickup_container---------------
geometry_msgs::msg::Pose pickup_container = []() {
    geometry_msgs::msg::Pose pickup_container;
    pickup_container.position.x = 0.159;//0.157049
    pickup_container.position.y = -0.434;//-0.438737;
    pickup_container.position.z = 0.146676;
    pickup_container.orientation.x = 0.998697;
    pickup_container.orientation.y = 0.028464;
    pickup_container.orientation.z = -0.038196;
    pickup_container.orientation.w = 0.018321;
    return pickup_container;
}();


//-----------lift_container---------------
geometry_msgs::msg::Pose lift_container = []() {
    geometry_msgs::msg::Pose lift_container;
    lift_container.position.x = 0.094916;
    lift_container.position.y = -0.440819;
    lift_container.position.z = 0.197919;
    lift_container.orientation.x = 0.998737;
    lift_container.orientation.y = 0.028132;
    lift_container.orientation.z = -0.038132;
    lift_container.orientation.w = 0.016717;
    return lift_container;
}();


//-----------container_deliver_hover---------------
geometry_msgs::msg::Pose container_deliver_hover = []() {
    geometry_msgs::msg::Pose container_deliver_hover;
    container_deliver_hover.position.x = -0.149792;
    container_deliver_hover.position.y = -0.450516;
    container_deliver_hover.position.z = 0.157748;
    container_deliver_hover.orientation.x = -0.022676;
    container_deliver_hover.orientation.y = 0.998524;
    container_deliver_hover.orientation.z = 0.035221;
    container_deliver_hover.orientation.w = -0.034573;
    return container_deliver_hover;
}();


//-----------container_deliver_dropoff---------------
geometry_msgs::msg::Pose container_deliver_dropoff = []() {
    geometry_msgs::msg::Pose container_deliver_dropoff;
    container_deliver_dropoff.position.x = -0.205045;
    container_deliver_dropoff.position.y = -0.449349;
    container_deliver_dropoff.position.z = 0.110909;
    container_deliver_dropoff.orientation.x = -0.047424;
    container_deliver_dropoff.orientation.y = 0.998177;
    container_deliver_dropoff.orientation.z = 0.037127;
    container_deliver_dropoff.orientation.w = 0.003860;
    return container_deliver_dropoff;
}();


//-----------container_deliver_backoff---------------
geometry_msgs::msg::Pose container_deliver_backoff = []() {
    geometry_msgs::msg::Pose container_deliver_backoff;
    container_deliver_backoff.position.x = -0.098738;
    container_deliver_backoff.position.y = -0.440861;
    container_deliver_backoff.position.z = 0.132858;
    container_deliver_backoff.orientation.x = -0.037849;
    container_deliver_backoff.orientation.y = 0.991565;
    container_deliver_backoff.orientation.z = -0.000573;
    container_deliver_backoff.orientation.w = 0.123957;
    return container_deliver_backoff;
}();

#endif // SAVED_POSES_HPP