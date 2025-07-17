#ifndef SAVED_POSES_HPP
#define SAVED_POSES_HPP

#include <geometry_msgs/msg/pose.hpp>

//Poses for target aiming v1 (discarded)


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

//-----------align_with_container_v1---------------
geometry_msgs::msg::Pose align_with_container_v1 = []() {
    geometry_msgs::msg::Pose align_with_container_v1;
    align_with_container_v1.position.x = -0.000783;
    align_with_container_v1.position.y = -0.483044;
    align_with_container_v1.position.z = 0.144427;
    align_with_container_v1.orientation.x = 0.725243;
    align_with_container_v1.orientation.y = -0.687584;
    align_with_container_v1.orientation.z = -0.033648;
    align_with_container_v1.orientation.w = 0.010912;
    return align_with_container_v1;
}();


//-----------pickup_container_v1---------------
geometry_msgs::msg::Pose pickup_container_v1 = []() {
    geometry_msgs::msg::Pose pickup_container_v1;
    pickup_container_v1.position.x = -0.008361;
    pickup_container_v1.position.y = -0.553331;
    pickup_container_v1.position.z = 0.147874;
    pickup_container_v1.orientation.x = 0.726149;
    pickup_container_v1.orientation.y = -0.686832;
    pickup_container_v1.orientation.z = -0.021021;
    pickup_container_v1.orientation.w = 0.022960;
    return pickup_container_v1;
}();


//-----------lift_container_v1---------------
geometry_msgs::msg::Pose lift_container_v1 = []() {
    geometry_msgs::msg::Pose lift_container_v1;
    lift_container_v1.position.x = -0.005846;
    lift_container_v1.position.y = -0.540276;
    lift_container_v1.position.z = 0.196811;
    lift_container_v1.orientation.x = 0.726175;
    lift_container_v1.orientation.y = -0.686803;
    lift_container_v1.orientation.z = -0.021119;
    lift_container_v1.orientation.w = 0.022896;
    return lift_container_v1;
}();


//-----------container_deliver_hover_v1---------------
geometry_msgs::msg::Pose container_deliver_hover_v1 = []() {
    geometry_msgs::msg::Pose container_deliver_hover_v1;
    container_deliver_hover_v1.position.x = -0.120715;
    container_deliver_hover_v1.position.y = -0.219226;
    container_deliver_hover_v1.position.z = 0.174047;
    container_deliver_hover_v1.orientation.x = -0.009634;
    container_deliver_hover_v1.orientation.y = 0.999517;
    container_deliver_hover_v1.orientation.z = 0.006806;
    container_deliver_hover_v1.orientation.w = -0.028769;
    return container_deliver_hover_v1;
}();


//-----------container_deliver_dropoff_v1---------------
geometry_msgs::msg::Pose container_deliver_dropoff_v1 = []() {
    geometry_msgs::msg::Pose container_deliver_dropoff_v1;
    container_deliver_dropoff_v1.position.x = -0.119746;
    container_deliver_dropoff_v1.position.y = -0.218875;
    container_deliver_dropoff_v1.position.z = 0.153292;
    container_deliver_dropoff_v1.orientation.x = -0.009901;
    container_deliver_dropoff_v1.orientation.y = 0.995385;
    container_deliver_dropoff_v1.orientation.z = 0.026504;
    container_deliver_dropoff_v1.orientation.w = 0.091702;
    return container_deliver_dropoff_v1;
}();


//-----------container_deliver_backoff_v1---------------
geometry_msgs::msg::Pose container_deliver_backoff_v1 = []() {
    geometry_msgs::msg::Pose container_deliver_backoff_v1;
    container_deliver_backoff_v1.position.x = -0.059995;
    container_deliver_backoff_v1.position.y = -0.218747;
    container_deliver_backoff_v1.position.z = 0.215678;
    container_deliver_backoff_v1.orientation.x = -0.003292;
    container_deliver_backoff_v1.orientation.y = 0.937792;
    container_deliver_backoff_v1.orientation.z = 0.029052;
    container_deliver_backoff_v1.orientation.w = 0.345964;
    return container_deliver_backoff_v1;
}();

//-----------container_deliver_backoff_v2---------------
geometry_msgs::msg::Pose container_deliver_backoff_v2 = []() {
    geometry_msgs::msg::Pose container_deliver_backoff_v2;
    container_deliver_backoff_v2.position.x = -0.060602;
    container_deliver_backoff_v2.position.y = -0.219235;
    container_deliver_backoff_v2.position.z = 0.214142;
    container_deliver_backoff_v2.orientation.x = 0.001694;
    container_deliver_backoff_v2.orientation.y = 0.882811;
    container_deliver_backoff_v2.orientation.z = 0.027847;
    container_deliver_backoff_v2.orientation.w = 0.468899;
    return container_deliver_backoff_v2;
}();


//-----------pre_cube_drop---------------
geometry_msgs::msg::Pose pre_cube_drop = []() {
    geometry_msgs::msg::Pose pre_cube_drop;
    pre_cube_drop.position.x = -0.012457;
    pre_cube_drop.position.y = -0.477385;
    pre_cube_drop.position.z = 0.260235;
    pre_cube_drop.orientation.x = 0.000129;
    pre_cube_drop.orientation.y = 0.955994;
    pre_cube_drop.orientation.z = -0.293388;
    pre_cube_drop.orientation.w = -0.000022;
    return pre_cube_drop;
}();

#endif // SAVED_POSES_HPP
