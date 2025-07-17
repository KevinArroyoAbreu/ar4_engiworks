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
    right_cube_drop.position.z = 0.113455;//0.133455
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


//-----------align_with_container---------------
geometry_msgs::msg::Pose align_with_container = []() {
    geometry_msgs::msg::Pose align_with_container;
    align_with_container.position.x = -0.001996;
    align_with_container.position.y = -0.455229;
    align_with_container.position.z = 0.149846;
    align_with_container.orientation.x = 0.692433;
    align_with_container.orientation.y = 0.720686;
    align_with_container.orientation.z = 0.023470;
    align_with_container.orientation.w = -0.024451;
    return align_with_container;
}();


//-----------pickup_container---------------
geometry_msgs::msg::Pose pickup_container = []() {
    geometry_msgs::msg::Pose pickup_container;
    pickup_container.position.x = -0.000464;
    pickup_container.position.y = -0.584076;
    pickup_container.position.z = 0.149896;
    pickup_container.orientation.x = 0.685590;
    pickup_container.orientation.y = 0.727204;
    pickup_container.orientation.z = -0.019998;
    pickup_container.orientation.w = -0.027217;
    return pickup_container;
}();


//-----------lift_container---------------
geometry_msgs::msg::Pose lift_container = []() {
    geometry_msgs::msg::Pose lift_container;
    lift_container.position.x = -0.000456;
    lift_container.position.y = -0.544152;
    lift_container.position.z = 0.190172;
    lift_container.orientation.x = 0.685635;
    lift_container.orientation.y = 0.727177;
    lift_container.orientation.z = -0.023832;
    lift_container.orientation.w = -0.023471;
    return lift_container;
}();

//-----------container_deliver_hover---------------
geometry_msgs::msg::Pose container_deliver_hover = []() {
    geometry_msgs::msg::Pose container_deliver_hover;
    container_deliver_hover.position.x = -0.135088;
    container_deliver_hover.position.y = -0.213391;
    container_deliver_hover.position.z = 0.154733;
    container_deliver_hover.orientation.x = 0.998513;
    container_deliver_hover.orientation.y = 0.024427;
    container_deliver_hover.orientation.z = -0.014004;
    container_deliver_hover.orientation.w = -0.046684;
    return container_deliver_hover;
}();


//-----------container_deliver_dropoff---------------
geometry_msgs::msg::Pose container_deliver_dropoff = []() {
    geometry_msgs::msg::Pose container_deliver_dropoff;
    container_deliver_dropoff.position.x = -0.163651;
    container_deliver_dropoff.position.y = -0.212380;
    container_deliver_dropoff.position.z = 0.129830;
    container_deliver_dropoff.orientation.x = 0.996831;
    container_deliver_dropoff.orientation.y = 0.024161;
    container_deliver_dropoff.orientation.z = 0.049428;
    container_deliver_dropoff.orientation.w = -0.057460;
    return container_deliver_dropoff;
}();


//-----------container_deliver_backoff---------------
geometry_msgs::msg::Pose container_deliver_backoff = []() {
    geometry_msgs::msg::Pose container_deliver_backoff;
    container_deliver_backoff.position.x = -0.122963;
    container_deliver_backoff.position.y = -0.213187;
    container_deliver_backoff.position.z = 0.156441;
    container_deliver_backoff.orientation.x = 0.968886;
    container_deliver_backoff.orientation.y = 0.011354;
    container_deliver_backoff.orientation.z = 0.239903;
    container_deliver_backoff.orientation.w = -0.059817;
    return container_deliver_backoff;
}();


//-----------container_deliver_clear---------------
geometry_msgs::msg::Pose container_deliver_clear = []() {
    geometry_msgs::msg::Pose container_deliver_clear;
    container_deliver_clear.position.x = -0.086025;
    container_deliver_clear.position.y = -0.231904;
    container_deliver_clear.position.z = 0.210483;
    container_deliver_clear.orientation.x = 0.931948;
    container_deliver_clear.orientation.y = 0.004519;
    container_deliver_clear.orientation.z = 0.357198;
    container_deliver_clear.orientation.w = -0.062145;
    return container_deliver_clear;
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

#endif // SAVED_POSES_HPP
