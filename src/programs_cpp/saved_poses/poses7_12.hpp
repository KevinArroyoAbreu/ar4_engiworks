#ifndef SAVED_POSES_HPP
#define SAVED_POSES_HPP

#include <geometry_msgs/msg/pose.hpp>




//-----------test---------------
geometry_msgs::msg::Pose test = []() {
    geometry_msgs::msg::Pose test;
    test.position.x = -0.007011;
    test.position.y = -0.327909;
    test.position.z = 0.634711;
    test.orientation.x = 0.000357;
    test.orientation.y = 0.707107;
    test.orientation.z = -0.707107;
    test.orientation.w = -0.000101;
    return test;
}();


//-----------pose1---------------
geometry_msgs::msg::Pose pose1 = []() {
    geometry_msgs::msg::Pose pose1;
    pose1.position.x = -0.007011;
    pose1.position.y = -0.327909;
    pose1.position.z = 0.634711;
    pose1.orientation.x = 0.000357;
    pose1.orientation.y = 0.707107;
    pose1.orientation.z = -0.707107;
    pose1.orientation.w = -0.000101;
    return pose1;
}();


//-----------pose2---------------
geometry_msgs::msg::Pose pose2 = []() {
    geometry_msgs::msg::Pose pose2;
    pose2.position.x = -0.006965;
    pose2.position.y = -0.507675;
    pose2.position.z = 0.474764;
    pose2.orientation.x = -0.000030;
    pose2.orientation.y = 0.707125;
    pose2.orientation.z = -0.707089;
    pose2.orientation.w = 0.000020;
    return pose2;
}();


//-----------pose3---------------
geometry_msgs::msg::Pose pose3 = []() {
    geometry_msgs::msg::Pose pose3;
    pose3.position.x = -0.007006;
    pose3.position.y = -0.604830;
    pose3.position.z = 0.214659;
    pose3.orientation.x = -0.000016;
    pose3.orientation.y = 0.707141;
    pose3.orientation.z = -0.707073;
    pose3.orientation.w = -0.000029;
    return pose3;
}();

#endif // SAVED_POSES_HPP
