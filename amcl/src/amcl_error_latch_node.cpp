#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

using namespace message_filters;

static void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amclPose);
static void fusePoseCallback(const nav_msgs::OdometryConstPtr &fusePose);
static void truePoseCallback(const nav_msgs::OdometryConstPtr &truePose);

static ros::Publisher amclErrorPub;
static ros::Publisher fuseErrorPub;

static double amcl_x;
static double amcl_y;
static double amcl_z;

static double fuse_x;
static double fuse_y;
static double fuse_z;

static double true_x;
static double true_y;
static double true_z;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amcl_error");

    ros::NodeHandle nh;

    amclErrorPub = nh.advertise<std_msgs::Float64>("amcl_error", 10);
    fuseErrorPub = nh.advertise<std_msgs::Float64>("fuse_error", 10);

    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> amclPoseSub(nh, "amcl_pose", 1);
    message_filters::Subscriber<nav_msgs::Odometry> truePoseSub(nh, "marker_pos", 1);
    message_filters::Subscriber<nav_msgs::Odometry> fusePoseSub(nh, "odometry/filtered", 1);

    amclPoseSub.registerCallback(amclPoseCallback);
    truePoseSub.registerCallback(truePoseCallback);
    fusePoseSub.registerCallback(fusePoseCallback);

    ros::spin();

    return 0;
}

static void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amclPose)
{
    amcl_x = amclPose->pose.pose.position.x;
    amcl_y = amclPose->pose.pose.position.y;
    amcl_z = amclPose->pose.pose.orientation.z;
}

static void fusePoseCallback(const nav_msgs::OdometryConstPtr &fusePose)
{
    fuse_x = fusePose->pose.pose.position.x;
    fuse_y = fusePose->pose.pose.position.y;
    fuse_z = fusePose->pose.pose.orientation.z;
}

static void truePoseCallback(const nav_msgs::OdometryConstPtr &truePose)
{
    true_x = truePose->pose.pose.position.x;
    true_y = truePose->pose.pose.position.y;
    true_z = truePose->pose.pose.orientation.z;

    double errorAmcl_x = true_x - amcl_x;
    double errorAmcl_y = true_y - amcl_y;
    double errorAmcl_z = true_z - amcl_z;

    double errorFuse_x = true_x - fuse_x;
    double errorFuse_y = true_y - fuse_y;
    double errorFuse_z = true_z - fuse_z;

    // Error including yaw
    // double errorAmcl = sqrt(errorAmcl_x * errorAmcl_x + errorAmcl_y * errorAmcl_y + errorAmcl_z * errorAmcl_z);
    // double errorFuse = sqrt(errorFuse_x * errorFuse_x + errorFuse_y * errorFuse_y + errorFuse_z * errorFuse_z);

    // Error without yaw
    double errorAmcl = sqrt(errorAmcl_x * errorAmcl_x + errorAmcl_y * errorAmcl_y);
    double errorFuse = sqrt(errorFuse_x * errorFuse_x + errorFuse_y * errorFuse_y);
    amclErrorPub.publish(errorAmcl);
    fuseErrorPub.publish(errorFuse);

    ROS_INFO("amcl: x: %f, y: %f, angle: %f", amcl_x, amcl_y, amcl_z);
    ROS_INFO("fuse: x: %f, y: %f, angle: %f", amcl_x, amcl_y, amcl_z);
    ROS_INFO("true: x: %f, y: %f, angle: %f", true_x, true_y, true_z);
    ROS_INFO("errorAmcl: %f", errorAmcl);
    ROS_INFO("errorFuse: %f", errorFuse);
}
