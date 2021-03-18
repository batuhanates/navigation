#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

using namespace message_filters;

static void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amclPose, const nav_msgs::OdometryConstPtr &truePose, const nav_msgs::OdometryConstPtr &fusePose);

static ros::Publisher amclErrorPub;
static ros::Publisher fuseErrorPub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amcl_error");

    ros::NodeHandle nh;

    amclErrorPub = nh.advertise<std_msgs::Float64>("amcl_error", 10);
    fuseErrorPub = nh.advertise<std_msgs::Float64>("fuse_error", 10);

    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> amclPoseSub(nh, "amcl_pose", 1);
    message_filters::Subscriber<nav_msgs::Odometry> truePoseSub(nh, "ground_truth/state", 1);
    message_filters::Subscriber<nav_msgs::Odometry> fusePoseSub(nh, "odometry/filtered", 1);

    typedef sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amclPoseSub, truePoseSub, fusePoseSub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    ros::spin();

    return 0;
}

static void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amclPose, const nav_msgs::OdometryConstPtr &truePose, const nav_msgs::OdometryConstPtr &fusePose)
{
    double amcl_x = amclPose->pose.pose.position.x;
    double amcl_y = amclPose->pose.pose.position.y;
    double amcl_z = amclPose->pose.pose.orientation.z;

    double true_x = truePose->pose.pose.position.x;
    double true_y = truePose->pose.pose.position.y;
    double true_z = truePose->pose.pose.orientation.z;

    double fuse_x = fusePose->pose.pose.position.x;
    double fuse_y = fusePose->pose.pose.position.y;
    double fuse_z = fusePose->pose.pose.orientation.z;

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
