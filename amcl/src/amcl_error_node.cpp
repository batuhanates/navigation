#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

using namespace message_filters;

static void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amclPose, const nav_msgs::OdometryConstPtr &truePose);

static ros::Publisher amclErrorPub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amcl_error");

    ros::NodeHandle nh;

    amclErrorPub = nh.advertise<std_msgs::Float64>("amcl_error", 10);

    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> amclPoseSub(nh, "amcl_pose_center", 100);
    message_filters::Subscriber<nav_msgs::Odometry> truePoseSub(nh, "robot_detector/robot_pose", 100);

    // For sync between topics
    typedef sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, nav_msgs::Odometry> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amclPoseSub, truePoseSub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}

static void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amclPose, const nav_msgs::OdometryConstPtr &truePose)
{
    double amcl_x = amclPose->pose.pose.position.x;
    double amcl_y = amclPose->pose.pose.position.y;
    double amcl_z = amclPose->pose.pose.orientation.z;

    double true_x = truePose->pose.pose.position.x;
    double true_y = truePose->pose.pose.position.y;
    double true_z = truePose->pose.pose.orientation.z;

    double errorAmcl_x = true_x - amcl_x;
    double errorAmcl_y = true_y - amcl_y;
    double errorAmcl_z = true_z - amcl_z;

    double errorAmcl = sqrt(errorAmcl_x * errorAmcl_x + errorAmcl_y * errorAmcl_y);

    std_msgs::Float64 errorAmcl_msg;
    errorAmcl_msg.data = errorAmcl;
    amclErrorPub.publish(errorAmcl_msg);

    // ROS_INFO("amcl: x: %f, y: %f, angle: %f", amcl_x, amcl_y, amcl_z);
    // ROS_INFO("true: x: %f, y: %f, angle: %f", true_x, true_y, true_z);
    // ROS_INFO("errorAmcl: %f", errorAmcl);
}
