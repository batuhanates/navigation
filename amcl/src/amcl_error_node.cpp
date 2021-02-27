#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

using namespace message_filters;

static void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amclPose, const nav_msgs::OdometryConstPtr &truePose);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amcl_error");

    ros::NodeHandle nh;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> amclPoseSub(nh, "amcl_pose", 1);
    message_filters::Subscriber<nav_msgs::Odometry> truePose(nh, "ground_truth/state", 1);

    typedef sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, nav_msgs::Odometry> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amclPoseSub, truePose);
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

    double error_x = true_x - amcl_x;
    double error_y = true_y - amcl_y;
    double error_z = true_z - amcl_z;

    // double error = sqrt(error_x * error_x + error_y * error_y + error_z * error_z);
    double error = sqrt(error_x * error_x + error_y * error_y);

    ROS_INFO("amcl: x: %f, y: %f, angle: %f", amcl_x, amcl_y, amcl_z);
    ROS_INFO("true: x: %f, y: %f, angle: %f", true_x, true_y, true_z);
    ROS_INFO("error: %f", error);
}
