#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>

#include <queue>
using namespace std;

#include "wifi_estimator.h"


ros::Publisher pub_path;
ros::Publisher pub_odometry;
ros::Publisher pub_pose;
ros::Publisher pub_ap;
nav_msgs::Path path;

queue<sensor_msgs::Imu> imu_buf;

WiFiEstimator estimator;

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    //double t = imu_msg.header.stamp.toSec();

    //ROS_INFO("processing IMU data with stamp %lf", t);
    imu_buf.push(*imu_msg);
}

void send_imu(const sensor_msgs::Imu &imu_msg)
{
    double t = imu_msg.header.stamp.toSec();

    //ROS_INFO("processing IMU data with stamp %lf", t);

    double dx = imu_msg.linear_acceleration.x;
    double dy = imu_msg.linear_acceleration.y;
    double dz = imu_msg.linear_acceleration.z;
    //printf("dz=%lf\n", dz);

    double rx = imu_msg.angular_velocity.x;
    double ry = imu_msg.angular_velocity.y;
    double rz = imu_msg.angular_velocity.z;
    //printf("rz=%lf\n", rz);

    estimator.processIMU(t, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}


void wifi_callback(const sensor_msgs::PointCloudPtr &wifi_msg)
{
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header.frame_id = "world";
    point_cloud.header.stamp = ros::Time::now();
    double t = wifi_msg->header.stamp.toSec();

    ROS_INFO("WiFi data with stamp %lf", t);
    if (imu_buf.empty() || t < imu_buf.front().header.stamp.toSec())
    {
        ROS_ERROR("wait for imu data");
        return;
    }
    printf("imu buf size:%d\n", imu_buf.size());
    while (!imu_buf.empty() && t >= imu_buf.front().header.stamp.toSec())
    {
        send_imu(imu_buf.front());
        imu_buf.pop();
    }
    vector<pair<int, Vector3d>> wifi;
    for (int i = 0; i < int(wifi_msg->points.size()); i++)
    {
        int id = wifi_msg->channels[0].values[i] + 0.5;
        double x = wifi_msg->points[i].x;
        double y = wifi_msg->points[i].y;
        double z = wifi_msg->points[i].z;
        wifi.push_back(make_pair(id, Vector3d(x, y, z)));
    }
    SolutionContainer solution = estimator.processWiFi(wifi);

    ROS_INFO_STREAM("position: " << solution.p.transpose());
    ROS_INFO_STREAM("velocity: " << solution.v.transpose());
    ROS_INFO_STREAM("ap: " << solution.p_ap[0].transpose());
    ROS_INFO_STREAM("gravity: " << solution.g.transpose() << " norm: " << solution.g.norm());

    Vector3d p_ap = solution.p_ap[0];
    geometry_msgs::Point32 p;
    p.x = p_ap(0);
    p.y = p_ap(1);
    p.z = p_ap(2);
    point_cloud.points.push_back(p);

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = solution.p(0);
    odometry.pose.pose.position.y = solution.p(1);
    odometry.pose.pose.position.z = solution.p(2);
    odometry.pose.pose.orientation.x = solution.q.x();
    odometry.pose.pose.orientation.y = solution.q.y();
    odometry.pose.pose.orientation.z = solution.q.z();
    odometry.pose.pose.orientation.w = solution.q.w();
    odometry.twist.twist.linear.x = solution.v(0);
    odometry.twist.twist.linear.y = solution.v(1);
    odometry.twist.twist.linear.z = solution.v(2);
    pub_odometry.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = odometry.pose.pose;
    path.poses.push_back(pose_stamped);
    //printf("%lu %lf %lf %lf\n",
    //       path.poses.size(),
    //       path.poses.back().pose.position.x,
    //       path.poses.back().pose.position.y,
    //       path.poses.back().pose.position.z);
    pub_ap.publish(point_cloud);
    pub_path.publish(path);
    pub_pose.publish(pose_stamped);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wifi_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::Level::Debug);

    pub_path     = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_pose     = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
    pub_ap	 = n.advertise<sensor_msgs::PointCloud>("ap", 1000);

    path.header.frame_id = "world";

    //ros::Subscriber sub_imu = n.subscribe("/wifi_estimator/wifi_imu", 1000, imu_callback);
    //ros::Subscriber sub_imu = n.subscribe("/i9dof_imu/imu", 1000, imu_callback);
    //ros::Subscriber sub_wifi = n.subscribe("/wifi_estimator/wifi", 1000, wifi_callback);
    ros::Subscriber sub_imu = n.subscribe("/data_generator/imu", 1000, imu_callback);
    //ros::Subscriber sub_imu = n.subscribe("/imu_3dm_gx4/imu", 1000, imu_callback);
    ros::Subscriber sub_wifi = n.subscribe("/data_generator/wifi", 1000, wifi_callback);
    ros::spin();
    return 0;
}
