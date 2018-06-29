#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <queue>
#include "wifi_ekf.h"




//simulation
#if SIMULATION 
	#define G_CNT 0
#else 
//experiments
	#define G_CNT 500
#endif 


using namespace std;
queue<sensor_msgs::Imu> imu_buf;
queue<sensor_msgs::PointCloud> wifi_buf;

ros::Publisher pub_path;
ros::Publisher pub_odometry;
ros::Publisher pub_pose;
ros::Publisher pub_ap;
ros::Publisher pub_v;
ros::Publisher pub_p;
nav_msgs::Path path;


int gravity_cnt = 0;
Eigen::Vector3d sum_g(0.0, 0.0, 0.0);
bool filter_start = false;

WifiEkf wifi_ekf;

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    Eigen::Vector3d acc(imu_msg->linear_acceleration.x,
                        imu_msg->linear_acceleration.y,
                        imu_msg->linear_acceleration.z);
    if (gravity_cnt < G_CNT)
    {
        sum_g += acc;
        gravity_cnt++;
        return;
    }
    else if (gravity_cnt == G_CNT)
    {
        sum_g += acc;
        gravity_cnt++;
		//for simulation
		#if SIMULATION 
        wifi_ekf.init(imu_msg->header.stamp.toSec(), Eigen::Vector3d(0.0, 0.0, -9.8));
        #else 
		//for experiments
        wifi_ekf.init(imu_msg->header.stamp.toSec(), sum_g/gravity_cnt);
        #endif 
        filter_start = true;
        return;
    }
    imu_buf.push(*imu_msg);
}

void wifi_callback(const sensor_msgs::PointCloudConstPtr &wifi_msg)
{
    wifi_buf.push(*wifi_msg);
    
    //ROS_INFO("phase: %f, RSSI: %f", wifi_msg->points[0].x,  wifi_msg->points[0].y);
}

void process()
{
    if (!filter_start || imu_buf.empty() || wifi_buf.empty())
        return;
    if (wifi_buf.front().header.stamp.toSec() < imu_buf.front().header.stamp.toSec())
    {
        wifi_buf.pop();
        return;
    }
    if (wifi_buf.front().header.stamp.toSec() >= imu_buf.back().header.stamp.toSec())
    {
        return;
    }

    sensor_msgs::PointCloud wifi_msg = wifi_buf.front();
    wifi_buf.pop();

    ROS_DEBUG("IMU integration");
    double check_t = 0.0;
    while (imu_buf.front().header.stamp.toSec() <= wifi_msg.header.stamp.toSec())
    {
        check_t = imu_buf.front().header.stamp.toSec();
        wifi_ekf.predict(check_t,
                         Eigen::Vector3d(imu_buf.front().linear_acceleration.x,
                                         imu_buf.front().linear_acceleration.y,
                                         imu_buf.front().linear_acceleration.z),
                         Eigen::Vector3d(imu_buf.front().angular_velocity.x,
                                         imu_buf.front().angular_velocity.y,
                                         imu_buf.front().angular_velocity.z)
                        );
        imu_buf.pop();
    }
	
//    //TODO Test wifi message, more than one AP message can be received 
//    ROS_INFO_STREAM("wifi " << wifi_msg.points.size() << "  " << wifi_msg.channels.size());
//    int jj = wifi_msg.channels.size();
//    for(int ii = 0; ii < wifi_msg.points.size(); ii++)
//    {
//    	ROS_INFO_STREAM("wifi " << ii << "  " << wifi_msg.channels[jj-1].values[ii]);
//    }

	
	

	ROS_DEBUG("Time shift: %f", std::fabs(check_t - wifi_msg.header.stamp.toSec()));
	//for more than one AP	
	// Eigen::VectorXd zz(wifi_msg.points.size());//wifi measurement
	// for(int ii = 0; ii < wifi_msg.points.size(); ii++)
	// {
	// 	//ROS_INFO_STREAM("wifi " << ii << "  " << wifi_msg.channels[jj-1].values[ii]);
    // int ap_id = wifi_msg.channels[0].values[ii] + 0.5;
	// 	zz(ap_id) = wifi_msg.points[ii].x;
	// }
	// wifi_ekf.update(zz);

	//for only one AP
    wifi_ekf.update(wifi_msg.points[0].x);

    
    
    ROS_INFO_STREAM("wifi position: " << wifi_ekf.p.transpose());
    ROS_INFO_STREAM("wifi velocity: " << wifi_ekf.v.transpose());
    ROS_INFO_STREAM("ap: " << wifi_ekf.ap.transpose());
    ROS_INFO_STREAM("gravity: " << wifi_ekf.g.transpose() << " norm: " << wifi_ekf.g.norm());
    ROS_INFO_STREAM("cov: " << wifi_ekf.P.diagonal().transpose());
	geometry_msgs::Point32 p_v;
	p_v.x = wifi_ekf.v(0);
	p_v.y = wifi_ekf.v(1);
	p_v.z = wifi_ekf.v(2);
	pub_v.publish(p_v);

	geometry_msgs::Point32 p_p;
	p_p.x = wifi_ekf.p(0);
	p_p.y = wifi_ekf.p(1);
	p_p.z = wifi_ekf.p(2);
	pub_p.publish(p_p);

    nav_msgs::Odometry odometry;
	sensor_msgs::PointCloud point_cloud;
	geometry_msgs::Point32 p_ap;
	for(int i = 0; i < wifi_ekf.APs.size(); i++)
	{
		p_ap.x = wifi_ekf.APs[i](0);
		p_ap.y = wifi_ekf.APs[i](1);
		p_ap.z = wifi_ekf.APs[i](2);
		point_cloud.points.push_back(p_ap);
	}
	point_cloud.header.stamp = ros::Time::now();
	point_cloud.header.frame_id = "world";
	pub_ap.publish(point_cloud);
	
	
	point_cloud.points.clear();

    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = wifi_ekf.p(0);
    odometry.pose.pose.position.y = wifi_ekf.p(1);
    odometry.pose.pose.position.z = wifi_ekf.p(2);
    odometry.pose.pose.orientation.x = Eigen::Quaterniond(wifi_ekf.q).x();
    odometry.pose.pose.orientation.y = Eigen::Quaterniond(wifi_ekf.q).y();
    odometry.pose.pose.orientation.z = Eigen::Quaterniond(wifi_ekf.q).z();
    odometry.pose.pose.orientation.w = Eigen::Quaterniond(wifi_ekf.q).w();
    odometry.twist.twist.linear.x = wifi_ekf.v(0);
    odometry.twist.twist.linear.y = wifi_ekf.v(1);
    odometry.twist.twist.linear.z = wifi_ekf.v(2);
    pub_odometry.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = odometry.pose.pose;
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);
    pub_pose.publish(pose_stamped);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wifi_ekf");
    ros::NodeHandle n("~");
    //ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    pub_path     = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_pose     = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
	pub_ap 		 = n.advertise<sensor_msgs::PointCloud>("ap", 1000);
	pub_v 		 = n.advertise<geometry_msgs::Point32>("velocity", 1000);
	pub_p 		 = n.advertise<geometry_msgs::Point32>("position", 1000);

    path.header.frame_id = "world";
    
	#if SIMULATION
    ros::Subscriber sub_imu  = n.subscribe("/data_generator/imu", 1000, imu_callback);
    ros::Subscriber sub_wifi = n.subscribe("/data_generator/wifi", 1000, wifi_callback);
    #else 
    ros::Subscriber sub_imu  = n.subscribe("/imu_3dm_gx4/imu", 1000, imu_callback);
    ros::Subscriber sub_wifi = n.subscribe("/wifi", 1000, wifi_callback);
	#endif
	 
    ros::Rate r(1000);
    while (ros::ok())
    {
        process();
        ros::spinOnce();
        r.sleep();
    }
    //ros::spin();
}
