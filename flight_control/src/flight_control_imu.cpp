/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_sdk_demo/flight_control_imu.h"
#include "dji_sdk/dji_sdk.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;
ros::Publisher path_pub;
ros::Publisher pose_pub;
ros::Publisher odometry_pub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
//sensor_msgs::NavSatFix current_gps;
geometry_msgs::Point current_global_pos;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;
// sensor_msgs::NavSatFix land_gps_position;           //absolute land position
// sensor_msgs::NavSatFix home_gps_position;       //absolute  home position
geometry_msgs::Point land_global_pos;
geometry_msgs::Point home_global_pos;
nav_msgs::Path path;
nav_msgs::Odometry odometry;

Mission square_mission;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flight_control_imu_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  //ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber globalPostion      = nh.subscribe("guidance/global_position", 10, &global_position_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);

  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  
  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

  // Publish the path in rviz
  path_pub = nh.advertise<nav_msgs::Path>("trajectory",10,true);
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
  path.header.frame_id = "local";

  
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  bool gohome_result=true;                 //bool result for whether gohome success ,default is true
  bool landing_result;                 //bool result for whether landing success
  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }

  if(is_M100())
  {
    ROS_INFO("M100 taking off!");
    takeoff_result = M100monitoredTakeoff();
  }
  else
  {
    ROS_INFO("A3/N3 taking off!");
    takeoff_result = monitoredTakeoff();
  }

  if(takeoff_result)
  {
    square_mission.reset();
    //square_mission.start_gps_location = currecunt_gps;
    square_mission.start_global_position = current_global_pos;
    square_mission.start_local_position = current_local_pos;
    square_mission.setTarget(0, 0, 5, 0);
    square_mission.state = 1;
    ROS_INFO("##### Start route %d ....", square_mission.state);
  }
  while(square_mission.state!=0){
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  geometry_msgs::Vector3 localOffset;
  localOffsetFromGlobalOffset(localOffset,home_global_pos,current_global_pos);
  if(std::abs(localOffset.x)>2||std::abs(localOffset.y)>2||std::abs(localOffset.z)>1){                     //if
      square_mission.reset();
      square_mission.setTarget(localOffset.x,localOffset.y,localOffset.z,0);
      square_mission.state = 5;
      gohome_result = M100monitoredGoHome();
  }

  if(gohome_result)
  {
    ROS_INFO("M100 landing!");
    if(!(landing_result =M100monitoredLand()) )
    {
      ROS_INFO("Abnormal landing state");
    }
  }
  else
  {
    ROS_INFO("Abnormal go home state");
  }
  ros::spin();
  return 0;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/
// void
// localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
//                          sensor_msgs::NavSatFix& target,
//                          sensor_msgs::NavSatFix& origin)
// {
//   double deltaLon = target.longitude - origin.longitude;
//   double deltaLat = target.latitude - origin.latitude;

//   deltaNed.y = deltaLat * deg2rad * C_EARTH;
//   deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
//   deltaNed.z = target.altitude - origin.altitude;
// }
void localOffsetFromGlobalOffset(geometry_msgs::Vector3&  localOffset,
                          geometry_msgs::Point& target,
                          geometry_msgs::Point& origin)
{
  localOffset.x = target.x - origin.x;
  localOffset.y = target.y - origin.y;
  localOffset.z = target.z - origin.z;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void Mission::step(int inbound_limit,int outbound_limit,int break_limit)
{
  static int info_counter = 0;
  geometry_msgs::Vector3     localOffset;

  float speedFactor         = 1;
  float yawThresholdInDeg   = 1;

  float xCmd, yCmd, zCmd;

  //localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);
  localOffsetFromGlobalOffset(localOffset, current_global_pos, start_global_position);

  //double xOffsetRemaining = target_offset_x - localOffset.x;
  //double yOffsetRemaining = target_offset_y - localOffset.y;
  // double zOffsetRemaining = target_offset_z - localOffset.z;

  double xOffsetRemaining = target_offset_x - (current_global_pos.x-start_global_position.x);
  double yOffsetRemaining = target_offset_y - (current_global_pos.y-start_global_position.y);
  double zOffsetRemaining = target_offset_z - (current_global_pos.z-start_global_position.z);

  double yawDesiredRad     = deg2rad * target_yaw;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  double yawInRad          = toEulerAngle(current_atti).z;

  info_counter++;
  if(info_counter > 25)
  {
    info_counter = 0;
    ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, current_local_pos.z-start_local_position.z,yawInRad);
    ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
    ROS_INFO("*****inbount=%d, outbound=%d, break=%d, gx=%f, gy=%f, gz=%f ...",inbound_counter,outbound_counter,break_counter,current_global_pos.x,current_global_pos.y,current_global_pos.z);
  }
  if (abs(xOffsetRemaining) >= speedFactor)
    xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    xCmd = xOffsetRemaining;

  if (abs(yOffsetRemaining) >= speedFactor)
    yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    yCmd = yOffsetRemaining;

/*   if (abs(zOffsetRemaining) >= speedFactor)
   zCmd = (zOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
  zCmd = zOffsetRemaining; */
  //zCmd = start_global_position.z + target_offset_z;
  zCmd =start_local_position.z + target_offset_z;


  /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */

  if (break_counter > break_limit)
  {
    ROS_INFO("##### Route %d finished....", state);
    finished = true;
    return;
  }
  else if(break_counter > 0)
  {
    sensor_msgs::Joy controlVelYawRate;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_GROUND   |
                DJISDK::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(flag);

    ctrlBrakePub.publish(controlVelYawRate);
    break_counter++;
    return;
  }
  else //break_counter = 0, not in break stage
  {
    sensor_msgs::Joy controlPosYaw;


    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawDesiredRad);
    ctrlPosYawPub.publish(controlPosYaw);
  }

  if (std::abs(xOffsetRemaining) < 0.5 &&
      std::abs(yOffsetRemaining) < 0.5 &&
      std::abs(zOffsetRemaining) < 0.5 &&
      std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
  {
    //! 1. We are within bounds; start incrementing our in-bound counter
    inbound_counter ++;
  }
  else
  {
    if (inbound_counter != 0)
    {
      //! 2. Start incrementing an out-of-bounds counter
      outbound_counter ++;
    }
  }

  //! 3. Reset withinBoundsCounter if necessary
  if (outbound_counter > outbound_limit)
  {
    ROS_INFO("##### Route %d: out of bounds, reset....", state);
    inbound_counter  = 0;
    outbound_counter = 0;
  }

  if (inbound_counter > inbound_limit)
  {
    ROS_INFO("##### Route %d start break....", state);
    break_counter = 1;
  }

}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;

  geometry_msgs::PoseStamped tmp;

  tmp.header.frame_id = "local";
  tmp.header.stamp = ros::Time::now();
  tmp.pose.position = current_local_pos;
  tmp.pose.orientation = current_atti;
  path.poses.push_back(tmp);
  
  pose_pub.publish(tmp);
  path_pub.publish(path);
}

//void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
void global_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;

  current_global_pos.x = msg->pose.position.x;
  current_global_pos.y = msg->pose.position.y;
  current_global_pos.z = msg->pose.position.z;

  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {
    start_time = ros::Time::now();
    switch(square_mission.state)
    {
      case 0:
        break;                    // standby wait for command

      case 1:
        if(!square_mission.finished)
        {
          square_mission.step(20,10,20);
        }
        else
        {
          square_mission.reset();
          square_mission.start_global_position = current_global_pos;
          square_mission.start_local_position = current_local_pos;
          square_mission.setTarget(10, 0, 0, 0);
          square_mission.state = 2;
          ROS_INFO("##### Start route %d ....", square_mission.state);
        }
        break;

      case 2:
        if(!square_mission.finished)
        {
          square_mission.step(20,10,20);
        }
        else
        {
          square_mission.reset();
          square_mission.start_global_position = current_global_pos;
          square_mission.start_local_position = current_local_pos;
          square_mission.setTarget(0, -10, 0, 0);
          square_mission.state = 3;
          ROS_INFO("##### Start route %d ....", square_mission.state);
        }
        break;
      case 3:
        if(!square_mission.finished)
        {
          square_mission.step(20,10,20);
        }
        else
        {
          square_mission.reset();
          square_mission.start_global_position = current_global_pos;
          square_mission.start_local_position = current_local_pos;
          square_mission.setTarget(-10, 0, 0, 0);
          square_mission.state = 4;
          ROS_INFO("##### Start route %d ....", square_mission.state);
        }
        break;
      case 4:
        if(!square_mission.finished)
        {
          square_mission.step(20,10,20);
        }
        else
        {
          square_mission.reset();
          square_mission.start_global_position = current_global_pos;
          square_mission.start_local_position = current_local_pos;
          square_mission.setTarget(0,0,0,0);
          ROS_INFO("##### Mission %d Finished ....", square_mission.state);
          square_mission.state = 0;                       //return to case 0
          //ros::shutdown();                                     //get off  main spin()  loop
        }
        break;
  
 /*       case 5:
          if(!square_mission.finished){
              square_mission.step(10,10,3);
          }
          else
          {
            if(square_mission.circle_state<8)
            {
              ++(square_mission.circle_state);
              square_mission.reset();
              square_mission.start_gps_location = current_gps;
              square_mission.start_local_position = current_local_pos;
              
            }
          }*/
        case 5:
          if(!square_mission.finished)
              square_mission.step(20,10,20);
            else
            {
              square_mission.reset();
              square_mission.start_global_position = current_global_pos;
              square_mission.start_local_position = current_local_pos;
              square_mission.setTarget(0,0,0,0);
              ROS_INFO("##### Drone now in Home");
              square_mission.state = 0;
            }
            break;
    }
  }
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}


/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}


/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  land_global_pos = current_global_pos;                    //log takeoff position
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while ((ros::Time::now() - start_time < ros::Duration(10)))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_global_pos.z - land_global_pos.z < 0.5)                  //if height less than 1m then take off false
  {
    ROS_ERROR("Takeoff failed.Flight status:%d,Actual status:%d,cpos:%f,lpos:%f",
    flight_status,DJISDK::M100FlightStatus::M100_STATUS_IN_AIR,current_global_pos.z,land_global_pos.z);
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    home_global_pos = current_global_pos;          //log home position
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}
          // call drontaskcontrol to land
bool
M100monitoredLand()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(6))                   //land   
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while ((ros::Time::now() - start_time < ros::Duration(10))&&(current_global_pos.z - land_global_pos.z > 0.1))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
  
  ros::Duration(0.01).sleep();
  ros::spinOnce();

  if(current_global_pos.z - land_global_pos.z > 1.0)                         // if height > 1m then landing false
  {
    ROS_INFO("c:%f,l:%f,state:%d",current_global_pos.z,land_global_pos.z,flight_status);
    ROS_ERROR("Landing failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful Landing!");
    ros::spinOnce();
  }

  return true;
}

               //call dronetaskcontrol to go home
bool
M100monitoredGoHome()
{
  ros::Time start_time = ros::Time::now();

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while ((ros::Time::now() - start_time < ros::Duration(15))&&square_mission.state!=0)
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  if(square_mission.state!=0)
  {
    ROS_ERROR("Go home failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful go home!");
    ros::spinOnce();
  }

  return true;
}
bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}
