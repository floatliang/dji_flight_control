# dji_flight_control
GPS,Imu,AoA and WiFi based localization and flight control.

# GPS-based flight

This demo consists of two files : `demo_flight_control.h` and `demo_flight_control.cpp`.Now let's get started with its data struct.

## Data struct

- [geometry_msgs::PointStamped](docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html) consists of `std_msgs::Header` and `geometry_msgs::Point`,header is a common struct which contains `frame` and `timestamp` of messages.
- [geometry_msgs::Vector3](docs.ros.org/api/geometry_msgs/html/msg/Vector3.html) is a vector which consists of three float variables `x`,`y`,`z`.
- [sensor_msgs::Joy](docs.ros.org/api/sensor_msgs/html/msg/Joy.html).In this demo it's used to command the X, Y position **offset**, Z position (**height**) and yaw angle in ENU ground frame to the publisher `controlPosYaw`:

	sensor_msgs/Joy |description (LB2, M100) 	|description (SBUS) 	| Range (LB2) |	Range (SBUS) | Range (M100)
	-----------------------|----------------------------------|--------------------------|-----------------|-------------------|---------------------
	axes[0] | Roll Channel | Channel A |	-1 to +1 |	-1 to +1 	|-1 to +1
	axes[1] | Pitch Channel | Channel E |	-1 to +1 |	-1 to +1 	|-1 to +1
	axes[2] | Yaw Channel |	Channel R 	|-1 to +1 | -1 to +1 | -1 to +1
	axes[3] | Throttle Channel |Channel T |	-1 to +1 |	-1 to +1|	-1 to +1
	axes[4] | Mode switch |	Channel U 	|-10000, 0, 10000 |	-10000, 0, 10000 | -8000, 0, 8000
	axes[5] | Landing gear (H) switch |	Channel Gear |	-5000, -10000 |-10000, 10000 |-4545, -10000
	Known bug |For SBUS controllers, the gear output depend on the channel mapping. Please refer to DJI Assistant 2 Remote controller settings.

- [sensor_msgs::NavSatFix](docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html) is a Navigation Satellite fix for any Global Navigation Satellite System.

## Variables

- Class `Mission` is used to control a series of tasks.	* Member variable `state` indicates current mission of you drone, you can write your own mission in `gps_callback()` function.
	* Member variables `inbound_counter`,`outbound_counter` and `break_counter` indicate how long function `step()` takes to figure out drone's state.For example,state inbound means drone is reaching mission target.
	* Member variables `target_offset_x` .. indicate the target point of current mission,which is a relative distance from the previous mission's end point.
	* Member variables `start_gps_location` and `start_local_position` indicate the start postion of current mission.
- `M100FlightStatus` is a enum variable(derived from `DJI::OSDK::VehicleStatus::M100FlightStatus` in `dji_sdk.h`) indicate current status of drone.
-  Global variable `land_gps_position` and `home_gps_position` indicate the position drone take off and the position drone finished taking off.

## Functions
- Member function `step()` of Class `Mission` compare its current GPS position and target position, and publish `controlPosYaw` messages of next step to [dji_sdk/flight_control_setpoint_ENUposition_yaw](wiki.ros.org/dji_sdk) topic which subscribed by master node.
- `localOffsetFromGpsOffset()` is a simplified calculation of local NED offset between two pairs of GPS coordinates. It is accurate when distances are small.
g- `flight_status_callback()` subscribes to [dji_sdk/flight_status](wiki.ros.org/dji_sdk) topic and get `M100FlightStatus` of drone from master node.
- `gps_callback()` subscribes to [dji_sdk/gps_position](wiki.ros.org/dji_sdk) topic and get GPS information from master node.It will do a certain job(you can modify it as you wish) according to the `state` of `Mission`.
- `local_position_callback()` subscribes to [dji_sdk/local_position](wiki.ros.org/dji_sdk) topic and get local position information(actually based on GPS information) from master node.It is used to determine the altitude(a relative height from local original point which set by `set_local_position()` function) published by `controlPosYaw` publisher.
- `obtain_control()` will call [dji_sdk::SDKControlAuthority](docs.ros.org/indigo/api/dji_sdk/html/srv/SDKPermissionControl.html) service to obtain DJI API control which can be allocated by `DJI Assitant2` software.
- `M100monitoredTakeoff()` will call [dji_sdk::DroneTaskControl](docs.ros.org/indigo/api/dji_sdk/html/srv/DroneTaskControl.html) service to take off.Its process actually all accomplished in master node and has nothing to do with `demo_flight_control` node.Request and Response of service:

	|Request|
	|------------| ------------| ------------| ------------| 
	|uint8 task |	4--takeoff |	6--landing |	1--gohome|

	|Response |
	|---------------|	------------| ------------| 
	|bool result |	true--succeed |	false--failed|

- `M100monitoredLand()` is the same as `M100monitoredTakeoff()` but call drone to land.
- `M100monitoredGoHome()` will control drone to go home.It's a `state` of `Mission` in `gps_callback()`,so its process is done by `step()`.
- `set_local_position()` will set a certain GPS point as local original point,so it need sufficient GPS signal.GPS signal can be generated by `DJI Assitant2` software when simulation.

## Installation

1. Make sure you have installed `dji-sdk/Onboard-SDK` and `dji-sdk/Onboard-SDK-ROS` package.
1. Add `flight_control` package in your workspace's `/src` file.
1. `catkin_make` your workspace.
1. Make sure you have run a master node before the next step.
1. Run this demo use commands below:
 
```shell
$ source {$WORKSPACE_DIR}/devel/setup.bash
$ rosrun flight_control flight_control_gps
```

# IMU-based flight

This demo consists of three files : `flight_control_imu.h` , `flight_control_imu.cpp` and `GuidanceNode.cpp`,and uses imu data from guidance to locate drone.

## Variables
- `global_q` is a `Eigen::Quaterniond` variable,it is the accumulative quaterniond rotation after the drone taking off.
- `g` is a `Eigen::Vector3d` variable,it is a average value of 500 Imu acceleration when drone in ground.It is mainly used to eliminate the effect of gravity,but will still have accumulative error as it only caculate the first 500 data.
- `current_global_pos`  and `current_global_vel` is accumulative distance and velocity after drone taking off.Velocity acculated from Imu acceleration data using equation `v=a*dt`,and distance acculated from velocity using equation `x=v*dt+1/2*a*(dt)^2`.

## Functions
- `accumulate_distance()` rotate the Imu acceleration from local(relative to the drone's coordinate system) to global(relative to the world's coordinate system) frame.Then integrate accleration to caculate distance from the taking off point.
- `my_callback()` is a shared callback function used by subscriber threads.It will do a certain callback job according to `data_type` variable.
- `select_imu()` and those similar function are used to subscribe a topic,e.g. `select_imu()` will subscribe a Imu topic and get data  from Guidance who publish data to Imu topic.All these function will use `my_callback()` function as callback function.

You can visit [Guidance SDK Reference](https://developer.dji.com/guidance-sdk/documentation/introduction/index.html) to get more information.

## Installation

- Make sure you have installed the `flight_control` package,Guidance node will transfer location information to `demo_flight_control` node though `/guidance/global_position` topic.
- Make sure you have installed [Guidance-SDK-ROS](https://github.com/dji-sdk/Guidance-SDK-ROS) package.
- Replace 'GuidanceNode.cpp' with our file.
- `catkin_make` your workspace.
- If you are in simulation,make sure you have opened `DJI Assitant2` simulation and turned the remote control to `F` mode.
- Launch DJI master node:

```shell
$ source {$WORKSPACE_DIR}/devel/setup.bash
$ roslaunch dji_sdk sdk.launch
```
- `rosrun` Guidance node:

```shell
$ rosrun guidance guidancenode
```
- `rosrun` `demo_flight_control` node:

```shell
$ rosrun dji_sdk_demo demo_flight_control
```
- You can also use `rviz` to see its flight path(select our `flightcontrol.rviz` config file):

```shell
$ rosrun rviz rviz
```
