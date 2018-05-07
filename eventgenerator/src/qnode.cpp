/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>

#include "../include/eventgenerator/qnode.hpp"
#include "../include/eventgenerator/main_window.hpp"


 // %EndTag(ROS_HEADER)%
 // %Tag(MSG_HEADER)%


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>


#include "polyx_nodea/StaticGeoPoseEvent.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

//#include "polyxdata.h"

#include <math.h>

// %EndTag(MSG_HEADER)%

#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>  
#include <unistd.h>


#include <termios.h>  

#include <time.h> 

int count = 0;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace eventgenerator {

/*****************************************************************************
** Implementation
*****************************************************************************/

	QNode::QNode(int argc, char** argv ):
		init_argc(argc),
		init_argv(argv)
		{}

	QNode::~QNode() {
		if(ros::isStarted()) {
		  ros::shutdown(); // explicitly needed since we use ros::start();
		  ros::waitForShutdown();
		}
		wait();
	}

	bool QNode::init() {
		ros::init(init_argc, init_argv, "eventgenerator");
		if (!ros::master::check()) {
			return false;
		}
		ros::start(); // explicitly needed since our nodehandle is going out of scope.
		ros::NodeHandle n;
		// Add your ros communications here.
		staticGeoPose_pub = n.advertise<polyx_nodea::StaticGeoPoseEvent>("polyx_StaticGeoPose", 1000);

		start();
		return true;
	}

	bool QNode::init(const std::string &master_url, const std::string &host_url) {
		std::map<std::string,std::string> remappings;
		remappings["__master"] = master_url;
		remappings["__hostname"] = host_url;
		ros::init(remappings, "eventgenerator");
		if (!ros::master::check()) {
			return false;
		}
		ros::start(); // explicitly needed since our nodehandle is going out of scope.
		ros::NodeHandle n;
		// Add your ros communications here.
		staticGeoPose_pub = n.advertise<polyx_nodea::StaticGeoPoseEvent>("polyx_StaticGeoPose", 1000);

		start();
		return true;
	}

	void QNode::run() {
		ros::Rate loop_rate(300);

		activate = true;

		// msg.Latitude = 37.405109067;		// 37.405109067 degrees
		// msg.Longitude = -121.918100758;  	// -121.918100758 degrees
		// msg.EllipsoidalHeight = -10.136; 	// -10.136 m
		// msg.Roll = 0;						// 0 degree
		// msg.Pitch = 0; 						// 0 degree
		// msg.Heading = 3000; 				// 30.00 degrees
		// msg.PositionRMS = 200;				// 2.00 m
		// msg.ZUPTRMS = 10;					// 0.010 m	
		// msg.HeadingRMS = 100;    			// 10 degrees
		// msg.Flags = 0;  					// 0

		while (ros::ok()) {

			if (!activate) break;

		    printf("Sent a StaticGeoPose message, count = %d\n", ++count);
		  	ROS_INFO("Latitude: %.9lf deg", msg.Latitude);
		    ROS_INFO("Longitude: %.9lf deg",  msg.Longitude);
		    ROS_INFO("EllipsoidalHeight: %.3lf m",  msg.EllipsoidalHeight);
		    ROS_INFO("Roll: %.2lf deg",  msg.Roll / float (100));
		    ROS_INFO("Pitch: %.2lf deg",  msg.Pitch / float (100));
		    ROS_INFO("Heading: %.2lf deg",  msg.Heading / float (100));
		    ROS_INFO("PositionRMS: %.2lf m",  msg.PositionRMS / float (100));
		    ROS_INFO("ZUPTRMS: %.3lf m/s",  msg.ZUPTRMS / float (1000));
		    ROS_INFO("HeadingRMS: %.1lf deg",  msg.HeadingRMS / float (10));
		    ROS_INFO("Roll & Pitch Valid, disable GNSS: %d\n",  msg.Flags);


		    // %EndTag(ROSCONSOLE)%
		    // %Tag(PUBLISH)%
		    staticGeoPose_pub.publish(msg);

		    // %Tag(SPINONCE)%
		    ros::spinOnce();
		    // %EndTag(SPINONCE)%

		    loop_ros:
		    // %Tag(RATE_SLEEP)%
		    loop_rate.sleep();
		    // %EndTag(RATE_SLEEP)%
		}//end while
		std::cout << "Stop sending messages." << std::endl;
	}
}  // namespace eventgenerator
