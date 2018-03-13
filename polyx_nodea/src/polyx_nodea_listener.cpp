/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "polyx_nodea/Icd.h"
#include "polyx_nodea/Geoid.h"
#include "polyx_nodea/EulerAttitude.h"
#include "polyx_nodea/Kalman.h"
#include "polyx_nodea/RawIMU.h"
#include "polyx_nodea/CorrectedIMU.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>

#include <geographic_msgs/GeoPoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>      /* Error number definitions */ 
#include <unistd.h>
#include <string.h>

#define  log_file "/tmp/gpstime.log"
#define RAD_TO_DEG 57.295779513082323

//int fd_log;
FILE *flog =NULL;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
 


void dumpICDMessage(const polyx_nodea::Icd::ConstPtr& msg)
{
 //  char sbuf[128];
//   sprintf(sbuf, "%lf\n", msg->GpsTimeWeek);
 //  write( fd_log, sbuf, strlen(sbuf) );
  if (flog) fprintf(flog, "%lf\n", msg->GpsTimeWeek);

  ROS_INFO("msg->GpsTimeOfWeek=%.3lf", msg->GpsTimeWeek);
  ROS_INFO("msg->Latitude=%.3lf", msg->Latitude);
  ROS_INFO("msg->Longitude=%.3lf", msg->Longitude);
  ROS_INFO("msg->Altitude=%e", msg->Altitude);
  ROS_INFO("msg->VelocityNED=[%e,%e,%e]", msg->VelocityNED[0], msg->VelocityNED[1], msg->VelocityNED[2]);
  ROS_INFO("msg->Attitude=[%e,%e,%e,%e]", msg->Quaternion[0],msg->Quaternion[1],msg->Quaternion[2],msg->Quaternion[3]);
  ROS_INFO("msg->Acceleration=[%e,%e,%e]", msg->Acceleration[0], msg->Acceleration[1], msg->Acceleration[2]);
  ROS_INFO("msg->RotationRate=[%e,%e,%e]", msg->RotationRate[0], msg->RotationRate[1], msg->RotationRate[2]);
  ROS_INFO("msg->PositionRMS=[%e,%e,%e]", msg->PositionRMS[0], msg->PositionRMS[1], msg->PositionRMS[2]);
  ROS_INFO("msg->VelocityRMS=[%e,%e,%e]", msg->VelocityRMS[0], msg->VelocityRMS[1], msg->VelocityRMS[2]);
  ROS_INFO("msg->AttitudeRMS=[%e,%e,%e]", msg->AttitudeRMS[0], msg->AttitudeRMS[1], msg->AttitudeRMS[2]);
  ROS_INFO("msg->GpsWeekNumber=%d", msg->GpsWeekNumber);
  ROS_INFO("msg->Alignment=%d\n", msg->Alignment);

}

void dumpRawIMUMessage(const polyx_nodea::RawIMU::ConstPtr& imsg)
{

  if (flog) fprintf(flog, "%lf\n", imsg->SystemTime);

  ROS_INFO("msg->SystemTime=%f", imsg->SystemTime);

  ROS_INFO("msg->Acceleration=[%f,%f,%f]", imsg->Acceleration[0], imsg->Acceleration[1], imsg->Acceleration[2]);
  ROS_INFO("msg->RotationRate=[%f,%f,%f]\n", imsg->RotationRate[0], imsg->RotationRate[1], imsg->RotationRate[2]);

}

void dumpCorrectedIMUMessage(const polyx_nodea::CorrectedIMU::ConstPtr& imsg)
{

  if (flog) fprintf(flog, "%lf\n", imsg->GpsTimeWeek);

  ROS_INFO("msg->GpsTimeWeek=%f", imsg->GpsTimeWeek);

  ROS_INFO("msg->Acceleration=[%f,%f,%f]", imsg->Acceleration[0], imsg->Acceleration[1], imsg->Acceleration[2]);
  ROS_INFO("msg->RotationRate=[%f,%f,%f]", imsg->RotationRate[0], imsg->RotationRate[1], imsg->RotationRate[2]);
  
  ROS_INFO("msg->GpsWeekNumber=%d\n", imsg->GpsWeekNumber);

}
 
void dumpGeoidMessage(const polyx_nodea::Geoid::ConstPtr& gmsg)
{
  if (flog) fprintf(flog, "%f\n", gmsg->GPSTime);

  ROS_INFO("gmsg->GPSTime=%f", gmsg->GPSTime);
  ROS_INFO("gmsg->GeoidHeight=%f\n", gmsg->GeoidHeight);

}

void dumpKalmanMessage(const polyx_nodea::Kalman::ConstPtr& kalmsg)
{
  if (flog) fprintf(flog, "%f\n", kalmsg->GPSTime);

  ROS_INFO("kalmsg->SystemTime=%f", kalmsg->SystemTime);
  ROS_INFO("kalmsg->GPSTime=%f", kalmsg->GPSTime);
  ROS_INFO("kalmsg->Latitude=%f", kalmsg->Latitude);
  ROS_INFO("kalmsg->Longitude=%f", kalmsg->Longitude);
  ROS_INFO("kalmsg->EllipsoidalHeight=%f", kalmsg->EllipsoidalHeight);
  ROS_INFO("kalmsg->VelocityNorth=%f", kalmsg->VelocityNorth);
  ROS_INFO("kalmsg->VelocityEast=%f", kalmsg->VelocityEast);
  ROS_INFO("kalmsg->VelocityDown=%f", kalmsg->VelocityDown);
  ROS_INFO("kalmsg->Roll=%f", kalmsg->Roll);
  ROS_INFO("kalmsg->Pitch=%f", kalmsg->Pitch);
  ROS_INFO("kalmsg->Heading=%f", kalmsg->Heading);
  ROS_INFO("kalmsg->PositionMode=%d", kalmsg->PositionMode);
  ROS_INFO("kalmsg->VelocityMode=%d", kalmsg->VelocityMode);
  ROS_INFO("kalmsg->AttitudeStatus=%d\n", kalmsg->AttitudeStatus);
}


void polyxCorrectedIMUCallback(const polyx_nodea::CorrectedIMU::ConstPtr& imsg)
{
  ROS_INFO(">>> Received a Corrected IMU Data message:");

  dumpCorrectedIMUMessage(imsg);
}


void polyxRawIMUCallback(const polyx_nodea::RawIMU::ConstPtr& imsg)
{
  ROS_INFO(">>> Received a Scaled Raw IMU Data message:");

  //ROS_INFO("message[%p], week=%d", gmsg, gmsg->GPSTime);
  dumpRawIMUMessage(imsg);

}

void polyxGeoidCallback(const polyx_nodea::Geoid::ConstPtr& gmsg)
{
  ROS_INFO(">>> Received a Geoid Height message:");

  //ROS_INFO("message[%p], week=%d", gmsg, gmsg->GPSTime);
  dumpGeoidMessage(gmsg);

}

void polyxKalmanCallback(const polyx_nodea::Kalman::ConstPtr& kalmsg) 
{
  ROS_INFO(">>> Received a Kalman Filter Navigation message:");

  //ROS_INFO("message[%p], week=%d", gmsg, gmsg->GPSTime);
  dumpKalmanMessage(kalmsg);
}


// %Tag(CALLBACK)%
void polyxICDCallback(const polyx_nodea::Icd::ConstPtr& msg)
{
  ROS_INFO(">>> Received an ICD message:");

  ROS_INFO("message[%p], week=%d\n", msg, msg->GpsWeekNumber);
	dumpICDMessage(msg);

}

//QuatToEuler listener
void EulerAttitudeCallback(const polyx_nodea::EulerAttitude::ConstPtr& qtemsg) 
{
  ROS_INFO(">>> Received an EulerAttitude message:");
  ROS_INFO("qtemsg->GpsTimeWeek=%f", qtemsg->GpsTimeWeek );
  ROS_INFO("qtemsg->roll=%f", qtemsg->roll * RAD_TO_DEG);
  ROS_INFO("qtemsg->pitch=%f", qtemsg->pitch * RAD_TO_DEG);
  ROS_INFO("qtemsg->heading=%f\n", qtemsg->heading * RAD_TO_DEG);
}

void PoseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO(">>> Received a geometry_msgs::PoseStamped message:");

  ROS_INFO("message[%p], time=%09u:%09u\n", msg, msg->header.stamp.sec, msg->header.stamp.nsec);
}

void TwistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  ROS_INFO(">>> Received a geometry_msgs::TwistStamped message:");

  ROS_INFO("message[%p], time=%09u:%09u\n", msg, msg->header.stamp.sec, msg->header.stamp.nsec);
}

void AccelStampedCallback(const geometry_msgs::AccelStamped::ConstPtr& msg)
{
  ROS_INFO(">>> Received a geometry_msgs::AccelStamped message:");

  ROS_INFO("message[%p], time=%09u:%09u\n", msg, msg->header.stamp.sec, msg->header.stamp.nsec);
}

void GeoPoseStampedCallback(const geographic_msgs::GeoPoseStamped::ConstPtr& msg)
{
  ROS_INFO(">>> Received a geographic_msgs::GeoPoseStamped message:");

  ROS_INFO("message[%p], time=%09u:%09u\n", msg, msg->header.stamp.sec, msg->header.stamp.nsec);
}

void NavSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  ROS_INFO(">>> Received a sensor_msgs::NavSatFix message:");

  ROS_INFO("message[%p], time=%09u:%09u\n", msg, msg->header.stamp.sec, msg->header.stamp.nsec);
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO(">>> Received a sensor_msgs::Imu message:");

  ROS_INFO("message[%p], time=%09u:%09u\n", msg, msg->header.stamp.sec, msg->header.stamp.nsec);
}


// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
//    fd_log = open(log_file, O_WRONLY | O_CREAT);
      flog = fopen( "~/gpstime.log", "wt");

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("polyx_ICD", 50, polyxICDCallback);

  ros::Subscriber pose_sub = n.subscribe("current_pose", 50, PoseStampedCallback);
  ros::Subscriber twist_sub = n.subscribe("current_velocity", 50, TwistStampedCallback);
  ros::Subscriber accel_sub = n.subscribe("current_acceleration", 50, AccelStampedCallback);

  ros::Subscriber geopose_sub = n.subscribe("current_geopose", 50, GeoPoseStampedCallback);

  ros::Subscriber navsatfix_sub = n.subscribe("current_navsatfix", 50, NavSatFixCallback);
  ros::Subscriber imu_sub = n.subscribe("current_imu", 50, ImuCallback);
  
  ros::Subscriber geoid_sub = n.subscribe("polyx_Geoid", 50, polyxGeoidCallback);
  ros::Subscriber EulerAttitude_sub = n.subscribe("polyx_EulerAttitude", 50, EulerAttitudeCallback);

  ros::Subscriber Kalman_sub = n.subscribe("polyx_Kalman", 50, polyxKalmanCallback);
  ros::Subscriber RawIMU_sub = n.subscribe("polyx_rawIMU", 50, polyxRawIMUCallback);
  ros::Subscriber CorrectedIMU_sub = n.subscribe("polyx_correctedIMU", 50, polyxCorrectedIMUCallback);
  
  // %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

//  close(fd_log);
  if (flog) fclose(flog);  
  
  
  ros::waitForShutdown();

return 0;
}
// %EndTag(FULLTEXT)%
