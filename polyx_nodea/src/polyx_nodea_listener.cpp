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

#include "polyx_nodea/Kalman.h"
#include "polyx_nodea/RawIMU.h"
#include "polyx_nodea/SolutionStatus.h"
#include "polyx_nodea/Icd.h"
#include "polyx_nodea/EulerAttitude.h"
#include "polyx_nodea/TimeSync.h"
#include "polyx_nodea/Geoid.h"
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
 
void dumpKalmanMessage(const polyx_nodea::Kalman::ConstPtr& kalmsg)
{
  if (flog) fprintf(flog, "%f\n", kalmsg->GPSTime);

  ROS_INFO("SystemTime=%f", kalmsg->SystemTime);
  ROS_INFO("GPSTime=%f", kalmsg->GPSTime);
  ROS_INFO("Latitude=%.9lf", kalmsg->Latitude*RAD_TO_DEG);
  ROS_INFO("Longitude=%.9lf", kalmsg->Longitude*RAD_TO_DEG);
  ROS_INFO("EllipsoidalHeight=%f", kalmsg->EllipsoidalHeight);
  ROS_INFO("VelocityNorth=%f", kalmsg->VelocityNorth);
  ROS_INFO("VelocityEast=%f", kalmsg->VelocityEast);
  ROS_INFO("VelocityDown=%f", kalmsg->VelocityDown);
  ROS_INFO("Roll=%f", kalmsg->Roll);
  ROS_INFO("Pitch=%f", kalmsg->Pitch);
  ROS_INFO("Heading=%f", kalmsg->Heading);
  ROS_INFO("PositionMode=%d", kalmsg->PositionMode);
  ROS_INFO("VelocityMode=%d", kalmsg->VelocityMode);
  ROS_INFO("AttitudeStatus=%d\n", kalmsg->AttitudeStatus);
}

void dumpRawIMUMessage(const polyx_nodea::RawIMU::ConstPtr& imsg)
{

  if (flog) fprintf(flog, "%lf\n", imsg->SystemTime);

  ROS_INFO("SystemTime=%f", imsg->SystemTime);
  ROS_INFO("Acceleration=[%f,%f,%f]", imsg->Acceleration[0], imsg->Acceleration[1], imsg->Acceleration[2]);
  ROS_INFO("RotationRate=[%f,%f,%f]\n", imsg->RotationRate[0], imsg->RotationRate[1], imsg->RotationRate[2]);

}

void dumpSolutionStatus(const polyx_nodea::SolutionStatus::ConstPtr& smsg)
{

  if (flog) fprintf(flog, "%lf\n", smsg->SystemTime);

  ROS_INFO("SystemTime=%f", smsg->SystemTime);
  ROS_INFO("GpsWeekNumber=%d", smsg->GpsWeekNumber);
  ROS_INFO("NumberOfSVs=%d", smsg->NumberOfSVs);
  ROS_INFO("ProcessingMode=%d", smsg->ProcessingMode);
  ROS_INFO("GpsTimeWeek=%f", smsg->GpsTimeWeek);
  ROS_INFO("PositionRMS=[%f,%f,%f]", smsg->PositionRMS[0], smsg->PositionRMS[1], smsg->PositionRMS[2]);
  ROS_INFO("VelocityRMS=[%f,%f,%f]", smsg->VelocityRMS[0], smsg->VelocityRMS[1], smsg->VelocityRMS[2]);
  ROS_INFO("AttitudeRMS=[%f,%f,%f]\n", smsg->AttitudeRMS[0], smsg->AttitudeRMS[1], smsg->AttitudeRMS[2]);
}

void dumpICDMessage(const polyx_nodea::Icd::ConstPtr& msg)
{
  if (flog) fprintf(flog, "%lf\n", msg->GpsTimeWeek);

  ROS_INFO("GpsTimeOfWeek=%.3lf", msg->GpsTimeWeek);
  ROS_INFO("Latitude=%.9lf", msg->Latitude * RAD_TO_DEG);
  ROS_INFO("Longitude=%.9lf", msg->Longitude * RAD_TO_DEG);
  ROS_INFO("Altitude=%.3lf", msg->Altitude);
  ROS_INFO("VelocityNED=[%.3lf,%.3lf,%.3lf]", msg->VelocityNED[0], msg->VelocityNED[1], msg->VelocityNED[2]);
  ROS_INFO("Attitude=[%e,%e,%e,%e]", msg->Quaternion[0],msg->Quaternion[1],msg->Quaternion[2],msg->Quaternion[3]);
  ROS_INFO("Acceleration=[%e,%e,%e]", msg->Acceleration[0], msg->Acceleration[1], msg->Acceleration[2]);
  ROS_INFO("RotationRate=[%e,%e,%e]", msg->RotationRate[0], msg->RotationRate[1], msg->RotationRate[2]);
  ROS_INFO("PositionRMS=[%.3lf,%.3lf,%.3lf]", msg->PositionRMS[0], msg->PositionRMS[1], msg->PositionRMS[2]);
  ROS_INFO("VelocityRMS=[%.3lf,%.3lf,%.3lf]", msg->VelocityRMS[0], msg->VelocityRMS[1], msg->VelocityRMS[2]);
  ROS_INFO("AttitudeRMS=[%.3lf,%.3lf,%.3lf]", msg->AttitudeRMS[0], msg->AttitudeRMS[1], msg->AttitudeRMS[2]);
  ROS_INFO("GpsWeekNumber=%d", msg->GpsWeekNumber);
  ROS_INFO("Alignment=%d\n", msg->Alignment);

}

void dumpTimeSyncMessage(const polyx_nodea::TimeSync::ConstPtr& tsmsg)
{
  if (flog) fprintf(flog, "%f\n", tsmsg->SystemComputerTime);

  ROS_INFO("System Computer Time=%f", tsmsg->SystemComputerTime);
  ROS_INFO("Bias with respect To GPS Time=%f\n", tsmsg->BiasToGPSTime);

}

void dumpGeoidMessage(const polyx_nodea::Geoid::ConstPtr& gmsg)
{
  if (flog) fprintf(flog, "%f\n", gmsg->GPSTime);

  ROS_INFO("GPSTime=%f", gmsg->GPSTime);
  ROS_INFO("GeoidHeight=%f\n", gmsg->GeoidHeight);

}

void dumpCorrectedIMUMessage(const polyx_nodea::CorrectedIMU::ConstPtr& cimsg)
{

  if (flog) fprintf(flog, "%lf\n", cimsg->GpsTimeWeek);

  ROS_INFO("GpsTimeWeek=%f", cimsg->GpsTimeWeek);
  ROS_INFO("Acceleration=[%f,%f,%f]", cimsg->Acceleration[0], cimsg->Acceleration[1], cimsg->Acceleration[2]);
  ROS_INFO("RotationRate=[%f,%f,%f]", cimsg->RotationRate[0], cimsg->RotationRate[1], cimsg->RotationRate[2]);
  ROS_INFO("GpsWeekNumber=%d\n", cimsg->GpsWeekNumber);

}


// %Tag(CALLBACK)%
void polyxKalmanCallback(const polyx_nodea::Kalman::ConstPtr& kalmsg) 
{
  ROS_INFO(">>> Received a Kalman Filter Navigation message:");

  dumpKalmanMessage(kalmsg);
}

void polyxRawIMUCallback(const polyx_nodea::RawIMU::ConstPtr& imsg)
{
  ROS_INFO(">>> Received a Scaled Raw IMU Data message:");

  dumpRawIMUMessage(imsg);

}

void polyxSolutionStatusCallback(const polyx_nodea::SolutionStatus::ConstPtr& smsg)
{
  ROS_INFO(">>> Received a Solution Status message:");

  dumpSolutionStatus(smsg);
}

void polyxICDCallback(const polyx_nodea::Icd::ConstPtr& msg)
{
  ROS_INFO(">>> Received an ICD message:");

  ROS_INFO("message[%p], week=%d\n", msg, msg->GpsWeekNumber);
  dumpICDMessage(msg);
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

void EulerAttitudeCallback(const polyx_nodea::EulerAttitude::ConstPtr& qtemsg) 
{
  ROS_INFO(">>> Received an EulerAttitude message:");
  ROS_INFO("GpsTimeWeek=%f", qtemsg->GpsTimeWeek );
  ROS_INFO("roll=%f", qtemsg->roll * RAD_TO_DEG);
  ROS_INFO("pitch=%f", qtemsg->pitch * RAD_TO_DEG);
  ROS_INFO("heading=%f\n", qtemsg->heading * RAD_TO_DEG);
}

void polyxTimeSyncCallback(const polyx_nodea::TimeSync::ConstPtr& tsmsg)
{
  ROS_INFO(">>> Received a Time Sync message:");

  dumpTimeSyncMessage(tsmsg);
}

void polyxGeoidCallback(const polyx_nodea::Geoid::ConstPtr& gmsg)
{
  ROS_INFO(">>> Received a Geoid Height message:");

  dumpGeoidMessage(gmsg);

}

void polyxCorrectedIMUCallback(const polyx_nodea::CorrectedIMU::ConstPtr& cimsg)
{
  ROS_INFO(">>> Received a Corrected IMU Data message:");

  dumpCorrectedIMUMessage(cimsg);
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
  ros::Subscriber Kalman_sub = n.subscribe("polyx_Kalman", 50, polyxKalmanCallback);

  ros::Subscriber RawIMU_sub = n.subscribe("polyx_rawIMU", 50, polyxRawIMUCallback);

  ros::Subscriber SolutionStatus_sub = n.subscribe("polyx_solutionStatus", 50, polyxSolutionStatusCallback);
  
  ros::Subscriber icd_sub = n.subscribe("polyx_ICD", 50, polyxICDCallback);
  ros::Subscriber pose_sub = n.subscribe("current_pose", 50, PoseStampedCallback);
  ros::Subscriber twist_sub = n.subscribe("current_velocity", 50, TwistStampedCallback);
  ros::Subscriber accel_sub = n.subscribe("current_acceleration", 50, AccelStampedCallback);
  ros::Subscriber geopose_sub = n.subscribe("current_geopose", 50, GeoPoseStampedCallback);
  ros::Subscriber navsatfix_sub = n.subscribe("current_navsatfix", 50, NavSatFixCallback);
  ros::Subscriber imu_sub = n.subscribe("current_imu", 50, ImuCallback);
  ros::Subscriber EulerAttitude_sub = n.subscribe("polyx_EulerAttitude", 50, EulerAttitudeCallback);
  
  ros::Subscriber timeSync_sub = n.subscribe("polyx_timeSync", 50, polyxTimeSyncCallback);

  ros::Subscriber geoid_sub = n.subscribe("polyx_Geoid", 50, polyxGeoidCallback);

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
