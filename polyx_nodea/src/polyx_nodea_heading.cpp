/*
 * Copyright (C) 2017, Polynesian Exploration Inc.
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
#include "ros/ros.h"
 // %EndTag(ROS_HEADER)%
 // %Tag(MSG_HEADER)%
#include "std_msgs/String.h"

#include "polyx_nodea/StaticHeadingEvent.h"


/**
 * This tutorial demonstrates how to send PolyNav static heading event over the ROS system.
 */
int main(int argc, char **argv)
{
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
    // %Tag(INIT)%
   ros::init(argc, argv, "talker");
   // %EndTag(INIT)%

     /**
      * NodeHandle is the main access point to communications with the ROS system.
      * The first NodeHandle constructed will fully initialize this node, and the last
      * NodeHandle destructed will close down the node.
      */
      // %Tag(NODEHANDLE)%
   ros::NodeHandle n;
   // %EndTag(NODEHANDLE)%

   int count = 0;

   /**
    * The advertise() function is how you tell ROS that you want to
    * publish on a given topic name. This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing. After this advertise() call is made, the master
    * node will notify anyone who is trying to subscribe to this topic name,
    * and they will in turn negotiate a peer-to-peer connection with this
    * node.  advertise() returns a Publisher object which allows you to
    * publish messages on that topic through a call to publish().  Once
    * all copies of the returned Publisher object are destroyed, the topic
    * will be automatically unadvertised.
    *
    * The second parameter to advertise() is the size of the message queue
    * used for publishing messages.  If messages are published more quickly
    * than we can send them, the number here specifies how many messages to
    * buffer up before throwing some away.
    */
    // %Tag(PUBLISHER)%
   ros::Publisher chatter_pub = n.advertise<polyx_nodea::StaticHeadingEvent>("polyx_StaticHeading", 1000);
   // %EndTag(PUBLISHER)%

   // %Tag(LOOP_RATE)%
   ros::Rate loop_rate(300);
   // %EndTag(LOOP_RATE)%

   polyx_nodea::StaticHeadingEvent msg;

   double duration = 5.0; // seconds

   // Default message for testing

   msg.Heading = -11600; // -116 degrees
   msg.ZUPTRMS = 10;     // 1 cm/s
   msg.HeadingRMS = 100; // 10 degrees

   // Parse message contents from command line arguments
   if (argc == 5)
   {
      // convet deg to 0.01 deg
      msg.Heading = static_cast<short int>(atof(argv[1]) * 100);

      // convert m/s to mm/s
      msg.ZUPTRMS = static_cast<unsigned short int>(atof(argv[2]) * 1000);

      // convert deg to 0.1 deg
      msg.HeadingRMS = static_cast<unsigned char>(atof(argv[3]) * 10);

      duration = atof(argv[4]);
   }

   double start_time = ros::Time::now().toSec();

   /*
   *  Start the talker part to check the serial port for data
   */
   while (ros::ok())
   {

      msg.header.stamp = ros::Time::now();

      if (msg.header.stamp.toSec() - start_time > duration)
         break;

      ROS_INFO("%d, %d", count++, msg.Heading);

      // %EndTag(ROSCONSOLE)%
      // %Tag(PUBLISH)%
      chatter_pub.publish(msg);
      // %EndTag(PUBLISH)%

   loop_ros:

      // %Tag(SPINONCE)%
      ros::spinOnce();
      // %EndTag(SPINONCE)%

      // %Tag(RATE_SLEEP)%
      loop_rate.sleep();
      // %EndTag(RATE_SLEEP)%
   }

   return 0;
}
// %EndTag(FULLTEXT)%

