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
#include "std_msgs/String.h"
#include "polyx_nodea/StaticGeoPoseEvent.h"


bool isNumber (const char *value)
{
	int i;
	long res;
	char *next;

	res = strtol(value, &next, 10);

	if ((next == value) || ((*next != '\0') && (*next != '.')))
		return false;
	else
		return true;
}

bool isNegative (const char *value)
{
	if (*value == '-') 
	{
		if (*(value+1) >= '0' && *(value+1) <= '9')		
			return true;		
		else		
			return false;
		
	}
	return false;
}

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
   ros::Publisher staticGeoPose_pub = n.advertise<polyx_nodea::StaticGeoPoseEvent>("polyx_StaticGeoPose", 1000);
   // %EndTag(PUBLISHER)%

   // %Tag(LOOP_RATE)%
   ros::Rate loop_rate(3);
   // %EndTag(LOOP_RATE)%

   polyx_nodea::StaticGeoPoseEvent msg;
   int count = 0;
   bool option;
   double duration = 5.0; // seconds

   // Default message for testing

   msg.Latitude = 0;		
   msg.Longitude = 0;  
   msg.EllipsoidalHeight = 0; 
   msg.Roll = 0;					
   msg.Pitch = 0; 					
   msg.Heading = 0; 				
   msg.PositionRMS = 0;			
   msg.ZUPTRMS = 10;					
   msg.HeadingRMS = 0;    		
   msg.Flags = 0;  					

   option = false;
	
   for (int i = 0; i < argc; i++) 
   {
   		if (argv[i][0] == '-') 
   		{
   			if (isdigit(argv[i][1]))
   			{
   				if (!option)
	   			{
	   				printf("Please enter a command-line option first\n");
	   				return -1;
	   			}
	   			continue;
   			} 
   			else
   			{
   				
   				switch(*(argv[i]+1))
   				{
   					case 'p':
   						for (int j = 1; j <= 4; j++)
				    	{
				    		if (i + j >= argc || (!isNumber(argv[i+j]) && !isNegative(argv[i+j])))
				    		{
				    			printf("Invalid command line arguments after -p option\n");
				    			return -1;
				    		}
				    	}
				    	if (i + 5 < argc && (isNegative(argv[i+5]) || *argv[i+5] != '-'))
		    			{
		    				printf("too many arguments after -p option\n");
		    				return -1;
		    			}

		    			double lat, lon;
		    			float posRMS;
		    			lat = static_cast<double>(atof(argv[i+1]));
		    			if (lat > 90.00 || lat < -90.00)	
		    			{
		    				printf("Invalid Latitude\n");
		    				return -1;
		    			}	
		    			lon = static_cast<double>(atof(argv[i+2]));
		    			if (lon > 180.00 || lon < -180.00)	
		    			{
		    				printf("Invalid Longitude\n");
		    				return -1;
		    			}
		    			posRMS = static_cast<float>(atof(argv[i+4]));
		    			if (posRMS < 0 || posRMS > 655.35)	
		    			{
		    				printf("Invalid PositionRMS\n");
		    				return -1;
		    			}
				    	msg.Latitude = lat;
						msg.Longitude = lon;
						msg.EllipsoidalHeight = static_cast<float>(atof(argv[i+3]));
						msg.PositionRMS = posRMS * 100;
						option = true;
						break;
					case 'z':
   						if (i + 1 >= argc || (!isNumber(argv[i+1]) && !isNegative(argv[i+1])))
			    		{
			    			printf("Invalid command line arguments after -z option\n");
			    			return -1;
			    		}
			    		if (i + 2 < argc && (isNegative(argv[i+2]) || *argv[i+2] != '-'))
		    			{
		    				printf("too many arguments after -z option\n");
		    				return -1;
		    			}

		    			float zuptRMS;
		    			zuptRMS = static_cast<float>(atof(argv[i+1]));
		    			if (zuptRMS < 0 || zuptRMS > 65.535)	
		    			{
		    				printf("Invalid ZUPT RMS\n");
		    				return -1;
		    			}
				    	msg.ZUPTRMS = zuptRMS * 1000;
				    	option = true;
						break;
					case 'h':
   						for (int j = 1; j <= 2; j++)
				    	{
				    		if (i + j >= argc || (!isNumber(argv[i+j]) && !isNegative(argv[i+j])))
				    		{
				    			printf("Invalid command line arguments after -h option\n");
				    			return -1;
				    		}
				    	}
				    	if (i + 3 < argc && (isNegative(argv[i+3]) || *argv[i+3] != '-'))
		    			{
		    				printf("too many arguments after -h option\n");
		    				return -1;
		    			}
		    			short int heading;
		    			float headingRMS;
		    			heading = static_cast<short int>(atof(argv[i+1]) * 100);
		    			if (heading < -18000 || heading > 18000)	
		    			{
		    				printf("Invalid heading\n");
		    				return -1;
		    			}
		    			headingRMS = static_cast<float>(atof(argv[i+2]));
		    			if (headingRMS < 0 || headingRMS > 25.5)	
		    			{
		    				printf("Invalid headingRMS\n");
		    				return -1;
		    			}
				    	msg.Heading = heading;
						msg.HeadingRMS = headingRMS * 10;
						option = true;
						break;
					case 'd':
   						if (i + 1 >= argc || (!isNumber(argv[i+1]) && !isNegative(argv[i+1])))
			    		{
			    			printf("Invalid command line arguments after -d option\n");
			    			return -1;
			    		}
			    		if (i + 2 < argc && (isNegative(argv[i+2]) || *argv[i+2] != '-'))
		    			{
		    				printf("too many arguments after -d option\n");
		    				return -1;
		    			}
				    	duration = atof(argv[i+1]);
				    	option = true;
						break;
					case 't':
   						for (int j = 1; j <= 2; j++)
				    	{
				    		if (i + j >= argc || (!isNumber(argv[i+j]) && !isNegative(argv[i+j])))
				    		{
				    			printf("Invalid command line arguments after -t option\n");
				    			return -1;
				    		}
				    	}
				    	if (i + 3 < argc && (isNegative(argv[i+3]) || *argv[i+3] != '-'))
		    			{
		    				printf("too many arguments after -t option\n");
		    				return -1;
		    			}
		    			short int roll, pitch;
		    			roll = static_cast<short int>(atof(argv[i+1]) * 100);
		    			if (roll < -18000 || roll > 18000)	
		    			{
		    				printf("Invalid roll\n");
		    				return -1;
		    			}
		    			pitch = static_cast<short int>(atof(argv[i+2]) * 100);
		    			if (pitch < -9000 || pitch > 9000)	
		    			{
		    				printf("Invalid pitch\n");
		    				return -1;
		    			}
		    			msg.Flags |= 0x01;
				    	msg.Roll = roll;
						msg.Pitch = pitch;
						option = true;
						break;
					case 'g':
   						if (i + 1 < argc && *argv[i+1] != '-')
			    		{
			    			printf("Invalid command line arguments after -g option\n");
			    			return -1;
			    		}		    	
				    	msg.Flags |= 0x02;
				    	option = true;
						break;
					default:
						printf("Invalid command-line option argument\n");
						return -1;
   				}
   			}
   		}
   		else if (isdigit(argv[i][0]))
		{
			if (!option)
			{
				printf("Please enter a command-line option first\n");
				return -1;
			}
			continue;
		}
   		
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

      printf("Sent a StaticGeoPose message, count = %d\n", ++count);
      ROS_INFO("Latitude: %.9lf deg", msg.Latitude);
      ROS_INFO("Longitude: %.9lf deg",  msg.Longitude);
      ROS_INFO("EllipsoidalHeight: %.3lf m",  msg.EllipsoidalHeight);
      ROS_INFO("Roll: %.2lf deg",  msg.Roll/(float)100);
      ROS_INFO("Pitch: %.2lf deg",  msg.Pitch/(float)100);
      ROS_INFO("Heading: %.2lf deg",  msg.Heading/(float)100);
      ROS_INFO("PositionRMS: %.2lf m",  msg.PositionRMS/(float)100);
      ROS_INFO("ZUPTRMS: %.3lf m/s",  msg.ZUPTRMS/(float)1000);
      ROS_INFO("HeadingRMS: %.1lf deg",  msg.HeadingRMS/(float)10);
      ROS_INFO("Roll & Pitch Valid, disable GNSS: %d\n",  msg.Flags);

      // %EndTag(ROSCONSOLE)%
      // %Tag(PUBLISH)%
      staticGeoPose_pub.publish(msg);
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

