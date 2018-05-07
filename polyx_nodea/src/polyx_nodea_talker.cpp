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

#include "polyx_nodea/Kalman.h"
#include "polyx_nodea/RawIMU.h"
#include "polyx_nodea/SolutionStatus.h"
#include "polyx_nodea/Icd.h"
#include "polyx_nodea/EulerAttitude.h"
#include "polyx_nodea/TimeSync.h"
#include "polyx_nodea/Geoid.h"
#include "polyx_nodea/CorrectedIMU.h"

#include "polyx_nodea/WheelSpeedReport.h"
#include "polyx_nodea/StaticHeadingEvent.h"
#include "polyx_nodea/StaticGeoPoseEvent.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>


#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include "polyxdata.h"

#include <math.h>

// %EndTag(MSG_HEADER)%

#include <sstream>
//
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>      /* Error number definitions */ 
#include <unistd.h>
#include <string.h>

#include <termios.h>    /* POSIX terminal control definitions */

#include <time.h> 

#include "polyx_convert.h"

#define MON_PORT "/dev/ttyUSB1"
#define ICD_SYNC1 0xAF
#define ICD_SYNC2 0x20
#define ICD_TYPE 5

#define SERIAL_WAIT (1000000/46080)
#define SERIAL_TIMEOUT 10   // seconds

#define SERIAL_BAUD 230400


// global serial port file descriptor
int fd_mon = -1;
static const int MaxMsgLen = 2048;

uint8_t checksum(uint8_t* Buffer, uint16_t len, uint8_t& cka, uint8_t& ckb)
{
   unsigned char CK_A = 0;
   unsigned char CK_B = 0;

   if (Buffer)
   {
      for (int i = 0; i < len; i++)
      {
         CK_A = CK_A + Buffer[i];
         CK_B = CK_B + CK_A;
      }
      cka = CK_A;
      ckb = CK_B;
      return 1;
   }
   else
      return 0;
}


int write_port(int fd, void *buf, int len)
{
   struct timeval tout;
   // default uart read timeout
   tout.tv_sec = 0;
   tout.tv_usec = 500 * 1000;

   fd_set outputs;
   int num, ret;

   num = 0;

   FD_ZERO(&outputs);
   FD_SET(fd, &outputs);

   ret = select(fd + 1, (fd_set *)NULL, &outputs, (fd_set *)NULL, &tout);
   if (ret < 0) {
      perror("select error!!");
      return ret;
   }
   if (ret > 0) {
      if (FD_ISSET(fd, &outputs)) {
         num = write(fd, buf, len);
      }
   }
   return num;
}


int read_port(int fd, uint8_t *buf, size_t len, struct timeval *tout)
{
   fd_set inputs;
   int num, ret;

   num = 0;

   FD_ZERO(&inputs);
   FD_SET(fd, &inputs);

   ret = select(fd + 1, &inputs, (fd_set *)NULL, (fd_set *)NULL, tout);
   //printf("select = %d\n", ret);
   if (ret < 0) {
      perror("select error!!");
      return ret;
   }
   if (ret > 0) {
      if (FD_ISSET(fd, &inputs)) {
         size_t len = 0;
         int errsav;
         ioctl(fd, FIONREAD, &len);
         errsav = errno;
         if (len == 0) {
            return -EIO;
         }

         num = read(fd, buf, len);
      }
   }
   return num;
}


int open_mon(const char *port, int baud)
{
   struct termios options;                                             // Structure with the device's options
   int fd;
   // Open device
   fd = open(port, O_RDWR | O_NONBLOCK | O_NOCTTY);                    // Open port
   if (fd < 0)
   {
      perror("serial open");
      return fd;
   }                                            // If the device is not open, return -1
//	fcntl(fd, F_SETFL, FNDELAY);                                        // Open the device in nonblocking mode
                                                      // Set parameters
   tcgetattr(fd, &options);                                            // Get the current options of the port
   bzero(&options, sizeof(options));                                   // Clear all the options
   speed_t         Speed;

   switch (baud)                                                      // Set the speed (Bauds)
   {
   case 110:      Speed = B110; break;
   case 300:      Speed = B300; break;
   case 600:      Speed = B600; break;
   case 1200:     Speed = B1200; break;
   case 2400:     Speed = B2400; break;
   case 4800:     Speed = B4800; break;
   case 9600:     Speed = B9600; break;
   case 19200:    Speed = B19200; break;
   case 38400:    Speed = B38400; break;
   case 57600:    Speed = B57600; break;
   case 115200:   Speed = B115200; break;
   case 230400:   Speed = B230400; break;
   case 460800:   Speed = B460800; break;
   case 921600:   Speed = B921600; break;
   default:       Speed = baud;
   }
   cfsetispeed(&options, Speed);                                       // Set the baud rate at 115200 bauds
   cfsetospeed(&options, Speed);

   options.c_cflag |= (CLOCAL | CREAD | CS8);                        // Configure the device : 8 bits, no parity, no control
   options.c_iflag |= (IGNPAR | IGNBRK);
   options.c_cc[VTIME] = 0;                                              // Timer unused
   options.c_cc[VMIN] = 0;                                               // At least on character before satisfy reading

   options.c_cflag |= CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
   options.c_iflag &= ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
   options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
   options.c_oflag &= ~OPOST;              // make raw

   tcsetattr(fd, TCSANOW, &options);                                   // Activate the settings

   return fd;  // successful
}

#define DEG_TO_RAD (0.017453292519943295)

uint8_t checkMessageType(uint8_t *buf)
{
   int r = -1;
   int len = 0;
   uint8_t c1, c2;
   uint8_t messageType;

   len = buf[5];
   len = (len << 8) + buf[4];  // payload length

   checksum(buf + 6, len, c1, c2);

   if ((c1 == buf[len + 6]) && (c2 == buf[len + 7]))
   {
      messageType = buf[2];
   }
   else 
   {
      printf("invalid message.\n");
      messageType = 0;
   }
   return messageType;
}


void parse_Kalman_message(uint8_t *buf, polyx_nodea::Kalman &kalmsg)
{
   struct kalmanmessage *km = (struct kalmanmessage*)buf;

   // copy message

   kalmsg.SystemTime = km->sysTime;
   kalmsg.GPSTime = km->GPSTime;
   kalmsg.Latitude = km->lat;
   kalmsg.Longitude = km->lon;
   kalmsg.EllipsoidalHeight = km->ellHeight;
   kalmsg.VelocityNorth = km->velNorth;
   kalmsg.VelocityEast = km->velEast;
   kalmsg.VelocityDown = km->velDown;
   kalmsg.Roll = km->roll;
   kalmsg.Pitch = km->pitch;
   kalmsg.Heading = km->heading;
   kalmsg.PositionMode = km->posMode;
   kalmsg.VelocityMode = km->velMode;
   kalmsg.AttitudeStatus = km->attStatus;

}

void parse_RawIMU_message(uint8_t *buf, polyx_nodea::RawIMU &imsg)
{
   struct rawImuMessage *im = (struct rawImuMessage*)buf;

   // copy message

   int i;

   imsg.SystemTime = im->sysTime;
   for (i = 0; i < 3; i++) imsg.Acceleration[i] = im->acc[i];
   for (i = 0; i < 3; i++) imsg.RotationRate[i] = im->rotRate[i];

}

void parse_SolutionStatus_message(uint8_t *buf, polyx_nodea::SolutionStatus &smsg)
{
   struct solutionStatusMessage *im = (struct solutionStatusMessage*)buf;

   // copy message

   int i;

   smsg.SystemTime = im->systemTime;
   smsg.GpsWeekNumber = im->week;
   smsg.NumberOfSVs = im->NumberOfSVs;
   smsg.ProcessingMode = im->ProcessingMode;
   smsg.GpsTimeWeek = im->GPSTimeWeek;
   for (i = 0; i < 3; i++) smsg.PositionRMS[i] = im->PositionRMS[i];
   for (i = 0; i < 3; i++) smsg.VelocityRMS[i] = im->VelocityRMS[i];
   for (i = 0; i < 3; i++) smsg.AttitudeRMS[i] = im->AttitudeRMS[i];

}

void parse_Icd_message(uint8_t *buf, polyx_nodea::Icd &msg)
{
   struct icdmessage *im = (struct icdmessage*)buf;

   // copy message

   int i;

   msg.GpsTimeWeek = im->tow;
   msg.Latitude = im->lat*DEG_TO_RAD;
   msg.Longitude = im->lon*DEG_TO_RAD;
   msg.Altitude = im->alt;
   for (i = 0; i < 3; i++) msg.VelocityNED[i] = im->vel[i];
   for (i = 0; i < 4; i++) msg.Quaternion[i] = im->q_bn[i];
   for (i = 0; i < 3; i++) msg.Acceleration[i] = im->acc[i];
   for (i = 0; i < 3; i++) msg.RotationRate[i] = im->rot_rate[i] * DEG_TO_RAD;
   for (i = 0; i < 3; i++) msg.PositionRMS[i] = im->pos_rms[i];
   for (i = 0; i < 3; i++) msg.VelocityRMS[i] = im->vel_rms[i];
   for (i = 0; i < 3; i++) msg.AttitudeRMS[i] = im->att_rms[i] * DEG_TO_RAD;
   msg.GpsWeekNumber = im->week;
   msg.Alignment = im->align_mode;

   if (msg.GpsWeekNumber > 0)
      GpsToEpoch(msg.GpsWeekNumber, msg.GpsTimeWeek, msg.header.stamp);
   else
   	  msg.header.stamp = ros::Time::now();


}

void parse_TimeSync_message(uint8_t *buf, polyx_nodea::TimeSync &tsmsg)
{
   struct timeSyncmessage *tsm = (struct timeSyncmessage*)buf;

   // copy message

   tsmsg.SystemComputerTime = tsm->systemComTime;
   tsmsg.BiasToGPSTime = tsm->biasToGPSTime;

}

void parse_Geoid_message(uint8_t *buf, polyx_nodea::Geoid &gmsg)
{
   struct geoidmessage *gm = (struct geoidmessage*)buf;

   // copy message

   gmsg.GPSTime = gm->gpstime;
   gmsg.GeoidHeight = gm->geoidheight;

}

void parse_CorrectedIMU_message(uint8_t *buf, polyx_nodea::CorrectedIMU &imsg)
{
   struct correctedImuMessage *im = (struct correctedImuMessage*)buf;

   // copy message
   int i;

   imsg.GpsTimeWeek = im->GPSTimeWeek;
   for (i = 0; i < 3; i++) imsg.Acceleration[i] = im->acc[i];
   for (i = 0; i < 3; i++) imsg.RotationRate[i] = im->rotRate[i];
   imsg.GpsWeekNumber = im->week;
}

// %Tag(CALLBACK)%
void polyxWheelSpeedReportCallback(const polyx_nodea::WheelSpeedReport::ConstPtr& msg)
{

   ROS_INFO(">>> Received a WheelSpeedReport message:\n");

   struct speedmessage sm;
   sm.sync1 = 0xAF;
   sm.sync2 = 0x20;
   sm.msg_type = 0x09;
   sm.sub_id = 0x04;
   sm.payload_len = 15;

   sm.time = msg->Time;
   sm.speed = msg->Speed;
   sm.speed_RMS = msg->Speed_RMS;
   sm.flags = msg->Flags;

   checksum((uint8_t*)&sm.time, 15, sm.chksumA, sm.chksumB);

   if (fd_mon > 0)
   {
      int num;
      num = write_port(fd_mon, &sm, sizeof(sm));
      printf("write wheel speed message to port successfully\n");
   }
}

void polyxStaticHeadingEventCallback(const polyx_nodea::StaticHeadingEvent::ConstPtr& stmsg)
{

   ROS_INFO(">>> Received a StaticHeadingEvent message:\n");

   ROS_INFO("message[%p], Heading=%d", stmsg, stmsg->Heading);
   ROS_INFO("message[%p], ZUPT RMS=%d", stmsg, stmsg->ZUPTRMS);
   ROS_INFO("message[%p], Heading RMS=%d\n", stmsg, stmsg->HeadingRMS);

   struct staticHeadingmessage stm;
   stm.sync1 = 0xAF;
   stm.sync2 = 0x20;
   stm.msg_type = 0x09;
   stm.sub_id = 0x02;
   stm.payload_len = 5;


   stm.heading = stmsg->Heading;
   stm.ZUPT_RMS = stmsg->ZUPTRMS;
   stm.heading_RMS = stmsg->HeadingRMS;

   checksum((uint8_t*)&stm.heading, 5, stm.chksumA, stm.chksumB);

   if (fd_mon > 0)
   {
      int num;
      num = write_port(fd_mon, &stm, sizeof(stm));
      printf("write static heading message to port successfully\n");
   }
}

void polyxStaticGeoPoseEventCallback(const polyx_nodea::StaticGeoPoseEvent::ConstPtr& sgmsg)
{

   ROS_INFO(">>> Received a StaticGeoPoseEvent message:\n");

   struct staticGeoPosemessage sgm;
   sgm.sync1 = 0xAF;
   sgm.sync2 = 0x20;
   sgm.msg_type = 0x09;
   sgm.sub_id = 0x03;
   sgm.payload_len = 32;

   sgm.latitude = sgmsg->Latitude;
   sgm.longitude = sgmsg->Longitude;
   sgm.height = sgmsg->EllipsoidalHeight;
   sgm.roll = sgmsg->Roll;
   sgm.pitch = sgmsg->Pitch;
   sgm.heading = sgmsg->Heading;
   sgm.PositionRMS = sgmsg->PositionRMS;
   sgm.ZUPT_RMS = sgmsg->ZUPTRMS;
   sgm.heading_RMS = sgmsg->HeadingRMS;
   sgm.flags = sgmsg->Flags;

   checksum((uint8_t*)&sgm.latitude, 32, sgm.chksumA, sgm.chksumB);

   if (fd_mon > 0)
   {
      int num;
      num = write_port(fd_mon, &sgm, sizeof(sgm));
      printf("write static geo-pose message to port successfully\n");
   }
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
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
   ros::init(argc, argv, "polyx_talker");
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
   ros::Publisher kalman_pub = n.advertise<polyx_nodea::Kalman>("polyx_Kalman", 2);

   ros::Publisher RawIMU_pub = n.advertise<polyx_nodea::RawIMU>("polyx_rawIMU", 2);

   ros::Publisher SolutionStatus_pub = n.advertise<polyx_nodea::SolutionStatus>("polyx_solutionStatus", 2);

   ros::Publisher icd_pub = n.advertise<polyx_nodea::Icd>("polyx_ICD", 2);
   ros::Publisher geopose_pub = n.advertise<geographic_msgs::GeoPoseStamped>("current_geopose", 2);
   ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("current_pose", 2);
   ros::Publisher twist_pub = n.advertise<geometry_msgs::TwistStamped>("current_velocity", 2);
   ros::Publisher accel_pub = n.advertise<geometry_msgs::AccelStamped>("current_acceleration", 2);
   ros::Publisher navfix_pub = n.advertise<sensor_msgs::NavSatFix>("current_navsatfix", 2);
   ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("current_imu", 2);
   ros::Publisher EulerAttitude_pub = n.advertise<polyx_nodea::EulerAttitude>("polyx_EulerAttitude", 2);

   ros::Publisher timeSync_pub = n.advertise<polyx_nodea::TimeSync>("polyx_timeSync", 2);

   ros::Publisher geoid_pub = n.advertise<polyx_nodea::Geoid>("polyx_Geoid", 2);

   ros::Publisher CorrectedIMU_pub = n.advertise<polyx_nodea::CorrectedIMU>("polyx_correctedIMU", 2);

   struct origin_type myorigin;
   bool is_origin_set = false;

   // %EndTag(PUBLISHER)%

   // %Tag(LOOP_RATE)%
   ros::Rate loop_rate(200);
   // %EndTag(LOOP_RATE)%

     /**
      * A count of how many messages we have sent. This is used to create
      * a unique string for each message.
      */
      // %Tag(ROS_OK)%
   int count = 0;

   polyx_nodea::Kalman kalmsg;
   polyx_nodea::RawIMU imsg;
   polyx_nodea::SolutionStatus smsg;
   polyx_nodea::Icd msg;
   polyx_nodea::EulerAttitude qtemsg;
   polyx_nodea::TimeSync tsmsg;
   polyx_nodea::Geoid gmsg;
   polyx_nodea::CorrectedIMU cimsg;

   int bufpos = 0;
   int msglen = 0;
   uint8_t buf[256];
   uint8_t recvbuf[256];
   int num;

   int state = 0;
   struct timeval tout;
   // default uart read timeout
   tout.tv_sec = 1;
   tout.tv_usec = 100 * 1000;

   // get parameters
   std::string my_port;
   if (n.getParam("/polyx_port", my_port)) {
      ROS_INFO("serial port=%s", my_port.c_str());
   }
   else my_port = MON_PORT;

   int my_baud;
   if (n.getParam("/polyx_baud", my_baud)) {
      ROS_INFO("serial baud=%d", my_baud);
   }
   else my_baud = SERIAL_BAUD;

   int my_output;
   if (n.getParam("/polyx_output", my_output)) {
      ROS_INFO("output msgs=%d", my_output);
   }
   else my_baud = OUT_ICD | OUT_GEOPOSE | OUT_TWIST | OUT_ACCEL | OUT_NAVSATFIX | OUT_IMU;

   std::string my_speedreport;
   if (n.getParam("/polyx_speedreport", my_speedreport)) {
      ROS_INFO("speed report=%s", my_speedreport.c_str());
   }
   else my_speedreport = "polyx_WheelSpeed";

   std::string my_staticheading;
   if (n.getParam("/polyx_staticheading", my_staticheading)) {
      ROS_INFO("static heading=%s", my_staticheading.c_str());
   }
   else my_staticheading = "polyx_StaticHeading";

   std::string my_staticgeopose;
   if (n.getParam("/polyx_staticgeopose", my_staticgeopose)) {
      ROS_INFO("static geopose=%s", my_staticgeopose.c_str());
   }
   else my_staticgeopose = "polyx_StaticGeoPose";

   /*
   * Start the Listner part
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
   // %Tag(SUBSCRIBER)%
   ros::Subscriber wheelSpeed_sub = n.subscribe(my_speedreport, 100, polyxWheelSpeedReportCallback);

   ros::Subscriber staticHeading_sub = n.subscribe("polyx_StaticHeading", 100, polyxStaticHeadingEventCallback);

   ros::Subscriber staticGeoPose_sub = n.subscribe("polyx_StaticGeoPose", 100, polyxStaticGeoPoseEventCallback);
   // %EndTag(SUBSCRIBER)%

   /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */
    // %Tag(SPIN)%
    //  ros::spin();
    // %EndTag(SPIN)%

       /*
       *  Start the talker part to check the serial port for data
       */
   while (ros::ok())
   {
      int new_output;
      // check output msg control changed or not
      if (n.getParam("/polyx_output", new_output)) 
      {
         if (new_output != my_output) {
            ROS_INFO("changed otput msgs=%d", new_output);
            my_output = new_output;
         }
      }
      //

       // %Tag(SPINONCE)%
      ros::spinOnce();
      // %EndTag(SPINONCE)%

      // first check if we need reopen the serial port
      if (fd_mon < 0)
      {
         fd_mon = open_mon(my_port.c_str(), my_baud);
         
         if (fd_mon < 0) 
         {
            perror("open serial port error");
            usleep(200 * 1000);   // delay 200ms
//			   continue;
            goto loop_ros;
         }
      }

      num = read_port(fd_mon, recvbuf, 256, &tout);
      if (num < 0) 
      {
         if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
         {
            perror("read serial port error");
            // depends on the type or error, do we need re-open the serial port?
            close(fd_mon);
            fd_mon = -1;
            goto loop_ros;
         }
      }
      else 
      {
         int i;
         for (i = 0; i < num; i++)
         {
            uint8_t ch = recvbuf[i];
            switch (state)
            {
            case _SYNC1:
               if (ch == ICD_SYNC1) {
                  state++;
                  bufpos = 0;
                  buf[0] = ICD_SYNC1;
                  bufpos++;
               }
               break;
            case _SYNC2:
               if (ch == ICD_SYNC2) {
                  state++;
                  buf[bufpos] = ch;
                  bufpos++;
               }
               else state == 0;
               break;
            case _HEAD:
               if (bufpos < 6) {
                  buf[bufpos++] = ch;
               }
               if (bufpos == 6)
               {
                  msglen = buf[5];
                  msglen = (msglen << 8) + buf[4];

                  if (msglen > MaxMsgLen)
                  {
                     printf("Invalid message length length=%d\n", msglen);
                     state = 0;
                     continue;
                  }
                  else
                     state++;
               }
               break;
            case _MSG:
               if (bufpos < msglen + 8)   // payloadlen + checksum
               {
                  buf[bufpos++] = ch;
               }
               if (bufpos == (msglen + 8))  // got full message
               {
                  state = 0;
                  if (checkMessageType(buf) == ICD_TYPE)
                  {
                     switch (buf[3])
                     {
                     case 1:
                        printf("found message: Type=%02d, SubId=%02d, length=%03d\n", buf[2], buf[3], msglen);
                        parse_Kalman_message(buf, kalmsg);

                        ROS_INFO("%u, %f", count, kalmsg.SystemTime);
                        ROS_INFO("%u, %f", count++, kalmsg.GPSTime);

                        kalman_pub.publish(kalmsg);

                        break;
                     case 8:
                        printf("found message: Type=%02d, SubId=%02d, length=%03d\n", buf[2], buf[3], msglen);
                        parse_RawIMU_message(buf, imsg);
                        ROS_INFO("%u, %f", count++, imsg.SystemTime);
                        RawIMU_pub.publish(imsg);
                        break;
                     case 9:
                        printf("found message: Type=%02d, SubId=%02d, length=%03d\n", buf[2], buf[3], msglen);
                        parse_SolutionStatus_message(buf, smsg);
                        ROS_INFO("%u, %f", count++, smsg.SystemTime);
                        SolutionStatus_pub.publish(smsg);
                        break;

                     case 13:
                        printf("found message: Type=%02d, SubId=%02d, length=%03d\n", buf[2], buf[3], msglen);
                        parse_Icd_message(buf, msg);
                        // %Tag(ROSCONSOLE)%
                        ROS_INFO("%u, %d", count++, msg.GpsWeekNumber);
                        // %EndTag(ROSCONSOLE)%
                        // %Tag(PUBLISH)%
                        if (my_output & OUT_ICD)  icd_pub.publish(msg);
                        if (my_output & OUT_GEOPOSE)
                        {
                           geographic_msgs::GeoPoseStamped pmsg;
                           icd_to_GeoPoseStamped(msg, pmsg);
                           geopose_pub.publish(pmsg);
                        }
                        if (my_output & OUT_TWIST)
                        {
                           geometry_msgs::TwistStamped tmsg;
                           icd_to_TwistStamped(msg, tmsg);
                           twist_pub.publish(tmsg);
                        }
                        if (my_output & OUT_ACCEL)
                        {
                           geometry_msgs::AccelStamped amsg;
                           icd_to_AccelStamped(msg, amsg);
                           accel_pub.publish(amsg);
                        }
                        if (my_output & OUT_NAVSATFIX)
                        {
                           sensor_msgs::NavSatFix nmsg;
                           icd_to_NavSatFix(msg, nmsg);
                           navfix_pub.publish(nmsg);
                        }
                        if (my_output & OUT_IMU)
                        {
                           sensor_msgs::Imu imsg;
                           icd_to_Imu(msg, imsg);
                           imu_pub.publish(imsg);
                        }
                        if (my_output & OUT_EULER_ATT)
                        {
                           if (EulerAttitude(msg, qtemsg)) EulerAttitude_pub.publish(qtemsg);
                        }
                        if (!is_origin_set) {

                           // To set your own fixed origin use SetCustomOrigin() and
                           // comment out SetOrigin().

                           // SetCustomOrigin(lat, lon, alt, myorigin);

                           SetOrigin(msg, myorigin);

                           is_origin_set = true;
                        }

                        if (my_output & OUT_POSE)
                        {
                           geometry_msgs::PoseStamped pmsg;
                           icd_to_PoseStamped(msg, myorigin, pmsg);
                           pose_pub.publish(pmsg);
                        }

                        break;

                     case 16:
                        printf("found message: Type=%02d, SubId=%02d, length=%03d\n", buf[2], buf[3], msglen);
                        parse_TimeSync_message(buf, tsmsg);
                        timeSync_pub.publish(tsmsg);
                        break;

                     case 22:
                        printf("found message: Type=%02d, SubId=%02d, length=%03d\n", buf[2], buf[3], msglen);
                        parse_Geoid_message(buf, gmsg);

                        ROS_INFO("%u, %f", count, gmsg.GPSTime);
                        ROS_INFO("%u, %f", count++, gmsg.GeoidHeight);

                        geoid_pub.publish(gmsg);
                        break;

                     case 23:
                        printf("found message: Type=%02d, SubId=%02d, length=%03d\n", buf[2], buf[3], msglen);
                        parse_CorrectedIMU_message(buf, cimsg);

                        ROS_INFO("%u, %f", count, cimsg.GpsTimeWeek);
                        ROS_INFO("%u, %d", count++, cimsg.GpsWeekNumber);

                        CorrectedIMU_pub.publish(cimsg);
                        break;

                     default:
                        printf("skip packet[(%02X:%02X) type=%02d, subId=%02d, len=%d].\n", buf[0], buf[1], buf[2], buf[3], msglen);
                     }
                  }
               }
               break;
            default:
               state = _SYNC1;
            } // end switch
         } // end for
      }//	

   loop_ros:

      // %Tag(RATE_SLEEP)%
      loop_rate.sleep();
      // %EndTag(RATE_SLEEP)%
   } // while (ros::ok())

   close(fd_mon);
   fd_mon = -1;


   ros::waitForShutdown();

   return 0;
}
// %EndTag(FULLTEXT)%



