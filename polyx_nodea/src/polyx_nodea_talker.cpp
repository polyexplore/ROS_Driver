/*
 * Copyright (C) 2017, PolyExplore Inc.
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

#include <math.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>      /* Error number definitions */ 
#include <unistd.h>
#include <string.h>
#include <termios.h>    /* POSIX terminal control definitions */
#include <time.h> 
#include <signal.h>

#include "ros/ros.h"
 // %EndTag(ROS_HEADER)%
 // %Tag(MSG_HEADER)%
#include "polyx_nodea/Kalman.h"
#include "polyx_nodea/RawIMU.h"
#include "polyx_nodea/SolutionStatus.h"
#include "polyx_nodea/Icd.h"
#include "polyx_nodea/EulerAttitude.h"
#include "polyx_nodea/TimeSync.h"
#include "polyx_nodea/Geoid.h"
#include "polyx_nodea/CorrectedIMU.h"
#include "polyx_nodea/LeapSeconds.h"
#include "polyx_nodea/WheelSpeedReport.h"
#include "polyx_nodea/StaticHeadingEvent.h"
#include "polyx_nodea/StaticGeoPoseEvent.h"
#include "polyx_nodea/dmi.h"

 // %EndTag(MSG_HEADER)%
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "polyxdata.h"
#include "polyx_convert.h"
#include "polyx_nmea.h"

#define MON_PORT "/dev/ttyUSB1"
#define ICD_SYNC1 0xAF
#define ICD_SYNC2 0x20
#define ICD_TYPE 5

#define SERIAL_WAIT (1000000/46080)
#define SERIAL_TIMEOUT 10   // seconds
#define SERIAL_BAUD 230400
#define MASK_SIG_UART 1

static bool keepRunning = true;

// global serial port file descriptor
int fd_mon = -1;
#define MaxMsgLen 2048
uint8_t recvbuf[MaxMsgLen];
std::string my_port;
int my_baud;

// Ethernet options
bool eth_enable = false;
int sockfd = -1;

int LEAP_SECONDS = -1;
int INI_GPS_WEEK_NUM = -1;

void intHandler(int) 
{
   keepRunning = false;
   printf("polyx_nodea_talker: Ctrl-C hit, will stop!\r\n");
}

void abortHandler(int) 
{
   keepRunning = false;
   printf("polyx_nodea_talker: Asked to Abort, will stop!\r\n");
}

void segHandler(int) {
   printf("polyx_nodea_talker: Segment Fault\r\n");
}

void ioHandler(int) {
   printf("polyx_nodea_talker: Received SIGIO\r\n");
}

uint8_t checksum(uint8_t* Buffer, uint16_t len, uint8_t& cka, uint8_t& ckb)
{
   unsigned char CK_A = 0;
   unsigned char CK_B = 0;

   if (Buffer)
   {
      for (int i = 0; i < len; i++)
      {
         CK_A += Buffer[i];
         CK_B += CK_A;
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

#ifdef MASK_SIG_UART
   sigset_t mask;
   sigset_t orig_mask;

   sigemptyset(&mask);
   sigaddset(&mask, SIGIO);
   sigaddset(&mask, SIGINT);
   sigaddset(&mask, SIGABRT);

   if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0)
   {
      perror("sigprocmask");
      return -1;
   }
#endif

   num = 0;

   FD_ZERO(&outputs);
   FD_SET(fd, &outputs);

   ret = select(fd + 1, (fd_set *)NULL, &outputs, (fd_set *)NULL, &tout);
   if (ret < 0)
   {
      perror("select error!!");
   }
   else if (ret > 0) 
   {
      if (FD_ISSET(fd, &outputs))
      {
         num = write(fd, buf, len);
      }
   }

#ifdef MASK_SIG_UART
   if (sigprocmask(SIG_SETMASK, &orig_mask, NULL) < 0)
   {

   }
#endif

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

   if (ret < 0)
   {
      perror("select error!!");
   }
   else if (ret > 0)
   {
      if (FD_ISSET(fd, &inputs))
      {
         size_t len = 0;
         int errsav;
         ioctl(fd, FIONREAD, &len);
         errsav = errno;
         if (len == 0)
         {
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
   }
   tcgetattr(fd, &options);                                            // Get the current options of the port
   bzero(&options, sizeof(options));                                   // Clear all the options
   speed_t         Speed;

   switch (baud)                                                       // Set the speed (Bauds)
   {
   case 110:     Speed = B110; break;
   case 300:     Speed = B300; break;
   case 600:     Speed = B600; break;
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
   default:  Speed = baud;
   }
   cfsetispeed(&options, Speed);                                       // Set the baud rate at 115200 bauds
   cfsetospeed(&options, Speed);

   options.c_cflag |= (CLOCAL | CREAD | CS8);                          // Configure the device : 8 bits, no parity, no control
   options.c_iflag |= (IGNPAR | IGNBRK);
   options.c_cc[VTIME] = 0;                                            // Timer unused
   options.c_cc[VMIN] = 0;                                             // At least on character before satisfy reading

   options.c_cflag |= CREAD | CLOCAL;     							        // turn on READ & ignore ctrl lines
   options.c_iflag &= ~(IXON | IXOFF | IXANY);                         // turn off s/w flow ctrl
   options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);                 // make raw
   options.c_oflag &= ~OPOST;                                          // make raw

   tcsetattr(fd, TCSANOW, &options);                                   // Activate the settings

   return fd;                                                          // successful
}

int connectEthernet(char* str_server, char* str_port)
{
   int portno, n1;
   int res;
   long arg;
   fd_set myset;
   struct timeval tv;
   int valopt;
   socklen_t lon;

   struct sockaddr_in serv_addr;
   struct hostent *server;

   sockfd = socket(AF_INET, SOCK_STREAM, 0);
   //printf("sockfd is %d\n", sockfd);
   if (sockfd < 0)
   {
      printf("ERROR opening socket.\n");
      return -1;
   }

   // Set non-blocking 
   arg = fcntl(sockfd, F_GETFL, NULL);
   arg |= O_NONBLOCK;
   fcntl(sockfd, F_SETFL, arg);

   server = gethostbyname(str_server);
   bzero((char *)&serv_addr, sizeof(serv_addr));
   serv_addr.sin_family = AF_INET;

   portno = atoi(str_port);
   serv_addr.sin_port = htons(portno);

   bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);

   res = connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
   if (res < 0)
   {
      if (errno == EINPROGRESS) 
      {
         tv.tv_sec = 10;
         tv.tv_usec = 0;
         FD_ZERO(&myset);
         FD_SET(sockfd, &myset);
         if (select(sockfd + 1, NULL, &myset, NULL, &tv) > 0) 
         {
            lon = sizeof(int);
            getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (void*)(&valopt), &lon);
            if (valopt) 
            {
               fprintf(stderr, "Error in connection() %d - %s\n", valopt, strerror(valopt));
               return -2;
            }
         }
         else 
         {
            fprintf(stderr, "Timeout or error() %d - %s\n", valopt, strerror(valopt));
            return -2;
         }
      }
      else 
      {
         fprintf(stderr, "Error connecting %d - %s\n", errno, strerror(errno));
         return -2;
      }
   }
   // Set to blocking mode again... 
   // arg = fcntl(sockfd, F_GETFL, NULL);
   // arg &= (~O_NONBLOCK);
   // fcntl(sockfd, F_SETFL, arg);

   return 0;
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

   int i;

   imsg.SystemTime = im->sysTime;
   for (i = 0; i < 3; i++) imsg.Acceleration[i] = im->acc[i];
   for (i = 0; i < 3; i++) imsg.RotationRate[i] = im->rotRate[i];

}

void parse_SolutionStatus_message(uint8_t *buf, polyx_nodea::SolutionStatus &smsg)
{
   struct solutionStatusMessage *im = (struct solutionStatusMessage*)buf;

   int i;

   smsg.SystemTime = im->systemTime;
   smsg.GpsWeekNumber = im->week;
   smsg.NumberOfSVs = im->NumberOfSVs;
   smsg.ProcessingMode = im->ProcessingMode;
   smsg.GpsTimeWeek = im->GPSTimeWeek;
   for (i = 0; i < 3; i++) smsg.PositionRMS[i] = im->PositionRMS[i];
   for (i = 0; i < 3; i++) smsg.VelocityRMS[i] = im->VelocityRMS[i];
   for (i = 0; i < 3; i++) smsg.AttitudeRMS[i] = im->AttitudeRMS[i];
   
   // Initial GPS week number
   if (INI_GPS_WEEK_NUM < 0 && smsg.GpsWeekNumber > 0)
      INI_GPS_WEEK_NUM = smsg.GpsWeekNumber;

}

void parse_Icd_message(
   uint8_t*          buf, 
   polyx_nodea::Icd& msg,
   ref_frame_type    frame)
{
   struct icdmessage *im = (struct icdmessage*)buf;

   int i;

   msg.GpsTimeWeek = im->tow;
   msg.Latitude = im->lat*DEG_TO_RAD;
   msg.Longitude = im->lon*DEG_TO_RAD;
   msg.Altitude = im->alt;

   if (frame == NAD83)
      ConvertToNAD83(im->week, im->tow, msg.Latitude, msg.Longitude, msg.Altitude);

   for (i = 0; i < 3; i++) msg.VelocityNED[i] = im->vel[i];
   for (i = 0; i < 4; i++) msg.Quaternion[i] = im->q_bn[i];
   for (i = 0; i < 3; i++) msg.Acceleration[i] = im->acc[i];
   for (i = 0; i < 3; i++) msg.RotationRate[i] = im->rot_rate[i] * DEG_TO_RAD;
   for (i = 0; i < 3; i++) msg.PositionRMS[i] = im->pos_rms[i];
   for (i = 0; i < 3; i++) msg.VelocityRMS[i] = im->vel_rms[i];
   for (i = 0; i < 3; i++) msg.AttitudeRMS[i] = im->att_rms[i] * DEG_TO_RAD;
   msg.GpsWeekNumber = im->week;
   msg.Alignment = im->align_mode;

   if (msg.GpsWeekNumber > 0 && LEAP_SECONDS > 0)
   {
      GpsToEpoch(msg.GpsWeekNumber, msg.GpsTimeWeek - LEAP_SECONDS, msg.header.stamp);
	  if (INI_GPS_WEEK_NUM < 0)
	     INI_GPS_WEEK_NUM = msg.GpsWeekNumber;
   }
   else
      msg.header.stamp = ros::Time::now();
}

void parse_TimeSync_message(uint8_t *buf, polyx_nodea::TimeSync &tsmsg)
{
   struct timeSyncmessage *tsm = (struct timeSyncmessage*)buf;

   tsmsg.header.stamp = ros::Time::now();
   tsmsg.SystemComputerTime = tsm->systemComTime;
   tsmsg.BiasToGPSTime = tsm->biasToGPSTime;

}

void parse_Geoid_message(uint8_t *buf, polyx_nodea::Geoid &gmsg)
{
   struct geoidmessage *gm = (struct geoidmessage*)buf;

   if (gm == NULL) 
   {
      printf("Geoid message is null!\n");
      return;
   }

   gmsg.GPSTime = gm->gpstime;
   gmsg.GeoidHeight = gm->geoidheight;

}

void parse_CorrectedIMU_message(uint8_t *buf, polyx_nodea::CorrectedIMU &imsg)
{
   struct correctedImuMessage *im = (struct correctedImuMessage*)buf;

   int i;

   imsg.GpsTimeWeek = im->GPSTimeWeek;
   for (i = 0; i < 3; i++) imsg.Acceleration[i] = im->acc[i];
   for (i = 0; i < 3; i++) imsg.RotationRate[i] = im->rotRate[i];
   imsg.GpsWeekNumber = im->week;
}

void parse_LeapSeconds_message(uint8_t *buf, polyx_nodea::LeapSeconds &lsmsg)
{
   struct leapSecondsmessage *lsm = (struct leapSecondsmessage*)buf;

   lsmsg.LeapSeconds = lsm->leapSeconds;
}

void parse_dmi_message(
   uint8_t *buf,
   polyx_nodea::TimeSync& ts,   
   polyx_nodea::dmi &dmi)
{
   Decode(&buf[6], dmi.system_time);
   Decode(&buf[14], dmi.pulse_count);
   dmi.id = buf[18];

   if (ts.SystemComputerTime > 0 && INI_GPS_WEEK_NUM > 0 && LEAP_SECONDS > 0)
   {
	   float64 t_gps = dmi.system_time - ts.BiasToGPSTime - LEAP_SECONDS;
	   GpsToEpoch(INI_GPS_WEEK_NUM, t_gps, dmi.header.stamp);
   }
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

   if (eth_enable)
   {
      
      if (sockfd > 0)
      {
         int num;
         num = write(sockfd, &sm, sizeof(sm));
         printf("Wrote wheel speed message to Ethernet port successfully\n");
      }
   }
   else 
   {
      if (fd_mon > 0)
      {
         int num;
         num = write_port(fd_mon, &sm, sizeof(sm));
         printf("Wrote wheel speed message to port successfully\n");
      }
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

   if (eth_enable)
   {
      if (sockfd > 0)
      {
         int num;
         num = write(sockfd, &stm, sizeof(stm));
         printf("write static heading message to Ethernet port successfully\n");
      }
   }
   else 
   {
      if (fd_mon > 0)
      {
         int num; 
         num = write_port(fd_mon, &stm, sizeof(stm));
         printf("write static heading message to serial port successfully\n");
      }
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

   if (eth_enable)
   {
      if (sockfd > 0)
      {
         int num;
         num = write(sockfd, &sgm, sizeof(sgm));
         printf("write static geo-pose message to Ethernet port successfully\n");
      }
   }
   else 
   {
      if (fd_mon > 0)
      {
         int num;
         num = write_port(fd_mon, &sgm, sizeof(sgm));
         printf("write static geo-pose message to serial port successfully\n");
      }
   }
}

int read_serail(void)
{
   int num = 0;
   struct timeval tout;
   // default uart read timeout
   tout.tv_sec = 1;
   tout.tv_usec = 100 * 1000;

#ifdef MASK_SIG_UART
   sigset_t mask;
   sigset_t orig_mask;

   sigemptyset(&mask);
   sigaddset(&mask, SIGIO);
   sigaddset(&mask, SIGINT);
   sigaddset(&mask, SIGABRT);

   if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0) 
   {
      perror("sigprocmask");
      return -1;
   }
#endif

   if (fd_mon < 0)
   {
      fd_mon = open_mon(my_port.c_str(), my_baud);
      if (fd_mon < 0) 
      {
         perror("open serial port error");
         usleep(200 * 1000);   // delay 200ms
      }
   }
   else {
      num = read_port(fd_mon, recvbuf, MaxMsgLen, &tout);
      if (num < 0) 
      {
         if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
         {
            perror("read serial port error");
            // depends on the type or error, do we need re-open the serial port?
            close(fd_mon);
            fd_mon = -1;
         }
      }
   }

#ifdef MASK_SIG_UART   
   if (sigprocmask(SIG_SETMASK, &orig_mask, NULL) < 0)
   {

   }
#endif

   return num;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

   struct sigaction int_act;
   int_act.sa_handler = intHandler;
   sigaction(SIGINT, &int_act, NULL);

   struct sigaction abort_act;
   abort_act.sa_handler = abortHandler;
   sigaction(SIGABRT, &abort_act, NULL);

   struct sigaction seg_act;
   seg_act.sa_handler = segHandler;
   sigaction(SIGILL, &seg_act, NULL);

   struct sigaction io_act;
   io_act.sa_handler = ioHandler;
   sigaction(SIGIO, &io_act, NULL);

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
   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
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
   ros::Publisher leapSeconds_pub = n.advertise<polyx_nodea::LeapSeconds>("polyx_leapSeconds", 2);
   ros::Publisher nmeaGGA_pub = n.advertise<polyx_nodea::nmeaGGA>("polyx_nmeaGGA", 2);
   ros::Publisher dmi_pub = n.advertise<polyx_nodea::dmi>("polyx_dmi", 2);

   struct origin_type myorigin;
   bool is_origin_set = false;
   ref_frame_type msg13_frame = ITRF08; // Change to NAD83 if requred
   //ref_frame_type msg13_frame = NAD83; 
   
   // %EndTag(PUBLISHER)%

   // %Tag(LOOP_RATE)%
   ros::Rate loop_rate(200);
   // %EndTag(LOOP_RATE)%

   if (argc == 4)
   {
      if (strcmp(argv[1], "-e") == 0)
      {
         printf("Ethernet Enabled.\n");
         eth_enable = true;
         if (connectEthernet(argv[2], argv[3]) < 0)
            return -1;
         
      }
      else
      {
         printf("You entered a wrong option.\n");
         return -1;
      }
   }
   else if (argc != 1)
   {
      printf("You entered a wrong command.\n");
      return -1;
   }
   
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
   polyx_nodea::TimeSync tsmsg = {0};
   polyx_nodea::Geoid gmsg;
   polyx_nodea::CorrectedIMU cimsg;
   polyx_nodea::LeapSeconds lsmsg;
   polyx_nodea::dmi dmi_msg;

   int bufpos = 0;
   int msglen = 0;
   uint8_t buf[MaxMsgLen];
   int num;
   int state = 0;

   // get parameters
   if (n.getParam("/polyx_port", my_port))
   {
      ROS_INFO("serial port=%s", my_port.c_str());
   }
   else my_port = MON_PORT;

   if (n.getParam("/polyx_baud", my_baud))
   {
      ROS_INFO("serial baud=%d", my_baud);
   }
   else my_baud = SERIAL_BAUD;

   int my_output;
   if (n.getParam("/polyx_output", my_output))
   {
      ROS_INFO("output msgs=%d", my_output);
   }
   else my_output = OUT_ICD | OUT_GEOPOSE | OUT_TWIST | OUT_ACCEL | OUT_NAVSATFIX | OUT_IMU | OUT_EULER_ATT | OUT_POSE;

   std::string my_speedreport;
   if (n.getParam("/polyx_speedreport", my_speedreport))
   {
      ROS_INFO("speed report=%s", my_speedreport.c_str());
   }
   else my_speedreport = "polyx_WheelSpeed";

   std::string my_staticheading;
   if (n.getParam("/polyx_staticheading", my_staticheading))
   {
      ROS_INFO("static heading=%s", my_staticheading.c_str());
   }
   else my_staticheading = "polyx_StaticHeading";

   std::string my_staticgeopose;
   if (n.getParam("/polyx_staticgeopose", my_staticgeopose))
   {
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

      if (!keepRunning) break;

      // check output msg control changed or not
      if (n.getParam("/polyx_output", new_output))
      {
         if (new_output != my_output)
         {
            ROS_INFO("changed otput msgs=%d", new_output);
            my_output = new_output;
         }
      }
      //

       // %Tag(SPINONCE)%
      ros::spinOnce();
      // %EndTag(SPINONCE)%

      // first check if we need reopen the serial port
      if (eth_enable)
      {
         if (sockfd > 0)
         {
         	//printf("Ethernet Connected\n");
            num = read(sockfd, recvbuf, MaxMsgLen);
            //printf("Write socket message to buff\n");
         }
      }
      else
      {
         num = read_serail();
      }
	  
      if (num > 0)
      {
         int i;
         for (i = 0; i < num; i++)
         {
            uint8_t ch = recvbuf[i];
            switch (state)
            {
            case _SYNC:
               buf[0] = buf[1];
               buf[1] = ch;

               if ((buf[0] == ICD_SYNC1 && buf[1] == ICD_SYNC2)
                  || (buf[0] == '$' && buf[1] == 'G'))
               {
                  state = _HEAD;
                  bufpos = 2;
               }

               break;

            case _HEAD:

               buf[bufpos++] = ch;

               if (buf[0] == '$')
               {
                  if (ch < 32 || 126 < ch || bufpos >= MaxMsgLen)
                  {
                     state = 0;
                  }
                  else if (ch == '*')
                  {
                     state = _MSG;
                     msglen = bufpos + 2;
                  }
               }
               else if (bufpos == 6)
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

               buf[bufpos++] = ch;

               if (buf[0] == '$')
               {
                  if (bufpos >= msglen)
                  {
                     state = _SYNC;
                     buf[bufpos] = '\0';

                     if (nmeaChecksum((char*)buf))
                     {
                        if (strncmp((char*)&buf[3], "GGA,", 4) == 0)
                        {
                           polyx_nodea::nmeaGGA gga;

                           parseNmeaGga((char*)buf, gga);
                           gga.latitude *= DEG_TO_RAD;
                           gga.longitude *= DEG_TO_RAD;
                           
                           nmeaGGA_pub.publish(gga);
                        }
                     }
                     else
                        printf("NMEA chsecksum failure.\n");

                  }
               }
               else if (bufpos == (msglen + 8))  // got full message
               {
                  state = 0;
                  if (checkMessageType(buf) == ICD_TYPE)
                  {
                     switch (buf[3])
                     {
                     case 1:
                        parse_Kalman_message(buf, kalmsg);
                        kalman_pub.publish(kalmsg);

                        break;
                     case 8:
                        parse_RawIMU_message(buf, imsg);
                        RawIMU_pub.publish(imsg);
                        break;
                     case 9:
                        parse_SolutionStatus_message(buf, smsg);
                        SolutionStatus_pub.publish(smsg);
                        break;
						
                     case 12: // DMI message
                        parse_dmi_message(buf, tsmsg, dmi_msg);
                        dmi_pub.publish(dmi_msg);
                        break;

                     case 13:
                        //ROS_INFO("found message: Type=%02d, SubId=%02d, length=%03d\n", buf[2], buf[3], msglen);
                        parse_Icd_message(buf, msg, msg13_frame);
                        //ROS_INFO("%u, %f", count++, msg.GpsTimeWeek);
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
                           if (EulerAttitude(msg, qtemsg))
                           {
                              EulerAttitude_pub.publish(qtemsg);
                           }
                        }
                        if (!is_origin_set) {
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
                        parse_TimeSync_message(buf, tsmsg);
                        timeSync_pub.publish(tsmsg);
                        break;

                     case 22:
                        parse_Geoid_message(buf, gmsg);
                        geoid_pub.publish(gmsg);
                        break;

                     case 23:
                        parse_CorrectedIMU_message(buf, cimsg);
                        CorrectedIMU_pub.publish(cimsg);
                        break;
						
					 case 24:
						parse_LeapSeconds_message(buf, lsmsg);
						LEAP_SECONDS = lsmsg.LeapSeconds;
						leapSeconds_pub.publish(lsmsg);
						break;

                     default:
                        break;
                     } //end switch (buf[3])
                  } //end if checkMessageType
               } //end if buffpos
               if (bufpos >= MaxMsgLen)
               {
                  printf("message overflow length length=%d\n", msglen);
                  state = 0;
                  continue;
               }

               break;
            default:
               state = _SYNC;
            } // end switch(state)
         } // end for
      } //end if (num > 0)

   loop_ros:
      // %Tag(RATE_SLEEP)%
      loop_rate.sleep();
      // %EndTag(RATE_SLEEP)%
   }

   close(fd_mon);
   fd_mon = -1;

   ros::waitForShutdown();

   return 0;
}
// %EndTag(FULLTEXT)%
