
#pragma once

#define OUT_ICD     0x01
#define OUT_POSE    0x02
#define OUT_TWIST   0x04
#define OUT_ACCEL   0x08
#define OUT_NAVSATFIX 0x10
#define OUT_IMU     0x20
#define OUT_GEOPOSE 0x40
#define OUT_EULER_ATT 0x80

#define OUT_ALL     0x7FFFFFFF

typedef enum {
    _SYNC = 0,
    _HEAD,
    _MSG
 } DecodeStatus;
 
struct   __attribute__((packed)) kalmanmessage
{
   unsigned char sync1;
   unsigned char sync2;
   unsigned char msg_type;
   unsigned char sub_id;
   unsigned short int payload_len;

   // --- payload data
   
   double sysTime;            //SystemTime(seconds)
   double GPSTime;            //GPSTime(seconds)
   double lat;                //Latitude(radians)
   double lon;                //Longitude(radians)
   double ellHeight;          //EllipsoidalHeight(m)
   double velNorth;           //VelocityNorth(m/s)
   double velEast;            //VelocityEast(m/s)
   double velDown;            //VelocityDown(m/s)
   double roll;               //Roll(radians)
   double pitch;              //Pitch(radians)
   double heading;            //Heading(radians)
   unsigned char posMode;     //PositionMode 0:Invalid, 1:Dead-reckoning, 2:Stand-alone, 3.Precise point positioning 
                              //4. Code differential, 5.RTK float, 6.RTK fixed
   unsigned char velMode;     //VelocityMode
   unsigned char attStatus;   //AttitudeStatus 0:Invalid, 1:Coarse, 2:Fine

   
// ----- end payload    
   unsigned char chksumA;
   unsigned char chksumB;
};

struct   __attribute__((packed)) rawImuMessage
{
   unsigned char sync1;
   unsigned char sync2;
   unsigned char msg_type;
   unsigned char sub_id;
   unsigned short int payload_len;

   // --- payload data
   double   sysTime;         // system time (seconds)
   double   acc[3];          // acceleration(m/s^2)
   double   rotRate[3];      // rotation rate (deg/s)

// ----- end payload    
   unsigned char chksumA;
   unsigned char chksumB;
};

 struct   __attribute__((packed)) solutionStatusMessage
{
   unsigned char sync1;
   unsigned char sync2;
   unsigned char msg_type;
   unsigned char sub_id;
   unsigned short int payload_len;

   // --- payload data
   double systemTime;             // System time(seconds)
   unsigned char NumberOfSVs;     // Number of SVs used
   unsigned char ProcessingMode;  // processing mode
   unsigned short int week;       // GPS week number
   double   GPSTimeWeek;          // GPS time of week (seconds)
   double   PositionRMS[3];       // Position RMS(NED) m
   double   VelocityRMS[3];       // Velocity RMS(NED) m/s
   double   AttitudeRMS[3];       // Attitude RMS(NED) deg

   // ----- end payload    
   unsigned char chksumA;
   unsigned char chksumB;
};

struct   __attribute__((packed)) icdmessage
{
   unsigned char sync1;
   unsigned char sync2;
   unsigned char msg_type;
   unsigned char sub_id;
   unsigned short int payload_len;

   // --- payload data
   double   tow;               // GPS time of week (seconds)
   double   lat;               // latitude (deg)
   double   lon;               // longitude (deg)
   float    alt;               // altitude/Ellipsoidal height (m)
   float    vel[3];            // Velocity in North/East/Down (m/s)
   float    q_bn[4];           // Body to NED Attitude quaternion, Scalar/X/Y/Z
   float    acc[3];            // Vehicle acceleration in /IMU (m/s^2)
   float    rot_rate[3];       // Body rotation rate (deg/s)
   float    pos_rms[3];        // Position RMS (m)
   float    vel_rms[3];        // Velocity RMS (m/s)
   float    att_rms[3];        // Attitude RMS (deg)
   unsigned short int week;    // GPS week number
   unsigned char  align_mode;  // Alignment mode 0=invalid;1=Coarse;2=Fine
// ----- end payload    
   unsigned char chksumA;
   unsigned char chksumB;
};

struct   __attribute__((packed)) timeSyncmessage
{
   unsigned char sync1;
   unsigned char sync2;
   unsigned char msg_type;
   unsigned char sub_id;
   unsigned short int payload_len;

   // --- payload data
   double   systemComTime;      // System Computer Time (seconds)
   double   biasToGPSTime;      // Bias with respect to GPS time (seconds)

// ----- end payload    
   unsigned char chksumA;
   unsigned char chksumB;
};

struct   __attribute__((packed)) geoidmessage
{
   unsigned char sync1;
   unsigned char sync2;
   unsigned char msg_type;
   unsigned char sub_id;
   unsigned short int payload_len;

   // --- payload data
   double   gpstime;         // GPS time (seconds)
   float    geoidheight;     // Geoid height (m)
// ----- end payload    
   unsigned char chksumA;
   unsigned char chksumB;
};



struct   __attribute__((packed)) correctedImuMessage
{
   unsigned char sync1;
   unsigned char sync2;
   unsigned char msg_type;
   unsigned char sub_id;
   unsigned short int payload_len;

   // --- payload data
   double   GPSTimeWeek;        // GPS time of week (seconds)
   double   acc[3];             // acceleration(m/s^2)
   double   rotRate[3];         // rotation rate (deg/s)
   unsigned short int week;     // GPS week number

// ----- end payload    
   unsigned char chksumA;
   unsigned char chksumB;
};

struct   __attribute__((packed)) leapSecondsmessage
{
   unsigned char sync1;
   unsigned char sync2;
   unsigned char msg_type;
   unsigned char sub_id;
   unsigned short int payload_len;

   // --- payload data
   unsigned char   leapSeconds;      // offset between the GPS time and the UTC (seconds)

// ----- end payload    
   unsigned char chksumA;
   unsigned char chksumB;
};

struct   __attribute__((packed)) speedmessage
{
  unsigned char sync1;             // 0xAF
  unsigned char sync2;             // 0x20
  unsigned char msg_type;          // 0x09
  unsigned char sub_id;            // 0x04
  unsigned short int payload_len;  // 15

  // --- payload data  
  double             time;          //(seconds)
  float              speed;         // m/s
  unsigned short int speed_RMS;     // mm/s (0:Invalid speed)
  unsigned char      flags;         // 0 : Nav System 1 : GPS

// ----- end payload    
  unsigned char chksumA;
  unsigned char chksumB;
};

struct origin_type
{
   double r[3];      // ECEF coordinates of the origin (m)
   double Cen[3][3]; // DCM from ECEF to NED
};

struct   __attribute__((packed)) staticHeadingmessage
{
  unsigned char sync1;              // 0xAF
  unsigned char sync2;              // 0x20
  unsigned char msg_type;           // 0x09
  unsigned char sub_id;             // 0x02
  unsigned short int payload_len;   // 5

  // --- payload data  
  short int          heading;       // 0.01 degrees
  unsigned short int ZUPT_RMS;      // mm/s (0:Invalid)
  unsigned char      heading_RMS;   // 0.1 deg (0:Invalid)

// ----- end payload    
  unsigned char chksumA;
  unsigned char chksumB;
};

struct   __attribute__((packed)) staticGeoPosemessage
{
  unsigned char sync1;                        // 0xAF
  unsigned char sync2;                        // 0x20
  unsigned char msg_type;                     // 0x09
  unsigned char sub_id;                       // 0x03
  unsigned short int payload_len;             // 32

  // --- payload data  
  double             latitude;                //deg
  double             longitude;               //deg
  float              height;                  //m
  short int          roll;                    //0.01 degrees
  short int          pitch;                   //0.01 degrees
  short int          heading;                 //0.01 degrees
  unsigned short int PositionRMS;             //cm(0:Invalid position)
  unsigned short int ZUPT_RMS;                //mm/s(0:Invalid)
  unsigned char      heading_RMS;             //0.1 deg(0:Invalid)
  unsigned char      flags;                   //0x01:Roll&Pitch Valid, 0x02:disable GNSS
  
// ----- end payload    
  unsigned char chksumA;
  unsigned char chksumB;
};
//============================
// MSG definition from Baidu Apollo
//============================
