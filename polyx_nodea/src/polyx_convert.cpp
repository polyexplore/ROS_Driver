
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
//#include <Quaternion.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include "polyxdata.h"
#include "polyx_nodea/Icd.h"
#include "polyx_nodea/EulerAttitude.h"

#define RAD_TO_DEG (57.295779513082323)
#define WGS84_A    (6378137.0)
#define WGS84_E2   (6.69437999014e-3)
#define GPS_TIME_BEG 315964800.0
#define SEC_PER_WEEK 604800.0

//-----------------------------------------------------------------------------
void  GpsToEpoch(int gps_week, double gps_tow, ros::Time &tm)
{
   double t;
   
   if (gps_week > 0)
      t = GPS_TIME_BEG + gps_week * SEC_PER_WEEK + gps_tow;
   else
      t = gps_tow;

   tm.sec = floor(t);
   tm.nsec = floor((t - tm.sec)*1.0e+9);
}

//-----------------------------------------------------------------------------
void EpochToGps(
   const ros::Time& tm, 
   int&             gps_week, 
   double&          gps_tow)
{
   double t = tm.sec - GPS_TIME_BEG + tm.nsec * 1.0e-9;

   gps_week = floor(t / SEC_PER_WEEK);
   gps_tow = t - (gps_week * SEC_PER_WEEK);
}

//-----------------------------------------------------------------------------
// 3x3 Covariance
template<typename T, typename U> void AssignDiagCov3(const T& rms, U& cov)
{
   // Diagonals
   cov[0] = rms[0] * rms[0];
   cov[4] = rms[1] * rms[1];
   cov[8] = rms[2] * rms[2];

   // Off diagonals

   cov[1] = cov[2] = cov[3] = cov[5] = cov[6] = cov[7] = 0;

}

//-----------------------------------------------------------------------------
// 6x6 Covariance: to be used for pose and twist
template<typename T, typename U> void AssignDiagCov6(const T& rms1, const T& rms2, U& cov)
{
   // initialize to zero
   for (int i = 0; i < 36; ++i)
      cov[i] = 0;

   // Diagonals
   cov[0] = rms1[0] * rms1[0];
   cov[7] = rms1[1] * rms1[1];
   cov[14] = rms1[2] * rms1[2];

   cov[21] = rms2[0] * rms2[0];
   cov[28] = rms2[1] * rms2[1];
   cov[35] = rms2[2] * rms2[2];


}

//-----------------------------------------------------------------------------
// Quaternion product: q3 = q1 * q2
void QuatProd(
   const geometry_msgs::Quaternion& q1,
   const geometry_msgs::Quaternion& q2,
   geometry_msgs::Quaternion&       q3)
{
   q3.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
   q3.y = q1.w*q2.y + q1.y*q2.w + q1.z*q2.x - q1.x*q2.z;
   q3.z = q1.w*q2.z + q1.z*q2.w + q1.x*q2.y - q1.y*q2.x;
   q3.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
}

//-----------------------------------------------------------------------------
// Quaternion for the transformation from NED to ENU
#ifndef M_SQRT1_2
#define M_SQRT1_2 (0.70710678118654746)
#endif
void QuatNED2ENU(geometry_msgs::Quaternion& q)
{
   q.x = M_SQRT1_2;
   q.y = M_SQRT1_2;
   q.z = 0;
   q.w = 0;
}

//-----------------------------------------------------------------------------
// Convert PE ICD message to NavSatFix message
void icd_to_NavSatFix(
   polyx_nodea::Icd&       msg,
   sensor_msgs::NavSatFix& nsf)
{
   nsf.header.stamp = msg.header.stamp;

   nsf.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
   nsf.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS |
      sensor_msgs::NavSatStatus::SERVICE_GLONASS;

   nsf.latitude = msg.Latitude;
   nsf.longitude = msg.Longitude;
   nsf.altitude = msg.Altitude;

   nsf.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

   // Position RMS assignment

   AssignDiagCov3(msg.PositionRMS, nsf.position_covariance);

}

//-----------------------------------------------------------------------------
// Convert PE ICD message to IMU message
void icd_to_Imu(polyx_nodea::Icd &msg, sensor_msgs::Imu &imu)
{
   imu.header.stamp = msg.header.stamp;

   // Quaternion from body to NED
   imu.orientation.x = msg.Quaternion[1];
   imu.orientation.y = msg.Quaternion[2];
   imu.orientation.z = msg.Quaternion[3];
   imu.orientation.w = msg.Quaternion[0];

   // Rotation rate in body frame
   imu.angular_velocity.x = msg.RotationRate[0];
   imu.angular_velocity.y = msg.RotationRate[1];
   imu.angular_velocity.z = msg.RotationRate[2];

   AssignDiagCov3(msg.AttitudeRMS, imu.orientation_covariance);

   imu.linear_acceleration.x = msg.Acceleration[0]; // forward
   imu.linear_acceleration.y = msg.Acceleration[1]; // right
   imu.linear_acceleration.z = msg.Acceleration[2]; // down

  // Unknown acceleration covariance so set to zero

}

//-----------------------------------------------------------------------------
// Convert PE ICD message to GeoPoseStamped message
void icd_to_GeoPoseStamped(
   polyx_nodea::Icd&                msg,
   geographic_msgs::GeoPoseStamped& ps)
{
   geometry_msgs::Quaternion q1;
   geometry_msgs::Quaternion q2;

   QuatNED2ENU(q1);

   q2.x = msg.Quaternion[1];
   q2.y = msg.Quaternion[2];
   q2.z = msg.Quaternion[3];
   q2.w = msg.Quaternion[0];

   ps.header.stamp = msg.header.stamp;

   ps.pose.position.latitude = msg.Latitude * RAD_TO_DEG;   // Latitude (deg)
   ps.pose.position.longitude = msg.Longitude * RAD_TO_DEG; // Longitude (deg)
   ps.pose.position.altitude = msg.Altitude;  // Ellipsoidal height (m)

   // Get quaternion from the body to ENU frame

   QuatProd(q1, q2, ps.pose.orientation);

}

//-----------------------------------------------------------------------------
// Convert PE ICD message to TwistStamped message
void icd_to_TwistStamped(
   polyx_nodea::Icd&            msg,
   geometry_msgs::TwistStamped& ts)
{
   int i;

   ts.header.stamp = msg.header.stamp;

   // Velocity and angular in NED

   ts.twist.linear.x = msg.VelocityNED[0];
   ts.twist.linear.y = msg.VelocityNED[1];
   ts.twist.linear.z = msg.VelocityNED[2];

   ts.twist.angular.x = msg.RotationRate[0];
   ts.twist.angular.y = msg.RotationRate[1];
   ts.twist.angular.z = msg.RotationRate[2];

}

//-----------------------------------------------------------------------------
// Convert PE ICD message to AccelStamped message
void icd_to_AccelStamped(
   polyx_nodea::Icd&            msg,
   geometry_msgs::AccelStamped& as)
{
   int i;
   as.header.stamp = msg.header.stamp;

   as.accel.linear.x = msg.Acceleration[0];
   as.accel.linear.y = msg.Acceleration[1];
   as.accel.linear.z = msg.Acceleration[2];

   // Angular acceleration is not known
}
//-----------------------------------------------------------------------------
void GeodeticToECEF(
   const double& lat,
   const double& lon,
   const double& alt,
   double        r[])
{
   double clat = cos(lat);
   double clon = cos(lon);
   double slat = sin(lat);
   double slon = sin(lon);
   double Rn = WGS84_A / sqrt(1.0 - WGS84_E2 * slat * slat);
   double R = Rn + alt;

   r[0] = R * clat * clon;
   r[1] = R * clat * slon;
   r[2] = (Rn * (1.0 - WGS84_E2) + alt) * slat;
}

//-----------------------------------------------------------------------------
void DCM_ECEFToNED(const double& lat, const double& lon, double Cen[3][3])
{
   double clat = cos(lat);
   double slat = sin(lat);
   double clon = cos(lon);
   double slon = sin(lon);

   Cen[0][0] = -slat * clon; Cen[0][1] = -slat * slon; Cen[0][2] = clat;
   Cen[1][0] = -slon;        Cen[1][1] = clon;        Cen[1][2] = 0.0;
   Cen[2][0] = -clat * clon; Cen[2][1] = -clat * slon; Cen[2][2] = -slat;
}

//-----------------------------------------------------------------------------
// Set origin of pose specifically for customers
void SetCustomOrigin(
   double               latitude,   // radian
   double               longitude,  // radian
   double               altitude,   // meters
   struct origin_type&  org)
{
   GeodeticToECEF(latitude, longitude, altitude, org.r);
   DCM_ECEFToNED(latitude, longitude, org.Cen);
}

//-----------------------------------------------------------------------------
// Set origin for Pose message
void SetOrigin(
   const polyx_nodea::Icd&     msg,
   struct origin_type&         org)
{
   SetCustomOrigin(msg.Latitude, msg.Longitude, msg.Altitude, org);
}

//-----------------------------------------------------------------------------
// Convert PE ICD message to PoseStamped message
void icd_to_PoseStamped(
   const polyx_nodea::Icd&     msg,
   const struct origin_type&   org,
   geometry_msgs::PoseStamped& ps)
{
   double dr_e[3], dr_n[3];
   geometry_msgs::Quaternion q1;
   geometry_msgs::Quaternion q2;
   ps.header.stamp = msg.header.stamp;

   GeodeticToECEF(msg.Latitude, msg.Longitude, msg.Altitude, dr_e);

   for (int i = 0; i < 3; ++i)
      dr_e[i] -= org.r[i];

   for (int i = 0; i < 3; ++i)
   {
      dr_n[i] = 0;
      for (int j = 0; j < 3; ++j)
         dr_n[i] += org.Cen[i][j] * dr_e[j];
   }

   ps.pose.position.x = dr_n[0]; // North
   ps.pose.position.y = dr_n[1]; // East 
   ps.pose.position.z = dr_n[2]; // Down

   // Get quaternion from the body to NED frame

   ps.pose.orientation.x = msg.Quaternion[1];
   ps.pose.orientation.y = msg.Quaternion[2];
   ps.pose.orientation.z = msg.Quaternion[3];
   ps.pose.orientation.w = msg.Quaternion[0];
}


// Convert quaternion to Euler angles
bool EulerAttitude(polyx_nodea::Icd &msg, polyx_nodea::EulerAttitude &qtemsg)
{

   float q0 = msg.Quaternion[0] * msg.Quaternion[0];
   float q1 = msg.Quaternion[1] * msg.Quaternion[1];
   float q2 = msg.Quaternion[2] * msg.Quaternion[2];
   float q3 = msg.Quaternion[3] * msg.Quaternion[3];


   float C31 = 2.0f * (msg.Quaternion[1] * msg.Quaternion[3] - msg.Quaternion[0] * msg.Quaternion[2]);
   float C32 = 2.0f * (msg.Quaternion[2] * msg.Quaternion[3] + msg.Quaternion[0] * msg.Quaternion[1]);
   float C33 = q0 - q1 - q2 + q3;

   qtemsg.pitch = atan(-C31 / sqrt(C32 * C32 + C33 * C33));

   if (fabsf(C31) < 0.999f)
   {
      float C11 = q0 + q1 - q2 - q3;
      float C21 = 2.0f * (msg.Quaternion[1] * msg.Quaternion[2] + msg.Quaternion[0] * msg.Quaternion[3]);

      qtemsg.roll = atan2(C32, C33);
      qtemsg.heading = atan2(C21, C11);

      return true;
   }
   else
   {
      return false;
   }
}
