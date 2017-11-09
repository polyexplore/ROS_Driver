
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
//#include <Quaternion.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include "polyxdata.h"

#define RAD_TO_DEG (57.295779513082323)
#define WGS84_A    (6378137.0)
#define WGS84_E2   (6.69437999014e-3)

//-----------------------------------------------------------------------------
// GPS time to UNIX epoch convertion
//     315964800 + week * 604800 + GPS Time of week
//     where 315964800 is the Unix Epoch of GPS start time (Jan 6, 1980), 
//
void  GpsToEpoch(int gpsweek, double gpstow, ros::Time &tm)
{
       double t = 315964800.0 + gpsweek * 604800.0 + gpstow;
       tm.sec = floor(t);
       tm.nsec = floor((t- tm.sec)*1000000000.0);
       return;
}

//-----------------------------------------------------------------------------
// UNIX epoch to GPS time conversion
void EpochToGps(const ros::Time &tm, int& gpsweek, double& gpstow)
{
    double t = tm.sec - 315964800.0 + tm.nsec/1000000000.0;
    gpsweek = floor(t/604800.0);
    gpstow = t - (gpsweek*604800.0);
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
	for (int i=0; i<36; ++i)
		cov[i] = 0;
	
	// Diagonals
	cov[0]  = rms1[0] * rms1[0];
	cov[7]  = rms1[1] * rms1[1];
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
   geometry_msgs::Quaternion&  q3)
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
void icd_to_NavSatFix( polyx_nodea::Icd &msg, sensor_msgs::NavSatFix &nsf )
{
    nsf.header.stamp = msg.header.stamp;
    
    nsf.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    nsf.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS |
                         sensor_msgs::NavSatStatus::SERVICE_GLONASS ;
    
    nsf.latitude = msg.Latitude;
    nsf.longitude = msg.Longitude;
    nsf.altitude = msg.Altitude;

    nsf.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
	
	// Position RMS assignment
	
	AssignDiagCov3(msg.PositionRMS, nsf.position_covariance);
    
    return;
}

//-----------------------------------------------------------------------------
// Convert PE ICD message to IMU message
void icd_to_Imu( polyx_nodea::Icd &msg, sensor_msgs::Imu &imu )
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
     
    return;
}

//-----------------------------------------------------------------------------
// Convert PE ICD message to GeoPoseStamped message
void icd_to_GeoPoseStamped(polyx_nodea::Icd &msg, geographic_msgs::GeoPoseStamped &ps)
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

   return;
}

//-----------------------------------------------------------------------------
// Convert PE ICD message to TwistStamped message
void icd_to_TwistStamped( polyx_nodea::Icd &msg, geometry_msgs::TwistStamped &ts)
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
    
    return;
}

//-----------------------------------------------------------------------------
// Convert PE ICD message to AccelStamped message
void icd_to_AccelStamped( polyx_nodea::Icd &msg, geometry_msgs::AccelStamped &as)
{
    int i;
    as.header.stamp = msg.header.stamp;

    as.accel.linear.x = msg.Acceleration[0];
    as.accel.linear.y = msg.Acceleration[1];
    as.accel.linear.z = msg.Acceleration[2];

	// Angular acceleration is not known
    
    return;
}

//-----------------------------------------------------------------------------
// Set origin for Pose message
void SetOrigin(
   const polyx_nodea::Icd&     msg,
   struct origin_type&         org)
{
	double slat2 = sin(msg.Latitude);
	double tmp1, tmp2;
	
	slat2 *= slat2;
	tmp1 = 1.0 - WGS84_E2 * slat2;
	tmp2 = sqrt(tmp1);
	
	org.lat = msg.Latitude;
	org.lon = msg.Longitude;
	org.alt = msg.Altitude;
	
	org.Rn = msg.Altitude + WGS84_A*(1.0-WGS84_E2)/(tmp1 * tmp2);
	org.Re = (msg.Altitude + WGS84_A/tmp2) * cos(msg.Latitude);
	   
}

//-----------------------------------------------------------------------------
// Convert PE ICD message to PoseStamped message
void icd_to_PoseStamped(
   const polyx_nodea::Icd&     msg, 
   const struct origin_type&   org,
   geometry_msgs::PoseStamped& ps)
{
   geometry_msgs::Quaternion q1;
   geometry_msgs::Quaternion q2;
   ps.header.stamp = msg.header.stamp;

   ps.pose.position.x = (msg.Latitude - org.lat) * org.Rn; // North
   ps.pose.position.y = (msg.Longitude - org.lon) * org.Re; // East 
   ps.pose.position.z = org.alt - msg.Altitude;            // Down

   // Get quaternion from the body to NED frame
   
   ps.pose.orientation.x = msg.Quaternion[1];
   ps.pose.orientation.y = msg.Quaternion[2];
   ps.pose.orientation.z = msg.Quaternion[3];
   ps.pose.orientation.w = msg.Quaternion[0];


   return;
}


