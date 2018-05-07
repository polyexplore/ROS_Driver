#ifndef _POLYX_CONVERT_H
#define _POLYX_CONVERT_H

//-----------------------------------------------------------------------------
// GPS time to UNIX epoch conversion
//     315964800 + week * 604800 + GPS Time of week
//     where 315964800 is the Unix Epoch of GPS start time (Jan 6, 1980), 
//
void GpsToEpoch(int gpsweek, double gpstow, ros::Time &tm);

//-----------------------------------------------------------------------------
// UNIX epoch to GPS time conversion
void EpochToGps(const ros::Time &tm, int& gpsweek, double& gpstow);

//-----------------------------------------------------------------------------
// Quaternion product: q3 = q1 * q2
void QuatProd(
   const geometry_msgs::Quaternion& q1,
   const geometry_msgs::Quaternion& q2,
   geometry_msgs::Quaternion&       q3);

//-----------------------------------------------------------------------------
// Quaternion for the transformation from NED to ENU
void QuatNED2ENU(geometry_msgs::Quaternion& q);

//-----------------------------------------------------------------------------
// Convert PE ICD message to NavSatFix message
void icd_to_NavSatFix(
   polyx_nodea::Icd&       msg, 
   sensor_msgs::NavSatFix& nsf);

//-----------------------------------------------------------------------------
// Convert PE ICD message to IMU message
void icd_to_Imu(polyx_nodea::Icd& msg, sensor_msgs::Imu& imu);

//-----------------------------------------------------------------------------
// Convert PE ICD message to GeoPoseStamped message
void icd_to_GeoPoseStamped(
   polyx_nodea::Icd&                msg,
   geographic_msgs::GeoPoseStamped& ps);

//-----------------------------------------------------------------------------
// Convert PE ICD message to TwistStamped message
void icd_to_TwistStamped(
   polyx_nodea::Icd&            msg,
   geometry_msgs::TwistStamped& ts);

//-----------------------------------------------------------------------------
// Convert PE ICD message to AccelStamped message
void icd_to_AccelStamped(
   polyx_nodea::Icd&            msg,
   geometry_msgs::AccelStamped& as);

//-----------------------------------------------------------------------------
// Convert Geodetic coordinates to ECEF coordinates.
void GeodeticToECEF(
   const double& lat,
   const double& lon,
   const double& alt,
   double        r[]);

//-----------------------------------------------------------------------------
void DCM_ECEFToNED(const double& lat, const double& lon, double Cen[3][3]);

//-----------------------------------------------------------------------------
// Set origin of pose specifically for customers
void SetCustomOrigin(
   double               latitude,   // radian
   double               longitude,  // radian
   double               altitude,   // meters
   struct origin_type&  org);

//-----------------------------------------------------------------------------
// Set origin for Pose message
void SetOrigin(
   const polyx_nodea::Icd&     msg,
   struct origin_type&         org);

//-----------------------------------------------------------------------------
// Convert PE ICD message to PoseStamped message
void icd_to_PoseStamped(
   const polyx_nodea::Icd&     msg,
   const struct origin_type&   org,
   geometry_msgs::PoseStamped& ps);

//-----------------------------------------------------------------------------
// Convert quaternion to Euler angles
bool EulerAttitude(
   polyx_nodea::Icd&           msg, 
   polyx_nodea::EulerAttitude& qtemsg);

#endif // _POLYX_CONVERT_H