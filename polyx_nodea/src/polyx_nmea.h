#ifndef _POLYX_NMEA_H
#define _POLYX_NMEA_H

#include "polyx_nodea/nmeaGGA.h"

//-----------------------------------------------------------------------------
// Checks NMEA message checksum validity
// @return true if valid

bool nmeaChecksum(const char* msg);

//-----------------------------------------------------------------------------
// Parses NMEA GGA string into the decoded message format
void parseNmeaGga(const char* msg, polyx_nodea::nmeaGGA& gga);

#endif