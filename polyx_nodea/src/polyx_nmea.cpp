#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <stdint.h>

#include "polyx_nmea.h"

//-----------------------------------------------------------------------------
bool nmeaChecksum(const char* msg)
{
   uint16_t p = (uint16_t)strlen(msg) - 1;

   while (p > 0 && msg[p] != '*')
   {
      --p;
   }

   if (p > 0)
   {
      uint8_t c = 0;
      char cs[6];

      // Exclude beginning "$"

      for (int i = 1; i < p; ++i)
         c ^= msg[i];

      sprintf(cs, "%02X\r\n", c);

      return cs[0] == msg[p + 1] && cs[1] == msg[p + 2];
   }
   else
      return false;
}

//-----------------------------------------------------------------------------
void parseNmeaGga(const char* msg, polyx_nodea::nmeaGGA& gga)
{
   int n_delim = 0;
   int delim[15];

   // find delimiter positions
   for (int i = 0; i < strlen(msg); ++i)
   {
      if (msg[i] == ',' || msg[i] == '*' && n_delim < 15)
         delim[n_delim++] = i;
   }

   if (n_delim >= 15)
   {
      int n;
      char field[30];

      // Decode UTC time

      if (delim[1] - delim[0] > 5)
      {
         field[0] = msg[delim[0] + 1];
         field[1] = msg[delim[0] + 2];
         field[2] = '\0';

         gga.utc_hour = static_cast<uint8_t>(atoi(field));

         field[0] = msg[delim[0] + 3];
         field[1] = msg[delim[0] + 4];

         gga.utc_minute = static_cast<uint8_t>(atoi(field));

         n = delim[1] - delim[0] - 5;
         strncpy(field, &msg[delim[0] + 5], n);
         field[n] = '\0';
         gga.utc_millisec = static_cast<uint16_t>(atof(field) * 1000.0);
      }
      else
      {
         gga.utc_hour = 0;
         gga.utc_minute = 0;
         gga.utc_millisec = 0;
      }

      // Latitude
      if (delim[2] - delim[1] > 3)
      {
         field[0] = msg[delim[1] + 1];
         field[1] = msg[delim[1] + 2];
         field[2] = '\0';
         gga.latitude = atof(field);

         n = delim[2] - delim[1] - 3;
         strncpy(field, &msg[delim[1] + 3], n);
         field[n] = '\0';
         gga.latitude += atof(field) / 60.0;

         if (msg[delim[2] + 1] == 'S')
            gga.latitude *= -1.0;
      }
      else
         gga.latitude = 0;

      // Longitude
      if (delim[4] - delim[3] > 4)
      {
         strncpy(field, &msg[delim[3] + 1], 3);
         field[3] = '\0';
         gga.longitude = atof(field);

         n = delim[4] - delim[3] - 4;
         strncpy(field, &msg[delim[3] + 4], n);
         field[n] = '\0';
         gga.longitude += atof(field) / 60.0;

         if (msg[delim[4] + 1] == 'W')
            gga.longitude *= -1.0;
      }
      else
         gga.longitude = 0;

      // GNSS fix quality
      n = delim[6] - delim[5] - 1;
      if (n > 0)
      {
         strncpy(field, &msg[delim[5] + 1], n);
         field[n] = '\0';

         gga.fix_quality = static_cast<uint8_t>(atoi(field));
      }
      else
         gga.fix_quality = 0;

      // Number of satellites used
      n = delim[7] - delim[6] - 1;
      if (n > 0)
      {
         strncpy(field, &msg[delim[6] + 1], n);
         field[n] = '\0';

         gga.n_sv_used = static_cast<uint8_t>(atoi(field));
      }
      else
         gga.n_sv_used = 0;

      // HDOP
      n = delim[8] - delim[7] - 1;
      if (n > 0)
      {
         strncpy(field, &msg[delim[7] + 1], n);
         field[n] = '\0';

         gga.HDOP = static_cast<float>(atof(field));
      }
      else
         gga.HDOP = 0;

      // Orthometric height
      n = delim[9] - delim[8] - 1;
      if (n > 0)
      {
         strncpy(field, &msg[delim[8] + 1], n);
         field[n] = '\0';

         gga.orthometric_height = atof(field);
      }
      else
         gga.orthometric_height = 0;

      // Geoid undulation
      n = delim[11] - delim[10] - 1;
      if (n > 0)
      {
         strncpy(field, &msg[delim[10] + 1], n);
         field[n] = '\0';

         gga.geoid_undulation = static_cast<float>(atof(field));
      }
      else
         gga.geoid_undulation = 0;

      // Differential age
      n = delim[13] - delim[12] - 1;
      if (n > 0)
      {
         strncpy(field, &msg[delim[12] + 1], n);
         field[n] = '\0';

         gga.differential_age = static_cast<float>(atof(field));
      }
      else
         gga.differential_age = 0;

      // Reference station ID
      n = delim[14] - delim[13] - 1;
      if (n > 0)
      {
         strncpy(field, &msg[delim[13] + 1], n);
         field[n] = '\0';

         gga.ref_station_ID = static_cast<uint16_t>(atoi(field));
      }
      else
         gga.ref_station_ID = 0;

   }
}