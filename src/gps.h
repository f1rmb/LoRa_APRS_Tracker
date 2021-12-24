#ifndef GPS_H_
#define GPS_H_

#include "configuration.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

class GPSDevice
{
    public:
        typedef enum
        {
            GPS_FIXTYPE_NO_FIX = 0,
            GPS_FIXTYPE_DEAD_RECKONING,
            GPS_FIXTYPE_2D,
            GPS_FIXTYPE_3D,
            GPS_FIXTYPE_GNSS,
            GPS_FIXTYPE_TIME_FIX
        } GPS_FIXTYPE_t;

    public:
        GPSDevice();
        ~GPSDevice();

        bool Initialize(HardwareSerial &serial);
        bool FactoryReset();
        GPS_FIXTYPE_t GetFixType();
        bool HasFix();
        bool HasData();

        bool GetPVT();
        bool GetDateAndTime(struct tm &t);

        double GetHeading();


        bool SetLowPower(bool on, uint32_t millisecs);
        bool StillHasToSleep();
        unsigned long GetRemainingSleepTime();
        bool IsSleeping();

        double GetLatitude();    // in Degrees
        double GetLongitude();   // in Degrees
        double GetAltitude();    // In Meters
        double GetAltitudeFT();  // In Feet
        double GetSpeedMPS();    // in meter per second
        double GetSpeedKPH();    // in kilometer per hour
        double GetSpeedKT();     // in Knot

        uint8_t GetSatellites();
        double GetHDOP();

        static double DistanceBetweenTwoCoords(double lat_a, double lng_a, double lat_b, double lng_b);


    private:
        bool connect();
        bool setUBXMode();

    private:
        HardwareSerial   *m_serialGPS;
        bool              m_isConnected;
        SFE_UBLOX_GNSS    m_gnss;

        uint8_t           m_fixType;
        bool              m_lowPowerModeEnabled;
        unsigned long     m_lastSleepTime;
        uint32_t          m_sleepMS;
};


#endif // GPS_H_

