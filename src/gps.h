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

        // Initialize GPS module, using serial port "serial".
        bool          Initialize(HardwareSerial &serial);

        // Trigger a factory reset
        bool          FactoryReset();

        // Get the fix type (quality)
        GPS_FIXTYPE_t GetFixType();

        // GPS has a fix (valid fix (i.e within DOP & accuracy masks))
        bool          HasFix();

        // Check for available bytes on the user's specified port
        bool          HasData();

        // Position/Velocity/Time is available and
        // valid (also checking "Invalid lon, lat, height, hMSL, lonHp, latHp, heightHp and hMSLHp")
        bool          GetPVT();

        // Get GPS time (unix format), if valid.
        bool          GetDateAndTime(struct tm &t);

        // Enter Power Saving mode for "millisecs" long (could also be forced to exit)
        bool          SetLowPower(bool on, uint32_t millisecs);

        // Get GPS sleeping (PowerSave) state
        bool          StillHasToSleep();

        // Get remaining Sleep time, in milliseconds (0 == awake)
        unsigned long GetRemainingSleepTime();

        // Get Sleeping status
        bool          IsSleeping();

        // Get various GPS informations (call GetPVT() first)
        double        GetHeading();     // in Degrees
        double        GetLatitude();    // in Degrees
        double        GetLongitude();   // in Degrees
        double        GetAltitude();    // In Meters
        double        GetAltitudeFT();  // In Feet
        double        GetSpeedMPS();    // in meter per second
        double        GetSpeedKPH();    // in kilometer per hour
        double        GetSpeedKT();     // in Knot
        uint8_t       GetSatellites();  // number of acquired satellites
        double        GetHDOP();        // Horizontal Dilution Of Precision

        // Return distance between two locations, in meters
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

