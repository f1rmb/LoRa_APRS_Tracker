#ifndef GPS_H_
#define GPS_H_

#include "Configuration.h"
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
        void          Stop();

        // Get protocol version(s)
        bool          GetProtocolVersion(uint8_t &high, uint8_t &low);


        // Trigger a factory reset
        bool          FactoryReset();

        // Get the fix type (quality)
        GPS_FIXTYPE_t GetFixType();

        // GPS has a fix (valid fix (i.e within DOP & accuracy masks))
        bool          HasFix();

        // Check for available bytes on the user's specified port
        bool          Tick();

        // Position/Velocity/Time is available and
        // valid (also checking "Invalid lon, lat, height, hMSL, lonHp, latHp, heightHp and hMSLHp")
        bool          GetPVT();


        // Flush the current PVT packets, then re-enable the AutoPVT
        bool          FlushAndSetAutoPVT();
        bool          Flush();


        // Get GPS time (unix format), if valid.
        bool          GetDateAndTime(struct tm &t);

        bool          SetPowerOff(bool off, uint32_t duration);
        bool          SetPowerSaving(bool on);


        // Get PowerSaving status
        uint8_t       IsPowerSaving();


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

        bool          IsNeo6M();

    private:
        bool connect();
        bool setUBXMode();

    private:
        HardwareSerial       *m_serialGPS;
        bool                  m_isConnected;
        SFE_UBLOX_GNSS        m_gnss;
        SFE_UBLOX_GNSS_TYPE   m_gnssType;
        bool                  m_isPowerSaving;
};


#endif // GPS_H_

