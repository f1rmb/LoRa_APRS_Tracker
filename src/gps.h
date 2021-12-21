#ifndef GPS_H_
#define GPS_H_

#include "configuration.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

class GPSDevice
{
    public:
        GPSDevice();
        ~GPSDevice();

        bool Initialize(HardwareSerial &serial, bool doBegin);
        bool FactoryReset();
        bool HasFix();
        bool HasData();

        bool GetPVT();
        bool GetDateAndTime(struct tm &t);

        double GetHeading();


        void SetWake(bool on);


        double GetLatitude();
        double GetLongitude();
        double GetAltitude();
        double GetAltitudeInFeet();
        double GetSpeedMS();
        double GetSpeedKMH();
        double GetSpeedKnot();

        uint8_t GetSatellites();
        double GetHDOP();

        static float LatLongToMeter(double lat_a, double lng_a, double lat_b, double lng_b);


    private:
        bool connect();
        bool setUBXMode();
        void wake();
        void sleep(uint32_t seconds);

    private:
        HardwareSerial   *m_serialGPS;
        bool              m_isConnected;
        SFE_UBLOX_GNSS    m_gnss;

        uint8_t           m_fixType;
        bool              m_wakeState;
        unsigned long     m_lastWakeTime;
        unsigned long     m_lastSleepTime;
};


#endif // GPS_H_

