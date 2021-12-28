

#include "pins.h"
#include "gps.h"

#define GPS_BAUDRATE     9600



GPSDevice::GPSDevice() :
m_serialGPS(NULL),
m_isConnected(false),
m_fixType(0),
m_lowPowerModeEnabled(false),
m_lastSleepTime(0),
m_sleepMS(0)
{
}

GPSDevice::~GPSDevice()
{

}

bool GPSDevice::Initialize(HardwareSerial &serial)
{
    m_serialGPS = &serial;
    m_serialGPS->begin(GPS_BAUDRATE, SERIAL_8N1, GPS_TX, GPS_RX);

    //m_gnss.enableDebugging(Serial);

    for (int i = 0; (i < 3) && !connect(); i++)
    {
         delay(500);
    }

    if (m_isConnected)
    {
        bool ubxInit;

        // Enable UBX mode
        for (int i = 0; (i < 3); i++)
        {
            if ((ubxInit = setUBXMode()))
            {
                break;
            }

            delay(300);
        }

        // UBX mode failed 3 times in a row -> factoryReset
        if (ubxInit == false)
        {
            ubxInit = FactoryReset();
        }

        return ubxInit;
    }

    return false;
}

bool GPSDevice::GetProtocolVersion(uint8_t &high, uint8_t &low)
{
    if (m_serialGPS && m_isConnected)
    {
        if (m_gnss.getProtocolVersion())
        {
            high = m_gnss.getProtocolVersionHigh();
            low = m_gnss.getProtocolVersionLow();

            return true;
        }
    }

    return false;
}

bool GPSDevice::FactoryReset()
{
    if (m_serialGPS && m_isConnected)
    {
        m_gnss.flushPVT();
        m_gnss.factoryReset();
    }

    delay(5000);
    connect();

    for (int i = 0; (i < 3) && !connect(); i++)
    {
         delay(500);
    }

    if (m_isConnected)
    {
        if (setUBXMode())
        {
            return true;
        }
    }

    return false;
}

GPSDevice::GPS_FIXTYPE_t GPSDevice::GetFixType()
{
    return static_cast<GPS_FIXTYPE_t>(m_gnss.getFixType());
}

bool GPSDevice::HasFix()
{
    return m_gnss.getGnssFixOk();
}

bool GPSDevice::HasData()
{
    return (m_gnss.checkUblox());
}

bool GPSDevice::SetLowPower(bool on, uint32_t millisecs)
{
    if (m_lowPowerModeEnabled != on)
    {
        bool result = false;

        if (on)
        {
#if 1
            for (uint8_t i = 0; (i < 3) && ((result = m_gnss.powerSaveMode(true, 2000U)) == false); i++)
            {
                delay(200);
            }

            m_sleepMS = millisecs;
#else
            if ((result = m_gnss.powerSaveMode(true, 2000U)))
            {
                m_sleepMS = millisecs;
            }
#endif
        }
        else
        {
#if 1
            for (uint8_t i = 0; (i < 3) && ((result = m_gnss.powerSaveMode(false, 2000U)) == false); i++)
            {
                delay(200);
            }
#else
            if ((result = m_gnss.powerSaveMode(false, 2000U)))
            {
                m_sleepMS = 0;
            }
#endif
        }

        m_lowPowerModeEnabled = (result ? on : false);

        if (result)
        {
            m_lastSleepTime = millis();
        }

        return result;
    }

    return false;
}

bool GPSDevice::StillHasToSleep()
{
    return (m_lowPowerModeEnabled && ((millis() - m_lastSleepTime) < m_sleepMS));
}

unsigned long GPSDevice::GetRemainingSleepTime()
{
    if (m_lowPowerModeEnabled)
    {
        unsigned long elapsed = (millis() - m_lastSleepTime);

        if (elapsed < m_sleepMS)
        {
            return (m_sleepMS - elapsed);
        }
    }

    return 0UL;
}

bool GPSDevice::IsSleeping()
{
#if 0
    if (m_lowPowerModeEnabled)
    {
        uint8_t lowPowerMode = m_gnss.getPowerSaveMode(2000U);

        if (lowPowerMode == 255)
        {
            Serial.println(F("*** getPowerSaveMode FAILED ***"));
        }
        else
        {
            Serial.print(F("The low power mode is: "));
            Serial.print(lowPowerMode);
            if (lowPowerMode == 0)
            {
                Serial.println(F(" (Continuous)"));
            }
            else if (lowPowerMode == 1)
            {
                Serial.println(F(" (Power Save)"));
            }
            else if (lowPowerMode == 4)
            {
                Serial.println(F(" (Continuous)"));
            }
            else
            {
                Serial.println(F(" (Unknown!)"));
            }
        }
    }
#endif

    return (m_lowPowerModeEnabled);
}

bool GPSDevice::GetPVT()
{
    return (m_gnss.getPVT() && (m_gnss.getInvalidLlh() == false));
}

bool GPSDevice::GetDateAndTime(struct tm &dt)
{
    if (m_gnss.getTimeValid() && m_gnss.getTimeFullyResolved())
    {
        /* Convert to unix time
         *  The Unix epoch (or Unix time or POSIX time or Unix timestamp) is the number of seconds that have elapsed
         *  since January 1, 1970 (midnight UTC/GMT), not counting leap seconds (in ISO 8601: 1970-01-01T00:00:00Z).
         */
        dt.tm_sec   = m_gnss.getSecond(0);
        dt.tm_min   = m_gnss.getMinute(0);
        dt.tm_hour  = m_gnss.getHour(0);
        dt.tm_mday  = m_gnss.getDay(0);
        dt.tm_mon   = m_gnss.getMonth(0) - 1;
        dt.tm_year  = m_gnss.getYear(0) - 1900;
        dt.tm_isdst = false;

        return true;
    }

    return false;
}

double GPSDevice::GetHeading()
{
    return (m_gnss.getHeading() * 1e-5);
}

double GPSDevice::GetLatitude()
{
    return (m_gnss.getLatitude() * 1e-7);
}

double GPSDevice::GetLongitude()
{
    return (m_gnss.getLongitude() * 1e-7);
}

double GPSDevice::GetAltitude() // Meters
{
    return (m_gnss.getAltitudeMSL() * 1e-3); // mm to meter
}

double GPSDevice::GetAltitudeFT()
{
    return (GetAltitude() * 3.2808399);
}

double GPSDevice::GetSpeedMPS()
{
    return (m_gnss.getGroundSpeed() * 1e-3); // mm/s to m/s
}

double GPSDevice::GetSpeedKPH()
{
    return (GetSpeedMPS() * 0.2777777778); // m/s to kilometer per hour
}

double GPSDevice::GetSpeedKT()
{
    return (GetSpeedMPS() * 1.9438444924); // m/s to knot
}

uint8_t GPSDevice::GetSatellites()
{
    return (m_gnss.getSIV());
}

double GPSDevice::GetHDOP()
{
    return ((std::min(uint16_t(9999), (m_gnss.getHorizontalDOP()))) * 1e-2); // Avoid crazy value
}

// Took from TinyGPSPlus
double GPSDevice::DistanceBetweenTwoCoords(double lat1, double long1, double lat2, double long2)
{
    // returns distance in meters between two positions, both specified
    // as signed decimal-degrees latitude and longitude. Uses great-circle
    // distance computation for hypothetical sphere of radius 6372795 meters.
    // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
    // Courtesy of Maarten Lamers
    double delta = radians(long1-long2);
    double sdlong = sin(delta);
    double cdlong = cos(delta);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    double slat1 = sin(lat1);
    double clat1 = cos(lat1);
    double slat2 = sin(lat2);
    double clat2 = cos(lat2);
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
    delta = sq(delta);
    delta += sq(clat2 * sdlong);
    delta = sqrt(delta);
    double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
    delta = atan2(delta, denom);

    return delta * 6372795;
}

bool GPSDevice::connect()
{
    m_isConnected = false;
    m_lowPowerModeEnabled = false;

    if (m_serialGPS)
    {
        m_isConnected = m_gnss.begin(*m_serialGPS);
    }

    return m_isConnected;
}

bool GPSDevice::setUBXMode()
{
    if (m_serialGPS && m_isConnected)
    {
        m_lowPowerModeEnabled = false;
        m_gnss.powerSaveMode(false);

        // Configure the U-Blox
        if (m_gnss.setUART1Output(COM_TYPE_UBX))
        {
            delay(2000);
            if (m_gnss.setNavigationFrequency(1) && m_gnss.setAutoPVT(true))
            {
                m_gnss.flushPVT();
                return true;
            }
        }
    }

    return false;
}
