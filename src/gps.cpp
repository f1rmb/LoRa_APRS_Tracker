

//#include <list>
//#include <Arduino.h>
#include "pins.h"
#include "gps.h"

#define GPS_BAUDRATE     9600



GPSDevice::GPSDevice() :
m_serialGPS(NULL),
m_isConnected(false),
m_fixType(0),
m_wakeState(true),
m_lastWakeTime(0),
m_lastSleepTime(0)
{
}

GPSDevice::~GPSDevice()
{

}

bool GPSDevice::Initialize(HardwareSerial &serial, bool doBegin)
{
    m_serialGPS = &serial;

    if (doBegin)
    {
        m_serialGPS->begin(GPS_BAUDRATE, SERIAL_8N1, GPS_TX, GPS_RX);
    }

    for (int i = 0; (i < 3) && !connect(); i++)
    {
         delay(500);
    }

    if (m_isConnected)
    {
        if (setUBXMode())
        {
            return true;

            m_wakeState = true;
            m_lastWakeTime = millis();
        }
    }

    return false;
}

bool GPSDevice::FactoryReset()
{
    if (m_serialGPS && m_isConnected)
    {
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


bool GPSDevice::HasFix()
{
    return m_gnss.getGnssFixOk();
    //return (m_fixType >= 3 && m_fixType <= 4);
}

bool GPSDevice::HasData()
{
    return (m_gnss.checkUblox());
}

void GPSDevice::SetWake(bool on)
{
    //m_fixType = 0;
    if (m_wakeState != on)
    {
        if (on)
        {
            m_lastWakeTime = millis();
            wake();
        }
        else
        {
            m_lastSleepTime = millis();
#warning sleep time
            sleep(20000);
        }

        m_wakeState = on;
    }
}

bool GPSDevice::GetPVT()
{
    return (m_gnss.getPVT());
}

bool GPSDevice::GetDateAndTime(struct tm &t)
{
    if (m_gnss.getTimeValid() && m_gnss.getConfirmedTime())
    {
        /* Convert to unix time
        The Unix epoch (or Unix time or POSIX time or Unix timestamp) is the number of seconds that have elapsed since January
        1, 1970 (midnight UTC/GMT), not counting leap seconds (in ISO 8601: 1970-01-01T00:00:00Z).
         */
        t.tm_sec = m_gnss.getSecond(0);
        t.tm_min = m_gnss.getMinute(0);
        t.tm_hour = m_gnss.getHour(0);
        t.tm_mday = m_gnss.getDay(0);
        t.tm_mon = m_gnss.getMonth(0) - 1;
        t.tm_year = m_gnss.getYear(0) - 1900;
        t.tm_isdst = false;
        return true;
    }

    return false;
}

double GPSDevice::GetHeading()
{
    return (m_gnss.getHeading(0) * 1e-5);
}

double GPSDevice::GetLatitude()
{
    return (m_gnss.getLatitude(0) * 1e-7);
}

double GPSDevice::GetLongitude()
{
    return (m_gnss.getLongitude(0) * 1e-7);
}

double GPSDevice::GetAltitude()
{
    return (m_gnss.getAltitude(0));
}

double GPSDevice::GetAltitudeInFeet()
{
    return (GetAltitude() / 304.8);
}

double GPSDevice::GetSpeedMS()
{
    return (m_gnss.getGroundSpeed(0)); // m/s
}

double GPSDevice::GetSpeedKMH()
{
    return (GetSpeedMS() * 0.2777777778);
}

double GPSDevice::GetSpeedKnot()
{
    return (GetSpeedMS() * 1.9438444924);
}

uint8_t GPSDevice::GetSatellites()
{
    return (m_gnss.getSIV(0));
}

double GPSDevice::GetHDOP()
{
    return (m_gnss.getHorizontalDOP() * 1e-7);
}

float GPSDevice::LatLongToMeter(double lat_a, double lng_a, double lat_b, double lng_b)
{
    double pk = (180 / 3.14169);
    double a1 = lat_a / pk;
    double a2 = lng_a / pk;
    double b1 = lat_b / pk;
    double b2 = lng_b / pk;
    double cos_b1 = cos(b1);
    double cos_a1 = cos(a1);
    double t1 = cos_a1 * cos(a2) * cos_b1 * cos(b2);
    double t2 = cos_a1 * sin(a2) * cos_b1 * sin(b2);
    double t3 = sin(a1) * sin(b1);
    double tt = acos(t1 + t2 + t3);
    if (std::isnan(tt))
    {
        tt = 0.0; // Must have been the same point?
    }

    return (float)(6366000 * tt);
}

bool GPSDevice::connect()
{
    m_isConnected = false;

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
        // Configure the U-Blox
        if (m_gnss.setUART1Output(COM_TYPE_UBX, 1000) && m_gnss.setNavigationFrequency(1, 1000))
        {
            return true; // Success
        }
    }

    return false;
}

void GPSDevice::wake()
{

}

void GPSDevice::sleep(uint32_t seconds)
{
    m_gnss.powerOff(seconds);
}
