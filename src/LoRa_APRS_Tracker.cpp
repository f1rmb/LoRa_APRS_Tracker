#include <APRS-Decoder.h>
#include <Arduino.h>
#include <LoRa.h>
#include <OneButton.h>
#include <TimeLib.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <logger.h>

#include "configuration.h"
#include "display.h"
#include "pins.h"
#include "power_management.h"

static Configuration     cfg;
static OLEDDisplay       oled;
static PowerManagement   pm;
static OneButton         userBtn(BUTTON_PIN, true, true);
static HardwareSerial    ss(1);
static TinyGPSPlus       gps;


struct GlobalParameters
{
        GlobalParameters() :
            forcePositionUpdate(true),
            nextBeaconTimeStamp(-1),
            currentHeading(0),
            previousHeading(0),
            rateLimitMessageText(0),
            lastTxLat(0.0),
            lastTxLng(0.0),
            lastTxdistance(0.0),
            txInterval(60000L), // Initial 60 secs internal
            lastTxTime(millis()),
            speedZeroSent(0),
            batteryIsConnected(false),
            batteryVoltage(""),
            batteryChargeCurrent(""),
#ifdef TTGO_T_Beam_V1_0
            rateLimitCheckBattery(0),
#endif
            m_displayTimeout(Configuration::CONFIGURATION_DISPLAY_TIMEOUT),
            m_displayLastTimeout(millis()),
            m_displayTimeoutEnabled(true)
        {
        }

        void SetDisplayTimeout(uint32_t timeout)
        {
            m_displayTimeout = timeout;

            if (timeout == 0) // If timeout is set to 0ms, disable it
            {
                m_displayTimeoutEnabled = false;
            }
        }

        void ResetDisplayTimeout()
        {
#ifdef TTGO_T_Beam_V1_0
            if (oled.IsActivated() == false)
            {
                oled.Activate(true);
            }
#endif
            m_displayTimeoutEnabled = (m_displayTimeout > 0 ? true : false);
            m_displayLastTimeout = millis();
        }

        void DisableDisplayTimeout()
        {
            m_displayTimeoutEnabled = false;
        }

        void DisplayTick()
        {
#ifdef TTGO_T_Beam_V1_0
            if (oled.IsActivated())
            {
                if (m_displayTimeoutEnabled)
                {
                    if ((millis() - m_displayLastTimeout) > m_displayTimeout)
                    {
                        oled.Activate(false);
                    }
                }
            }
#endif
        }

        bool           forcePositionUpdate;
        time_t         nextBeaconTimeStamp;
        double         currentHeading;
        double         previousHeading;
        unsigned int   rateLimitMessageText;
        double         lastTxLat;
        double         lastTxLng;
        double         lastTxdistance;
        uint32_t       txInterval;
        uint32_t       lastTxTime;
        int            speedZeroSent;
        bool           batteryIsConnected;
        String         batteryVoltage;
        String         batteryChargeCurrent;
#ifdef TTGO_T_Beam_V1_0
        unsigned int   rateLimitCheckBattery;
#endif

    private:
        uint32_t       m_displayTimeout;
        unsigned long  m_displayLastTimeout;
        bool           m_displayTimeoutEnabled;
};


static GlobalParameters  gParams;


static void load_configuration();
static void lora_configuration();
static void gps_configuration();

static String create_lat_aprs(RawDegrees lat);
static String create_long_aprs(RawDegrees lng);
static String create_lat_aprs_dao(RawDegrees lat);
static String create_long_aprs_dao(RawDegrees lng);
static String create_dao_aprs(RawDegrees lat, RawDegrees lng);
static String createDateString(time_t t);
static String createTimeString(time_t t);
static String getSmartBeaconState();
static String padding(unsigned int number, unsigned int width);

static void handle_tx_click()
{
#ifdef TTGO_T_Beam_V1_0
    bool oledWasOn = oled.IsActivated();

    gParams.ResetDisplayTimeout(); // Reset the OLED timeout on any button event

    if (oledWasOn == false) // The OLED was off, hence we won't go further this time
    {
        return;
    }
#endif

    if (cfg.beacon.button_tx)
    {
        // attach TX action to user button (defined by BUTTON_PIN)
        gParams.forcePositionUpdate = true;
    }
}

// cppcheck-suppress unusedFunction
void setup()
{
    Serial.begin(115200);

    gParams.DisableDisplayTimeout(); // Keep the OLED screen ON

#ifdef TTGO_T_Beam_V1_0
    Wire.begin(SDA, SCL);
    if (!pm.begin(Wire))
    {
        logPrintlnI("AXP192 init done!");
    }
    else
    {
        logPrintlnE("AXP192 init failed!");
    }
    pm.LoRaActivate();
    pm.OLEDActivate();
    pm.GPSActivate();
    pm.MeasurementsActivate();
#endif

    delay(500);
    logPrintlnI("LoRa APRS Tracker by OE5BPA (Peter Buchegger)");
    oled.Init();

    oled.Display("OE5BPA", "LoRa APRS Tracker", "by Peter Buchegger", emptyString, emptyString, "Mods: F1RMB - v0.99");

    load_configuration();
    gps_configuration();

    delay(2000);

    lora_configuration();

    if (cfg.ptt.active)
    {
        pinMode(cfg.ptt.io_pin, OUTPUT);
        digitalWrite(cfg.ptt.io_pin, cfg.ptt.reverse ? HIGH : LOW);
    }

    gParams.SetDisplayTimeout(cfg.display_timeout);

    // make sure wifi and bt is off as we don't need it:
    WiFi.mode(WIFI_OFF);
    btStop();

    userBtn.attachClick(handle_tx_click);

    logPrintlnI("Smart Beacon is " + getSmartBeaconState());
    oled.Display("INFO", "Smart Beacon is " + getSmartBeaconState(), 1000);
    logPrintlnI("setup done...");

    delay(500);

    gParams.ResetDisplayTimeout(); // Enable OLED timeout
}

// cppcheck-suppress unusedFunction
void loop()
{
    userBtn.tick();
    gParams.DisplayTick();

    if (cfg.debug)
    {
        while (Serial.available() > 0)
        {
            char c = Serial.read();
            // Serial.print(c);
            gps.encode(c);
        }
    }
    else
    {
        while (ss.available() > 0)
        {
            char c = ss.read();
            // Serial.print(c);
            gps.encode(c);
        }
    }

    bool gps_time_update = gps.time.isUpdated();
    bool gps_loc_update  = gps.location.isUpdated();

    if (gps.time.isValid())
    {
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());

        if (gps_loc_update && gParams.nextBeaconTimeStamp <= now())
        {
            gParams.forcePositionUpdate = true;

            if (cfg.smart_beacon.active)
            {
                gParams.currentHeading = gps.course.deg();
                // enforce message text on slowest cfg.smart_beacon.slow_rate
                gParams.rateLimitMessageText = 0;
            }
            else
            {
                // enforce message text every n's cfg.beacon.timeout frame
                if (cfg.beacon.timeout * gParams.rateLimitMessageText > 30)
                {
                    gParams.rateLimitMessageText = 0;
                }
            }
        }
    }


#ifdef TTGO_T_Beam_V1_0
    if (!(gParams.rateLimitCheckBattery++ % 60))
    {
        gParams.batteryIsConnected = pm.isBatteryConnected();
    }

    if (gParams.batteryIsConnected)
    {
        gParams.batteryVoltage       = String(pm.getBatteryVoltage(), 2);
        gParams.batteryChargeCurrent = String(pm.getBatteryChargeDischargeCurrent(), 0);
    }
#endif

    if (!gParams.forcePositionUpdate && gps_loc_update && cfg.smart_beacon.active)
    {
        uint32_t lastTx = millis() - gParams.lastTxTime;

        gParams.currentHeading = gps.course.deg();
        gParams.lastTxdistance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), gParams.lastTxLat, gParams.lastTxLng);

        if (lastTx >= gParams.txInterval)
        {
            // Trigger Tx Tracker when Tx interval is reach
            // Will not Tx if stationary bcos speed < 5 and lastTxDistance < 20
            if (gParams.lastTxdistance > 20)
            {
                gParams.forcePositionUpdate = true;
            }
        }

        if (!gParams.forcePositionUpdate)
        {
            // Get headings and heading delta
            double headingDelta = abs(gParams.previousHeading - gParams.currentHeading);

            if (lastTx > cfg.smart_beacon.min_bcn * 1000)
            {
                // Check for heading more than 25 degrees
                if (headingDelta > cfg.smart_beacon.turn_min && gParams.lastTxdistance > cfg.smart_beacon.min_tx_dist)
                {
                    gParams.forcePositionUpdate = true;
                }
            }
        }
    }

    if (gParams.forcePositionUpdate && gps_loc_update)
    {
        APRSMessage msg;
        String      lat;
        String      lng;
        String      dao;

        gParams.forcePositionUpdate         = false;
        gParams.nextBeaconTimeStamp = now() + (cfg.smart_beacon.active ? cfg.smart_beacon.slow_rate : (cfg.beacon.timeout * SECS_PER_MIN));

        msg.setSource(cfg.callsign);
        msg.setDestination("APLT00-1");

        if (!cfg.enhance_precision)
        {
            lat = create_lat_aprs(gps.location.rawLat());
            lng = create_long_aprs(gps.location.rawLng());
        }
        else
        {
            lat = create_lat_aprs_dao(gps.location.rawLat());
            lng = create_long_aprs_dao(gps.location.rawLng());
            dao = create_dao_aprs(gps.location.rawLat(), gps.location.rawLng());
        }


        String alt     = "";
        int    alt_int = max(-99999, min(999999, (int)gps.altitude.feet()));

        if (alt_int < 0)
        {
            alt = "/A=-" + padding(alt_int * -1, 5);
        }
        else
        {
            alt = "/A=" + padding(alt_int, 6);
        }


        String course_and_speed = "";
        int    speed_int        = max(0, min(999, (int)gps.speed.knots()));

        if (gParams.speedZeroSent < 3)
        {
            String speed      = padding(speed_int, 3);
            int    course_int = max(0, min(360, (int)gps.course.deg()));

            /* course in between 1..360 due to aprs spec */
            if (course_int == 0)
            {
                course_int = 360;
            }

            String course    = padding(course_int, 3);
            course_and_speed = course + "/" + speed;
        }

        if (speed_int == 0)
        {
            /* speed is 0.
         we send 3 packets with speed zero (so our friends know we stand still).
         After that, we save airtime by not sending speed/course 000/000.
         Btw, even if speed we really do not move, measured course is changeing
         (-> no useful / even wrong info)
             */
            if (gParams.speedZeroSent < 3)
            {
                gParams.speedZeroSent++;
            }
        }
        else
        {
            gParams.speedZeroSent = 0;
        }


        String aprsmsg;
        aprsmsg = "!" + lat + cfg.beacon.overlay + lng + cfg.beacon.symbol + course_and_speed + alt;

        // message_text every 10's packet (i.e. if we have beacon rate 1min at high
        // speed -> every 10min). May be enforced above (at expirey of smart beacon
        // rate (i.e. every 30min), or every third packet on static rate (i.e.
        // static rate 10 -> every third packet)
        if (!(gParams.rateLimitMessageText++ % 10))
        {
            aprsmsg += cfg.beacon.message;
        }

        if (gParams.batteryIsConnected)
        {
            aprsmsg += " -  _Bat.: " + gParams.batteryVoltage + "V - Cur.: " + gParams.batteryChargeCurrent + "mA";
        }

        if (cfg.enhance_precision)
        {
            aprsmsg += " " + dao;
        }

        msg.getAPRSBody()->setData(aprsmsg);
        String data = msg.encode();
        logPrintlnD(data);
        oled.Display("<< TX >>", data);

        if (cfg.ptt.active)
        {
            digitalWrite(cfg.ptt.io_pin, cfg.ptt.reverse ? LOW : HIGH);
            delay(cfg.ptt.start_delay);
        }

        LoRa.beginPacket();
        // Header:
        LoRa.write('<');
        LoRa.write(0xFF);
        LoRa.write(0x01);
        // APRS Data:
        LoRa.write((const uint8_t *)data.c_str(), data.length());
        LoRa.endPacket();

        if (cfg.smart_beacon.active)
        {
            gParams.lastTxLat       = gps.location.lat();
            gParams.lastTxLng       = gps.location.lng();
            gParams.previousHeading = gParams.currentHeading;
            gParams.lastTxdistance  = 0.0;
            gParams.lastTxTime      = millis();
        }

        if (cfg.ptt.active)
        {
            delay(cfg.ptt.end_delay);
            digitalWrite(cfg.ptt.io_pin, cfg.ptt.reverse ? HIGH : LOW);
        }
    }

    if (gps_time_update)
    {
        oled.Display(cfg.callsign,
                createDateString(now()) + " " + createTimeString(now()),
                String("Sats: ") + gps.satellites.value() + " HDOP: " + gps.hdop.hdop(),
                String("Nxt Bcn: ") + (cfg.smart_beacon.active ? "~" : "") + createTimeString(gParams.nextBeaconTimeStamp),
                (gParams.batteryIsConnected ? (String("Bat: ") + gParams.batteryVoltage + "V, " + gParams.batteryChargeCurrent + "mA") : "Powered via USB"),
                String("Smart Beacon: " + getSmartBeaconState()));

        if (cfg.smart_beacon.active)
        {
            // Change the Tx internal based on the current speed
            int curr_speed = (int)gps.speed.kmph();
            if (curr_speed < cfg.smart_beacon.slow_speed)
            {
                gParams.txInterval = cfg.smart_beacon.slow_rate * 1000;
            }
            else if (curr_speed > cfg.smart_beacon.fast_speed)
            {
                gParams.txInterval = cfg.smart_beacon.fast_rate * 1000;
            }
            else
            {
                /* Interval inbetween low and high speed
                 * min(slow_rate, ..) because: if slow rate is 300s at slow speed <=
                 * 10km/h and fast rate is 60s at fast speed >= 100km/h everything below
                 * current speed 20km/h (100*60/20 = 300) is below slow_rate.
                 * -> In the first check, if curr speed is 5km/h (which is < 10km/h), tx
                 * interval is 300s, but if speed is 6km/h, we are landing in this
                 * section, what leads to interval 100*60/6 = 1000s (16.6min) -> this
                 * would lead to decrease of beacon rate in between 5 to 20 km/h. what
                 * is even below the slow speed rate.
                 */
                gParams.txInterval = min(cfg.smart_beacon.slow_rate, cfg.smart_beacon.fast_speed * cfg.smart_beacon.fast_rate / curr_speed) * 1000;
            }
        }
    }

    if ((cfg.debug == false) && (millis() > 5000 && gps.charsProcessed() < 10))
    {
        logPrintlnE("No GPS frames detected! Try to reset the GPS Chip with this "
                "firmware: https://github.com/lora-aprs/TTGO-T-Beam_GPS-reset");
    }
}

static void load_configuration()
{
    ConfigurationManagement confmg("/tracker.json");
    cfg = confmg.readConfiguration();

    if (cfg.callsign.startsWith("NOCALL"))
    {
        logPrintlnE("You have to change your settings in 'data/tracker.json' and "
                "upload it via \"Upload File System image\"!");
        oled.Display("ERROR", "You have to change your settings in 'data/tracker.json' and "
                "upload it via \"Upload File System image\"!");

        while (true) {}
    }
}

static void lora_configuration()
{
    logPrintlnI("Set SPI pins!");
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    logPrintlnI("Set LoRa pins!");
    LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

    long freq = cfg.lora.frequencyTx;
    logPrintI("frequency: ");
    logPrintlnI(String(freq));

    if (!LoRa.begin(freq))
    {
        logPrintlnE("Starting LoRa failed!");
        oled.Display("ERROR", "Starting LoRa failed!");

        while (true) {}
    }

    LoRa.setSpreadingFactor(cfg.lora.spreadingFactor);
    LoRa.setSignalBandwidth(cfg.lora.signalBandwidth);
    LoRa.setCodingRate4(cfg.lora.codingRate4);
    LoRa.enableCrc();

    LoRa.setTxPower(cfg.lora.power);
    logPrintlnI("LoRa init done!");
    oled.Display("INFO", "LoRa init done!", 2000);
}

static void gps_configuration()
{
    ss.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
}

static char *s_min_nn(uint32_t min_nnnnn, int high_precision)
{
    /* min_nnnnn: RawDegrees billionths is uint32_t by definition and is n'telth
     * degree (-> *= 6 -> nn.mmmmmm minutes) high_precision: 0: round at decimal
     * position 2. 1: round at decimal position 4. 2: return decimal position 3-4
     * as base91 encoded char
     */

    static char buf[6];
    min_nnnnn = min_nnnnn * 0.006;

    if (high_precision)
    {
        if ((min_nnnnn % 10) >= 5 && min_nnnnn < 6000000 - 5)
        {
            // round up. Avoid overflow (59.999999 should never become 60.0 or more)
            min_nnnnn = min_nnnnn + 5;
        }

    }
    else
    {
        if ((min_nnnnn % 1000) >= 500 && min_nnnnn < (6000000 - 500))
        {
            // round up. Avoid overflow (59.9999 should never become 60.0 or more)
            min_nnnnn = min_nnnnn + 500;
        }
    }

    if (high_precision < 2)
    {
        sprintf(buf, "%02u.%02u", (unsigned int)((min_nnnnn / 100000) % 100), (unsigned int)((min_nnnnn / 1000) % 100));
    }
    else
    {
        sprintf(buf, "%c", (char)((min_nnnnn % 1000) / 11) + 33);
    }

    // Like to verify? type in python for i.e. RawDegrees billions 566688333: i =
    // 566688333; "%c" % (int(((i*.0006+0.5) % 100)/1.1) +33)
    return buf;
}

static String create_lat_aprs(RawDegrees lat)
{
    char str[20];
    char n_s = 'N';

    if (lat.negative)
    {
        n_s = 'S';
    }

    // we like sprintf's float up-rounding.
    // but sprintf % may round to 60.00 -> 5360.00 (53° 60min is a wrong notation
    // ;)
    sprintf(str, "%02d%s%c", lat.deg, s_min_nn(lat.billionths, 0), n_s);
    return String(str);
}

static String create_lat_aprs_dao(RawDegrees lat)
{
    // round to 4 digits and cut the last 2
    char str[20];
    char n_s = 'N';

    if (lat.negative)
    {
        n_s = 'S';
    }

    // we need sprintf's float up-rounding. Must be the same principle as in
    // aprs_dao(). We cut off the string to two decimals afterwards. but sprintf %
    // may round to 60.0000 -> 5360.0000 (53° 60min is a wrong notation ;)
    sprintf(str, "%02d%s%c", lat.deg, s_min_nn(lat.billionths, 1 /* high precision */), n_s);
    return String(str);
}

static String create_long_aprs(RawDegrees lng)
{
    char str[20];
    char e_w = 'E';

    if (lng.negative)
    {
        e_w = 'W';
    }

    sprintf(str, "%03d%s%c", lng.deg, s_min_nn(lng.billionths, 0), e_w);
    return String(str);
}

static String create_long_aprs_dao(RawDegrees lng)
{
    // round to 4 digits and cut the last 2
    char str[20];
    char e_w = 'E';

    if (lng.negative)
    {
        e_w = 'W';
    }

    sprintf(str, "%03d%s%c", lng.deg, s_min_nn(lng.billionths, 1 /* high precision */), e_w);
    return String(str);
}

static String create_dao_aprs(RawDegrees lat, RawDegrees lng)
{
    // !DAO! extension, use Base91 format for best precision
    // /1.1 : scale from 0-99 to 0-90 for base91, int(... + 0.5): round to nearest
    // integer https://metacpan.org/dist/Ham-APRS-FAP/source/FAP.pm
    // http://www.aprs.org/aprs12/datum.txt
    //

    char str[10];
    // s_min_nn()'s high_precision parameter >= 2 ==> 1 char length
    sprintf(str, "!w%1s%1s!", s_min_nn(lat.billionths, 2), s_min_nn(lng.billionths, 2));

    return String(str);
}

static String createDateString(time_t t)
{
    return String(padding(day(t), 2) + "." + padding(month(t), 2) + "." + padding(year(t), 4));
}

static String createTimeString(time_t t)
{
    return String(padding(hour(t), 2) + "." + padding(minute(t), 2) + "." + padding(second(t), 2));
}

static String getSmartBeaconState()
{
    return String(cfg.smart_beacon.active ? "On" : "Off");
}

static String padding(unsigned int number, unsigned int width)
{
    char buffer[64];

    snprintf(buffer, sizeof(buffer), "%.*d", width, number);

    return String(buffer);
}
