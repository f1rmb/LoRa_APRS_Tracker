#include <APRS-Decoder.h>
#include <Arduino.h>
#include <LoRa.h>
#include <OneButton.h>
#include <TimeLib.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <logger.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

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
            nextBeaconTimeStamp(now()),
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
            loraIsBusy(false),
            gpsIsSleeping(false),
            gpsSleepActionTime(0),
#ifdef TTGO_T_Beam_V1_0
            batteryLastCheckTime(0),
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
        unsigned long  lastTxTime;
        int            speedZeroSent;
        bool           batteryIsConnected;
        String         batteryVoltage;
        String         batteryChargeCurrent;
        volatile bool  loraIsBusy;
        bool           gpsIsSleeping;
        unsigned long  gpsSleepActionTime;
#ifdef TTGO_T_Beam_V1_0
        unsigned long  batteryLastCheckTime;
#endif

    private:
        uint32_t       m_displayTimeout;
        unsigned long  m_displayLastTimeout;
        bool           m_displayTimeoutEnabled;
};


static GlobalParameters  gParams;


static void execSleep(uint32_t milliseconds);
static void buttonClickCallback();
static void loraTXDoneCallback();
static void loadConfiguration();
static void loraConfiguration();
static void gpsConfiguration();
static void gpsSuspend(bool suspend);
static void gpsReset();
static void gpsCheck();


static String createLatAPRS(RawDegrees lat);
static String createLongAPRS(RawDegrees lng);
static String createLatAPRSDAO(RawDegrees lat);
static String createLongAPRSDAO(RawDegrees lng);
static String createAPRSDAO(RawDegrees lat, RawDegrees lng);
static String createDateString(time_t t);
static String createTimeString(time_t t);
static String getSmartBeaconState();
static String padding(unsigned int number, unsigned int width);

// cppcheck-suppress unusedFunction
void setup()
{
    // Save some power, switch from 240MHz to 80MHz, power consummption from 66.8mA to 33.2mA
    //setCpuFrequencyMhz(80);
    setCpuFrequencyMhz(160);

    // Log only Errors
    Logger::instance().setDebugLevel(Logger::DEBUG_LEVEL_ERROR);

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

    oled.Display("OE5BPA", "LoRa APRS Tracker", "by Peter Buchegger", emptyString, emptyString, "Mods: F1RMB - v0.101");

    loadConfiguration();
    gpsConfiguration();

    delay(2000);

    loraConfiguration();

    if (cfg.ptt.active)
    {
        pinMode(cfg.ptt.io_pin, OUTPUT);
        digitalWrite(cfg.ptt.io_pin, cfg.ptt.reverse ? HIGH : LOW);
    }

    gParams.SetDisplayTimeout(cfg.display_timeout);

    // make sure wifi and bt is off as we don't need it:
    WiFi.mode(WIFI_OFF);
    btStop();

    userBtn.attachClick(buttonClickCallback);
    userBtn.tick();

    logPrintlnI("Smart Beacon is " + getSmartBeaconState());
    oled.Display("INFO", "Smart Beacon is " + getSmartBeaconState(), 1000);
    logPrintlnI("setup done...");

    delay(500);
    userBtn.tick();

    if (userBtn.isIdle() == false)
    {
        gpsReset();
        //gpsCheck();

        while (true) { }
    }

    oled.Display("INFO", "Waiting for Position");

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
            //Serial.print(c);
            gps.encode(c);
        }
    }

    bool gps_time_update = gps.time.isUpdated();
    bool gps_loc_update  = gps.location.isUpdated();

    if (gps.time.isValid())
    {
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());

        if (gps_loc_update && (gParams.nextBeaconTimeStamp <= now()))
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
    unsigned long m = millis();
    // Update the battery every 60 seconds
    if ((gParams.batteryLastCheckTime == 0) || ((m - gParams.batteryLastCheckTime) > 60000))
    {
        gParams.batteryIsConnected = pm.isBatteryConnected();
        gParams.batteryLastCheckTime = m;

        if (gParams.batteryIsConnected)
        {
            gParams.batteryVoltage       = String(pm.getBatteryVoltage(), 2);
            gParams.batteryChargeCurrent = String(pm.getBatteryChargeDischargeCurrent(), 0);
        }
    }
#endif

    if ((gParams.forcePositionUpdate == false) && gps_loc_update && cfg.smart_beacon.active)
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

            if (lastTx > (cfg.smart_beacon.min_bcn * 1000))
            {
                // Check for heading more than 25 degrees
                if ((headingDelta > cfg.smart_beacon.turn_min) && (gParams.lastTxdistance > cfg.smart_beacon.min_tx_dist))
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

        gParams.forcePositionUpdate = false;
        gParams.nextBeaconTimeStamp = now() + (cfg.smart_beacon.active ? cfg.smart_beacon.slow_rate : (cfg.beacon.timeout * SECS_PER_MIN));

        msg.setSource(cfg.callsign);
        msg.setDestination("APLT00-1");

        if (cfg.enhance_precision == false)
        {
            lat = createLatAPRS(gps.location.rawLat());
            lng = createLongAPRS(gps.location.rawLng());
        }
        else
        {
            lat = createLatAPRSDAO(gps.location.rawLat());
            lng = createLongAPRSDAO(gps.location.rawLng());
            dao = createAPRSDAO(gps.location.rawLat(), gps.location.rawLng());
        }


        String alt;
        int    alt_int = std::max(-99999, std::min(999999, (int)gps.altitude.feet()));

#if 1
        alt = String("/A=") + (alt_int < 0 ? "-" : "") + padding((unsigned int)abs(alt_int), ((alt_int < 0) ? 5 : 6));
        //Serial.println(alt.c_str());
#else
        if (alt_int < 0)
        {
            alt = "/A=-" + padding(alt_int * -1, 5);
        }
        else
        {
            alt = "/A=" + padding(alt_int, 6);
        }
#endif

        String course_and_speed;
        int    speed_int        = std::max(0, std::min(999, (int)gps.speed.knots()));

        if (gParams.speedZeroSent < 3)
        {
            String speed      = padding(speed_int, 3);
            int    course_int = std::max(0, std::min(360, (int)gps.course.deg()));

            /* course in between 1..360 due to aprs spec */
            if (course_int == 0)
            {
                course_int = 360;
            }

            String course(padding(course_int, 3));
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


        String aprsmsg("!" + lat + cfg.beacon.overlay + lng + cfg.beacon.symbol + course_and_speed + alt);

        // message_text every 10's packet (i.e. if we have beacon rate 1min at high
        // speed -> every 10min). May be enforced above (at expirey of smart beacon
        // rate (i.e. every 30min), or every third packet on static rate (i.e.
        // static rate 10 -> every third packet)
        if ((gParams.rateLimitMessageText++ % 10) == 0)
        {
            aprsmsg += cfg.beacon.message;
        }

        if (gParams.batteryIsConnected)
        {
            aprsmsg += " -  _Bat.: " + gParams.batteryVoltage + "V - Cur.: " + gParams.batteryChargeCurrent + "mA";
        }


        if (cfg.enhance_precision && (dao.length() > 0))
        {
            aprsmsg += " " + dao;
        }

        //aprsmsg.trim();
        msg.getAPRSBody()->setData(aprsmsg);
        String data(msg.encode());
        logPrintlnD(data);
#warning SPLIT
#if 0
        oled.Display("<< TX >>", data);
#else
        String splitData[5];

        splitData[0] = data.substring(0, 20);
        splitData[1] = data.substring(21, 21+20);
        splitData[2] = data.substring(41, 41+20);
        splitData[3] = data.substring(61, 61+20);
        splitData[4] = data.substring(81, 81+20);
        oled.Display("<< TX >>", splitData[0], splitData[1], splitData[2], splitData[3], splitData[4]);
        //Serial.print(data.c_str());
        //Serial.print("\n ***** :");
        //Serial.write((const uint8_t *)data.c_str(), data.length());
#endif

        if (cfg.ptt.active)
        {
            digitalWrite(cfg.ptt.io_pin, cfg.ptt.reverse ? LOW : HIGH);
            delay(cfg.ptt.start_delay);
        }

        gParams.loraIsBusy = true;
        if (LoRa.beginPacket() != 0) // Ensure the LoRa module is in RX mode.
        {
            // Header:
            LoRa.write('<');
            LoRa.write(0xFF);
            LoRa.write(0x01);
            // APRS Data:
            LoRa.write((const uint8_t *)data.c_str(), data.length());
            LoRa.endPacket();
            Serial.println("LoRa send");
        }
        else
        {
            gParams.forcePositionUpdate = true; // Try to resend on the next GPS update
            Serial.println("LoRa BUSY");
        }

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
                gParams.txInterval = std::min(cfg.smart_beacon.slow_rate, cfg.smart_beacon.fast_speed * cfg.smart_beacon.fast_rate / curr_speed) * 1000;
            }
        }
    }

    if ((cfg.debug == false) && ((millis() > 5000) && (gps.charsProcessed() < 10)))
    {
        logPrintlnE("No GPS frames detected! Try to reset the GPS Chip holding user button on startup.");
    }

#if 0
    if (gParams.loraIsBusy)
    {
        delay(100); // Slow it down
    }
    else
    {
        execSleep(100);
    }
#else
#if 0
    bool hasFix = false;
    if (gParams.gpsIsSleeping == false)
    {
        hasFix = gps.location.isValid();
        char buffer[64];

        snprintf(buffer, sizeof(buffer), "%s force: %d", (hasFix ? "FIX" : "NOFIX"), gParams.forcePositionUpdate);

        //Serial.println(hasFix ? "FIX" : "NOFIX");
        Serial.println(buffer);

        if (hasFix)
        {
            if ((millis() - gParams.gpsSleepActionTime) > 5000) // 5s up
            {
                Serial.println("GPS SUSPEND");
                gpsSuspend(true);
                gParams.gpsSleepActionTime = millis();// + 10000; // 10s
            }
        }

    }

    if (gParams.gpsIsSleeping && ((millis() - gParams.gpsSleepActionTime) > 10000)) // 10s down
    {
        Serial.println("GPS WAKEUP");
        gpsSuspend(false);
        gParams.gpsSleepActionTime = millis();
    }
#endif

    delay(200); // Slow it down
#endif
}

static void execSleep(uint32_t milliseconds)
{
    esp_sleep_enable_timer_wakeup(milliseconds * 1000U);
    // suspend peripherials


    esp_light_sleep_start();
    //esp_deep_sleep_start();

    // resume peripherials
}

static void loraTXDoneCallback()
{
    gParams.loraIsBusy = false;
}

static void buttonClickCallback()
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

static void loadConfiguration()
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

static void loraConfiguration()
{
    logPrintlnI("Set SPI pins!");
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    logPrintlnI("Set LoRa pins!");
    LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

    long freq = cfg.lora.frequencyTx;
    logPrintlnI("frequency: " + String(freq));

    if (LoRa.begin(freq) == false)
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
    LoRa.onTxDone(loraTXDoneCallback);

    logPrintlnI("LoRa init done!");
    oled.Display("INFO", "LoRa init done!", 2000);
}

static void gpsSuspend(bool suspend)
{
    SFE_UBLOX_GNSS gnss;

    if (gnss.begin(ss))
    {
        gnss.powerSaveMode(suspend);
        gParams.gpsIsSleeping = suspend;
    }
}


static void gpsConfiguration()
{
    SFE_UBLOX_GNSS gnss;

//    pinMode(GPS_RX, INPUT);
    //digitalWrite(GPS_RX, PULLUP);


    ss.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);

    if (gnss.begin(ss))
    {
        gnss.powerSaveMode(true, 5000);
        //gnss.softwareResetGNSSOnly();
        //gnss.hardReset();
        //delay(1000);
        //gnss.factoryReset();
        //delay(2000);
    }
}

static void gpsReset()
{
    SFE_UBLOX_GNSS gnss;
    uint8_t state = 0;

#if 1
    do
    {
        switch (state)
        {
            case 0:
                while(true)
                {
                    if (gnss.begin(ss))
                    {
                        oled.Display("GPS  RESET", emptyString, "Connected to GPS", 2000);
                        gnss.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only
                        gnss.saveConfiguration(); //Save the current settings to flash and BBR

                        oled.Display("GPS  RESET", emptyString, "GPS Configuration", 2000);
                        gnss.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
                        gnss.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
                        gnss.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
                        gnss.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
                        gnss.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
                        gnss.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
                        gnss.saveConfiguration(); //Save the current settings to flash and BBR
                        break;
                    }

                    oled.Display("GPS  RESET", emptyString, "Waiting for GPS", 1000);
                }

                oled.Display("GPS  RESET", emptyString, "GPS config. saved", 2000);
                state++;
                break;

            case 1:
                oled.Display("GPS  RESET", emptyString, "Hard-Reset cold start");
                gnss.hardReset();
                delay(3000);
                if (gnss.begin(ss))
                {
                    oled.Display("GPS  RESET", emptyString, "Hard-Reset SUCCESS");
                    state++;
                }
                else
                {
                    oled.Display("GPS  RESET", emptyString, "!! No Response !!", "Starting over");
                    state = 0;
                }
                break;

            case 2:
                oled.Display("GPS  RESET", emptyString, "Factory Reset");
                gnss.factoryReset();
                delay(3000); // takes more than one second... a loop to resync would be best

#if 0
                if (gnss.powerSaveMode(true) == false)
                {
                    oled.Display("GPS  RESET", emptyString, "power save failed");

                    while (true) {}
                }
                gnss.saveConfiguration();
#endif


                if (gnss.begin(ss))
                {
                    oled.Display("GPS  RESET",
                            "      SUCCESS",
                            " GPS has been reset",
                            "to factory settings.",
                            " It will take time",
                            "to acquire satellites", 5000);
                    state++;
                }
                else
                {
                    oled.Display("GPS  RESET", emptyString, "!! No Response !!", "Starting over");
                    state = 0;
                }
                break;

            case 3:
                oled.Display("GPS  RESET",
                        "       TESTING",
                        "Outputing GPS frames",
                        "  to Serial port",
                        "Power: " + String(gnss.getPowerSaveMode()),
                        "Press RESET to reboot");
                for (uint32_t c = 0; c < 300000000; c++)
                {
                    if (ss.available())
                    {
                        Serial.write(ss.read());  // print anything comes in from the GPS
                    }
                }
                state++;
                break;
        }
    } while (state < 4);
#else
    do
    {
        switch (state)
        {
            case 0:
                while(true)
                {
                    if (gnss.begin(ss))
                    {
                        oled.Display("GPS  RESET", emptyString, "Connected to GPS", 2000);
                        gnss.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only
                        gnss.saveConfiguration(); //Save the current settings to flash and BBR

                        oled.Display("GPS  RESET", emptyString, "GPS Configuration", 2000);
                        gnss.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
                        gnss.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
                        gnss.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
                        gnss.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
                        gnss.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
                        gnss.disableNMEAMessage(UBX_NMEA_TXT, COM_PORT_UART1);
                        gnss.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
                        //gnss.powerSaveMode(true);
                        gnss.saveConfiguration(); //Save the current settings to flash and BBR
                        break;
                    }

                    oled.Display("GPS  RESET", emptyString, "Waiting for GPS", 1000);
                }

                oled.Display("GPS  RESET", emptyString, "GPS config. saved", 2000);
                state++;
                break;

            case 1:
                oled.Display("GPS  RESET", emptyString, "Hard-Reset cold start");
                gnss.hardReset();
                delay(3000);
                if (gnss.begin(ss))
                {
                    oled.Display("GPS  RESET", emptyString, "Hard-Reset SUCCESS");
                    state++;
                }
                else
                {
                    oled.Display("GPS  RESET", emptyString, "!! No Response !!", "Starting over");
                    state = 0;
                }
                break;

#if 0
            case 2:
                oled.Display("GPS  RESET", emptyString, "Factory Reset");
                gnss.factoryReset();
                delay(3000); // takes more than one second... a loop to resync would be best

#if 0
                if (gnss.powerSaveMode(true) == false)
                {
                    oled.Display("GPS  RESET", emptyString, "power save failed");

                    while (true) {}
                }
                gnss.saveConfiguration();
#endif


                if (gnss.begin(ss))
                {
                    oled.Display("GPS  RESET",
                            "      SUCCESS",
                            " GPS has been reset",
                            "to factory settings.",
                            " It will take time",
                            "to acquire satellites", 5000);
                    state++;
                }
                else
                {
                    oled.Display("GPS  RESET", emptyString, "!! No Response !!", "Starting over");
                    state = 0;
                }
                break;
#endif

            case 2:
                oled.Display("GPS  RESET",
                        "       TESTING",
                        "Outputing GPS frames",
                        "  to Serial port",
                        "Power: " + String(gnss.getPowerSaveMode()),
                        "Press RESET to reboot");
                for (uint32_t c = 0; c < 300000000; c++)
                {
                    if (ss.available())
                    {
                        Serial.write(ss.read());  // print anything comes in from the GPS
                    }
                }
                state++;
                break;
        }
    } while (state < 3);
#endif
}

static void gpsCheck()
{
    SFE_UBLOX_GNSS gnss;

    while(true)
    {
        if (gnss.begin(ss))
        {
            oled.Display("GPS  CHK",
                    "       CHECKING",
                    "Power: " + String(gnss.getPowerSaveMode()),
                    "Press RESET to reboot");
            break;
        }
    }

    while (true) {}
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

static String createLatAPRS(RawDegrees lat)
{
    char str[20];

    // we like sprintf's float up-rounding.
    // but sprintf % may round to 60.00 -> 5360.00 (53° 60min is a wrong notation
    // ;)
    sprintf(str, "%02d%s%c", lat.deg, s_min_nn(lat.billionths, 0), (lat.negative ? 'S' : 'N'));
    return String(str);
}

static String createLatAPRSDAO(RawDegrees lat)
{
    // round to 4 digits and cut the last 2
    char str[20];

    // we need sprintf's float up-rounding. Must be the same principle as in
    // aprs_dao(). We cut off the string to two decimals afterwards. but sprintf %
    // may round to 60.0000 -> 5360.0000 (53° 60min is a wrong notation ;)
    sprintf(str, "%02d%s%c", lat.deg, s_min_nn(lat.billionths, 1 /* high precision */), (lat.negative ? 'S' : 'N'));
    return String(str);
}

static String createLongAPRS(RawDegrees lng)
{
    char str[20];

    sprintf(str, "%03d%s%c", lng.deg, s_min_nn(lng.billionths, 0), (lng.negative ? 'W' : 'E'));
    return String(str);
}

static String createLongAPRSDAO(RawDegrees lng)
{
    // round to 4 digits and cut the last 2
    char str[20];

    sprintf(str, "%03d%s%c", lng.deg, s_min_nn(lng.billionths, 1 /* high precision */), (lng.negative ? 'W' : 'E'));
    return String(str);
}

static String createAPRSDAO(RawDegrees lat, RawDegrees lng)
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
