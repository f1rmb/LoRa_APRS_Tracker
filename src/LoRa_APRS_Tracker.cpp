#include <APRS-Decoder.h>
#include <Arduino.h>
#include <LoRa.h>
#include <OneButton.h>
#include <TimeLib.h>
#include <WiFi.h>
#include <logger.h>

#include "configuration.h"
#include "gps.h"
#include "display.h"
#include "pins.h"
#include "power_management.h"
#include "Deg2DDMMMM.h"

static Configuration     cfg;
static OLEDDisplay       oled;
static PowerManagement   pm;
static OneButton         userBtn(BUTTON_PIN, true, true);
static HardwareSerial    ss(1);
//static TinyGPSPlus     gps;
static GPSDevice         gps;



struct GlobalParameters
{
        GlobalParameters() :
            sendPositionUpdate(true),
            nextBeaconTimeStamp(now()),
            currentHeading(0.0),
            previousHeading(0.0),
            rateLimitMessageText(0),
            lastTxLat(0.0),
            lastTxLong(0.0),
            lastTxDistance(0),
            txInterval(60000L), // Initial 60 secs internal
            lastTxTime(millis()),
            speedZeroSent(0),
            batteryIsConnected(false),
            batteryVoltage(""),
            batteryChargeCurrent(""),
            loraIsBusy(false),
            gpsIsSleeping(false),
            gpsSleepActionTime(0),
            lastUpdateTime(millis()),
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

        bool           sendPositionUpdate;
        time_t         nextBeaconTimeStamp;
        double         currentHeading;
        double         previousHeading;
        uint32_t       rateLimitMessageText;
        double         lastTxLat;
        double         lastTxLong;
        int32_t        lastTxDistance;
        uint32_t       txInterval;
        unsigned long  lastTxTime;
        int            speedZeroSent;
        bool           batteryIsConnected;
        String         batteryVoltage;
        String         batteryChargeCurrent;
        bool           loraIsBusy;
        bool           gpsIsSleeping;
        unsigned long  gpsSleepActionTime;
        unsigned long  lastUpdateTime;
#ifdef TTGO_T_Beam_V1_0
        unsigned long  batteryLastCheckTime;
#endif

    private:
        uint32_t       m_displayTimeout;
        unsigned long  m_displayLastTimeout;
        bool           m_displayTimeoutEnabled;
};


static GlobalParameters  gParams;

static double batCurrent[10];
static int batCurrentIndex = 0;


// Functions prototypes
static void buttonClickCallback();
static void loraTXDoneCallback();
static void loadConfiguration();
static void loraInit();
static String formatToDateString(time_t t);
static String formatToTimeString(time_t t);
static String getOnOff(bool state);
static String PadWithZeros(unsigned int number, unsigned int width);


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

    if (gps.Initialize((cfg.debug ? Serial : ss), (cfg.debug ? false : true)) == false)
    {
        oled.Display("GPS INIT", "Initialization failed");
        while (true) { }
    }

    //delay(2000);

    loraInit();

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

    logPrintlnI("Smart Beacon is " + getOnOff(cfg.smart_beacon.active));
    oled.Display("INFO", "Smart Beacon is " + getOnOff(cfg.smart_beacon.active), 1000);
    logPrintlnI("setup done...");

    delay(500);
    userBtn.tick();

    if (userBtn.isIdle() == false)
    {
        //gpsReset();
        //gpsCheck();
        oled.Display("GPS  RESET",
                "Resetting...");

        gps.FactoryReset();

        oled.Display("GPS  RESET",
                "      SUCCESS",
                " GPS has been reset",
                "to factory settings.",
                " It will take time",
                "to acquire satellites", 5000);
    }

    oled.Display("INFO", "Waiting for Position");

    gParams.ResetDisplayTimeout(); // Enable OLED timeout
}

// cppcheck-suppress unusedFunction
void loop()
{
    userBtn.tick();
    gParams.DisplayTick();

    if ((millis() - gParams.lastUpdateTime) >= 1000) // Update each 1 second
    {
        gParams.lastUpdateTime = millis();
#if 0
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
#else
        if (gps.HasData())
        {

        }
#endif

        while (Serial.available() > 0)
        {
            char c = Serial.read();

            if (c == 'f' || c == 'F')
            {
                gParams.sendPositionUpdate = true;
            }
            else if (c == 'R')
            {
                Serial.println("GPS Factory Reset");
                gps.FactoryReset();
            }
            else if (c == 'r')
            {
                ESP.restart();
            }
            else if (c == 'i' || c == 'I')
            {
                Serial.println("===================");
                Serial.print("Index: ");
                Serial.println(batCurrentIndex);
                for (size_t i = 0; i < (sizeof(batCurrent) / sizeof(batCurrent[0])); i++)
                {
                    char buf[32];

                    sprintf(buf, "I: %f", batCurrent[i]);
                    Serial.println(buf);
                }
                Serial.println("===================\n");
            }
        }



        bool gpsHasFix          = (gps.HasFix() && gps.GetPVT());
        double currentLat       = NAN;
        double currentLong      = NAN;
        double currentHeading   = NAN;
        double currentAltInFeet = NAN;
        double currentSpeedKnot = NAN;


        //if (gps.time.isValid())
        if (gpsHasFix)
        {
            currentLat       = gps.GetLatitude();
            currentLong      = gps.GetLongitude();
            currentHeading   = gps.GetHeading();
            currentAltInFeet = gps.GetAltitudeFT();
            currentSpeedKnot = gps.GetSpeedKT();
            double altitude  = gps.GetAltitude();
            double speedms   = gps.GetSpeedMPS();

            char buf[128];
            sprintf(buf, "Lat: %f  Long: %f  Heading: %f  Alt(Ft): %f/%f  Speed(knot): %f/%f",
                    currentLat, currentLong, currentHeading,
                    currentAltInFeet, altitude, currentSpeedKnot, speedms);
            Serial.println(buf);


            struct tm dt;
            if (gps.GetDateAndTime(dt))
            {
                setTime(mktime(&dt)); // Update the RTC

                if (gParams.nextBeaconTimeStamp <= now())
                {
                    gParams.sendPositionUpdate = true;

                    if (cfg.smart_beacon.active)
                    {
                        gParams.currentHeading = currentHeading;
                        // enforce message text on slowest cfg.smart_beacon.slow_rate
                        gParams.rateLimitMessageText = 0;
                    }
                    else
                    {
                        // enforce message text every n's cfg.beacon.timeout frame
                        if ((cfg.beacon.timeout * gParams.rateLimitMessageText) > 30)
                        {
                            gParams.rateLimitMessageText = 0;
                        }
                    }
                }
            }
        }

#ifdef TTGO_T_Beam_V1_0
        unsigned long m = millis();
        // Update the battery every 60 seconds, that's way enough
        if ((gParams.batteryLastCheckTime == 0) || ((m - gParams.batteryLastCheckTime) >= 60000))
        {
            gParams.batteryIsConnected = pm.isBatteryConnected();
            gParams.batteryLastCheckTime = m;

            if (gParams.batteryIsConnected)
            {
                double I = pm.getBatteryChargeDischargeCurrent();

                batCurrent[batCurrentIndex] = I;

                gParams.batteryVoltage       = String(pm.getBatteryVoltage(), 2);
                gParams.batteryChargeCurrent = String(I, 0);

                batCurrentIndex = (batCurrentIndex + 1) % 10;
            }
        }
#endif

        if ((gParams.sendPositionUpdate == false) && gpsHasFix && cfg.smart_beacon.active)
        {
            unsigned long lastTx = (millis() - gParams.lastTxTime);

            gParams.currentHeading = currentHeading;
            gParams.lastTxDistance = int32_t(gps.DistanceBetweenTwoCoords(currentLat, currentLong, gParams.lastTxLat, gParams.lastTxLong));

            //TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.long(), gParams.lastTxLat, gParams.lastTxLng);

            if (lastTx >= gParams.txInterval)
            {
                // Trigger Tx Tracker when Tx interval is reach
                // Will not Tx if stationary bcos speed < 5 and lastTxDistance < 20
                if (gParams.lastTxDistance > 20)
                {
                    gParams.sendPositionUpdate = true;
                }
            }

            if (gParams.sendPositionUpdate == false)
            {
                // Get headings and heading delta
                double headingDelta = fabs(gParams.previousHeading - gParams.currentHeading);

                if (lastTx > (cfg.smart_beacon.min_bcn * 1000))
                {
                    // Check for heading more than 25 degrees
                    if ((headingDelta > cfg.smart_beacon.turn_min) && (gParams.lastTxDistance > cfg.smart_beacon.min_tx_dist))
                    {
                        gParams.sendPositionUpdate = true;
                    }
                }
            }
        }

        // Time to send an APRS frame
        if (gParams.sendPositionUpdate && gpsHasFix)
        {
            APRSMessage        msgStr;

            gParams.sendPositionUpdate = false;
            gParams.nextBeaconTimeStamp = now() + (cfg.smart_beacon.active ? cfg.smart_beacon.slow_rate : (cfg.beacon.timeout * SECS_PER_MIN));

            msgStr.setSource(cfg.callsign);
            msgStr.setDestination("APLT00-1");

#if 1
            Deg2DDMMMMPosition pLat, pLong;
            char               latBuf[32];
            char               longBuf[32];

            Deg2DDMMMM::Convert(pLat, currentLat, cfg.enhance_precision);
            Deg2DDMMMM::Convert(pLong, currentLong, cfg.enhance_precision);

            String             latStr(Deg2DDMMMM::Format(latBuf, pLat));
            String             longStr(Deg2DDMMMM::Format(longBuf, pLong));
            String             daoStr;

            if (cfg.enhance_precision)
            {
                char daoBuf[16];

                daoStr = String(Deg2DDMMMM::DAO(daoBuf, pLat, pLong));
            }
#endif

            String altStr;
            int    altValue = std::max(-99999, std::min(999999, (int)currentAltInFeet));

            altStr = String("/A=") + (altValue < 0 ? "-" : "") + PadWithZeros((unsigned int)abs(altValue), ((altValue < 0) ? 5 : 6));


            String courseAndSpeedStr;
            int    speedValue = std::max(0, std::min(999, (int)currentSpeedKnot));

            if (gParams.speedZeroSent < 3)
            {
                String speedStr    = PadWithZeros(speedValue, 3);
                int    courseValue = std::max(0, std::min(360, (int)currentHeading));

                /* course in between 1..360 due to aprs spec */
                if (courseValue == 0)
                {
                    courseValue = 360;
                }

                String courseStr(PadWithZeros(courseValue, 3));

                courseAndSpeedStr = courseStr + "/" + speedStr;
            }

            if (speedValue == 0)
            {
                /* speed is 0.
                 * we send 3 packets with speed zero (so our friends know we stand still).
                 * After that, we save airtime by not sending speed/course 000/000.
                 * Btw, even if speed we really do not move, measured course is changeing
                 * (-> no useful / even wrong info)
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


            String aprsmsg("!" + latStr + cfg.beacon.overlay + longStr + cfg.beacon.symbol + courseAndSpeedStr + altStr);

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


            if (cfg.enhance_precision && (daoStr.length() > 0))
            {
                aprsmsg += " " + daoStr;
            }

            msgStr.getBody()->setData(aprsmsg);
            String data(msgStr.encode());
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
                digitalWrite(cfg.ptt.io_pin, (cfg.ptt.reverse ? LOW : HIGH));
                delay(cfg.ptt.start_delay);
            }

#if 1
            if (LoRa.beginPacket() != 0) // Ensure the LoRa module is in RX mode.
            {
                gParams.loraIsBusy = true;

                // Header:
                LoRa.write('<');
                LoRa.write(0xFF);
                LoRa.write(0x01);
                // APRS Data:
                LoRa.write((const uint8_t *)data.c_str(), data.length());
                LoRa.endPacket();

                Serial.println("LoRa sent");
            }
            else
            {
                gParams.sendPositionUpdate = true; // Try to resend on the next GPS update
                Serial.println("LoRa IS BUSY");
            }
#endif
            if (gParams.loraIsBusy)
            {
                Serial.print("=====> '");
                Serial.println(data.c_str());
            }

            if (cfg.smart_beacon.active)
            {
                gParams.lastTxLat       = currentLat;
                gParams.lastTxLong      = currentLong;
                gParams.previousHeading = gParams.currentHeading;
                gParams.lastTxDistance  = 0;
                gParams.lastTxTime      = millis();
            }

            if (cfg.ptt.active)
            {
                delay(cfg.ptt.end_delay);
                digitalWrite(cfg.ptt.io_pin, (cfg.ptt.reverse ? HIGH : LOW));
            }
        }

        //if (gps_time_update)
        {
            oled.Display(cfg.callsign,
                    formatToDateString(now()) + " " + formatToTimeString(now()),
                    String("Sats: ") + (gpsHasFix ? String(gps.GetSatellites()) : "-") + " HDOP: " + (gpsHasFix ? String(gps.GetHDOP()) : "--.--"),
                    String("Nxt Bcn: ") + (gpsHasFix ? (cfg.smart_beacon.active ? "~" : "") + formatToTimeString(gParams.nextBeaconTimeStamp) : "--:--:--"),
                    (gParams.batteryIsConnected ? (String("Bat: ") + gParams.batteryVoltage + "V, " + gParams.batteryChargeCurrent + "mA") : "Powered via USB"),
                    String("Smart Beacon: " + getOnOff(cfg.smart_beacon.active)));

            Serial.println(cfg.callsign);
            Serial.println(formatToDateString(now()) + " " + formatToTimeString(now()));
            Serial.println(String("Sats: ") + String(gpsHasFix ? String(gps.GetSatellites()) : "-") + " HDOP: " + (gpsHasFix ? String(gps.GetHDOP()) : "--.--"));
            Serial.println(String("Nxt Bcn: ") + (gpsHasFix ? (cfg.smart_beacon.active ? "~" : "") + formatToTimeString(gParams.nextBeaconTimeStamp) : "--:--:--"));
            Serial.println((gParams.batteryIsConnected ? (String("Bat: ") + gParams.batteryVoltage + "V, " + gParams.batteryChargeCurrent + "mA") : "Powered via USB"));
            Serial.println(String("Smart Beacon: " + getOnOff(cfg.smart_beacon.active)));

            if (cfg.smart_beacon.active)
            {
                // Change the Tx internal based on the current speed
                int currentSpeed = (int)gps.GetSpeedKPH();

                if (currentSpeed < cfg.smart_beacon.slow_speed)
                {
                    gParams.txInterval = (cfg.smart_beacon.slow_rate * 1000);
                }
                else if (currentSpeed > cfg.smart_beacon.fast_speed)
                {
                    gParams.txInterval = (cfg.smart_beacon.fast_rate * 1000);
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
                    gParams.txInterval = std::min(cfg.smart_beacon.slow_rate, cfg.smart_beacon.fast_speed * cfg.smart_beacon.fast_rate / currentSpeed) * 1000;
                }
            }
        }

#if 0
        if ((cfg.debug == false) && ((millis() > 5000) && (gps.charsProcessed() < 10)))
        {
            logPrintlnE("No GPS frames detected! Try to reset the GPS Chip holding user button on startup.");
        }
#endif

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

            snprintf(buffer, sizeof(buffer), "%s force: %d", (hasFix ? "FIX" : "NOFIX"), gParams.sendPositionUpdate);

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
        //    delay(200); // Slow it down
#endif
    }
}

#if 0
static void execSleep(uint32_t milliseconds)
{
    esp_sleep_enable_timer_wakeup(milliseconds * 1000U);
    // suspend peripherials


    esp_light_sleep_start();
    //esp_deep_sleep_start();

    // resume peripherials
}
#endif

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
        gParams.sendPositionUpdate = true;
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

static void loraInit()
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

static String formatToDateString(time_t t)
{
    return String(PadWithZeros(day(t), 2) + "." + PadWithZeros(month(t), 2) + "." + PadWithZeros(year(t), 4));
}

static String formatToTimeString(time_t t)
{
    return String(PadWithZeros(hour(t), 2) + "." + PadWithZeros(minute(t), 2) + "." + PadWithZeros(second(t), 2));
}

static String getOnOff(bool state)
{
    return String(state ? "On" : "Off");
}

static String PadWithZeros(unsigned int number, unsigned int width)
{
    char buffer[64];

    snprintf(buffer, sizeof(buffer), "%.*d", width, number);

    return String(buffer);
}
