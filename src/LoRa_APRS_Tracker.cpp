#include <APRS-Decoder.h>
#include <Arduino.h>
#include <LoRa.h>
#include <OneButton.h>
#include <TimeLib.h>
#include <WiFi.h>

#include "dummylogger.h"
#include "configuration.h"
#include "gps.h"
#include "display.h"
#include "pins.h"
#include "power_management.h"
#include "Deg2DDMMMM.h"


#define AWAKE_TIME_MS 5000
#define SLEEP_TIME_MS 10000

static Configuration     cfg;
static OLEDDisplay       oled;
static PowerManagement   pm;
static OneButton         userBtn(BUTTON_PIN, true, true);
static HardwareSerial    ss(1);
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
            gpsIsSleeping(false),
            gpsSleepActionTime(0),
            gpsHadFix(false),
            lastUpdateTime(millis()),
            gpsFixTime(0),
            gpsHasToSetTime(true),
            awakeButtonTime(0),
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
        bool           gpsIsSleeping;
        unsigned long  gpsSleepActionTime;
        bool           gpsHadFix;
        unsigned long  lastUpdateTime;
        unsigned long  gpsFixTime;
        bool           gpsHasToSetTime;
        unsigned long  awakeButtonTime;
#ifdef TTGO_T_Beam_V1_0
        unsigned long  batteryLastCheckTime;
#endif

    private:
        uint32_t       m_displayTimeout;
        unsigned long  m_displayLastTimeout;
        bool           m_displayTimeoutEnabled;
};


static GlobalParameters  gParams;


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
        DlogPrintlnE("You have to change your settings in 'data/tracker.json' and "
                "upload it via \"Upload File System image\"!");
        oled.Display("ERROR", "You have to change your settings in 'data/tracker.json' and "
                "upload it via \"Upload File System image\"!");

        while (true) {}
    }
}

static void loraInit()
{
    DlogPrintlnI("Set SPI pins!");
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    DlogPrintlnI("Set LoRa pins!");
    LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

    long freq = cfg.lora.frequencyTx;
    DlogPrintlnI("frequency: " + String(freq));

    if (LoRa.begin(freq) == false)
    {
        DlogPrintlnE("Starting LoRa failed!");
        oled.Display("ERROR", "Starting LoRa failed!");

        while (true) {}
    }

    LoRa.setSpreadingFactor(cfg.lora.spreadingFactor);
    LoRa.setSignalBandwidth(cfg.lora.signalBandwidth);
    LoRa.setCodingRate4(cfg.lora.codingRate4);
    LoRa.enableCrc();

    LoRa.setTxPower(cfg.lora.power);
    //LoRa.onTxDone(loraTXDoneCallback);

    LoRa.sleep();

    DlogPrintlnI("LoRa init done!");
    oled.Display("INFO", "LoRa init done!", 2000);
}

static String padWithZeros(unsigned int number, unsigned int width)
{
    char buffer[64];

    snprintf(buffer, sizeof(buffer), "%.*d", width, number);

    return String(buffer);
}

static String formatToDateString(time_t t)
{
    return String(padWithZeros(day(t), 2) + "." + padWithZeros(month(t), 2) + "." + padWithZeros(year(t), 4));
}

static String formatToTimeString(time_t t)
{
    return String(padWithZeros(hour(t), 2) + "." + padWithZeros(minute(t), 2) + "." + padWithZeros(second(t), 2));
}

static String getOnOff(bool state)
{
    return String(state ? "On" : "Off");
}

static esp_sleep_wakeup_cause_t execLightSleepIfPossible(uint64_t sleepMS)
{
    // ESP sleeping
    if ((cfg.display_timeout == 0) || ((cfg.display_timeout > 0) && (oled.IsActivated() == false)))
    {
        uint64_t sleepUS = sleepMS * 1000LL;

        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON); // RTC stays ON

#if 0 // Disabled: it trigger false interrupts with unplugged cable.
#warning DISABLE ME
        gpio_wakeup_enable((gpio_num_t)SERIAL0_RX_GPIO, GPIO_INTR_LOW_LEVEL);
#endif

        gpio_wakeup_enable((gpio_num_t)BUTTON_PIN, GPIO_INTR_LOW_LEVEL);

        assert(esp_sleep_enable_gpio_wakeup() == ESP_OK);
        assert(esp_sleep_enable_timer_wakeup(sleepUS) == ESP_OK);
        assert(esp_light_sleep_start() == ESP_OK);

        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

        return cause;
    }

    return ESP_SLEEP_WAKEUP_UNDEFINED;
}

// cppcheck-suppress unusedFunction
void setup()
{
    // Save some power, switch from 240MHz to 80MHz, power consummption from 66.8mA to 33.2mA
    //setCpuFrequencyMhz(80);
    //setCpuFrequencyMhz(160);

#if defined(LOGGER_ENABLED)
    // Log only Errors
    Logger::instance().setDebugLevel(Logger::DEBUG_LEVEL_ERROR);
#endif

    Serial.begin(115200);

    gParams.DisableDisplayTimeout(); // Keep the OLED screen ON

#ifdef TTGO_T_Beam_V1_0
    Wire.begin(SDA, SCL);
    if (!pm.begin(Wire))
    {
        DlogPrintlnI("AXP192 init done!");
    }
    else
    {
        DlogPrintlnE("AXP192 init failed!");
    }

    pm.GPSActivate();
    pm.LoRaActivate();
    pm.OLEDActivate();
    pm.MeasurementsActivate();
    delay(500);
#endif

    delay(500);
    DlogPrintlnI("LoRa APRS Tracker by OE5BPA (Peter Buchegger)");
    oled.Init();

    oled.Display("OE5BPA", "LoRa APRS Tracker", "by Peter Buchegger", emptyString, "Mods: Daniel, F1RMB", "               v0.201");

    loadConfiguration();

    if (gps.Initialize(ss) == false)
    {
        oled.Display("GPS INIT", "Initialization failed");

#ifdef TTGO_T_Beam_V1_0 // Execute a factory reset to try to solve the communication problem
        pm.GPSDeactivate();
        delay(2000);
        ESP.restart();
#endif
        while (true) { }
    }

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

    DlogPrintlnI("Smart Beacon is " + getOnOff(cfg.smart_beacon.active));
    oled.Display("INFO", "Smart Beacon is " + getOnOff(cfg.smart_beacon.active), 1000);
    DlogPrintlnI("setup done...");

#if 0
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
#endif

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
#endif

#if 0
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
        }
#endif

        double currentLat         = NAN;
        double currentLong        = NAN;
        double currentHeading     = NAN;
        double currentAltInFeet   = NAN;
        double currentSpeedKnot   = NAN;
        bool   timeIsValid        = false;
        bool   gpsStillHasToSleep = gps.StillHasToSleep();
        bool   gpsHasFix          = (gpsStillHasToSleep ? false : (gps.GetPVT() && gps.HasFix()));

        // GPS fix
        //
        // GPS acquired a fix, start to countdown for power saving state
        if ((gpsStillHasToSleep == false) && gpsHasFix && (gParams.gpsFixTime == 0))
        {
            gParams.gpsFixTime = millis();
        }

        // Keep track of the GPS fix, regardless of the GPS PowerSave status (sleeping or not).
        if ((gParams.gpsHadFix == false) && gpsHasFix)
        {
            gParams.gpsHadFix = true;
        }
        else if (gParams.gpsHadFix && ((gpsStillHasToSleep == false) && (gpsHasFix == false)))
        {
            gParams.gpsHadFix = false;
        }

        if (gpsHasFix)
        {
            currentLat       = gps.GetLatitude();
            currentLong      = gps.GetLongitude();
            currentHeading   = gps.GetHeading();
            currentAltInFeet = gps.GetAltitudeFT();
            currentSpeedKnot = gps.GetSpeedKT();

            // Fix has been lost, set the RTC on the next fix
            if ((gParams.gpsHasToSetTime == false) && (gpsStillHasToSleep == false) && (gpsHasFix == false))
            {
                gParams.gpsHasToSetTime = true;
            }

#if 0
            double altitude  = gps.GetAltitude();
            double speedms   = gps.GetSpeedMPS();

            char buf[128];
            sprintf(buf, "Lat: %f  Long: %f  Heading: %f  Alt(Ft): %f/%f  Speed(knot): %f/%f",
                    currentLat, currentLong, currentHeading,
                    currentAltInFeet, altitude, currentSpeedKnot, speedms);
            Serial.println(buf);
#endif

            struct tm dt;
            if ((timeIsValid = gps.GetDateAndTime(dt)))
            {
                if (gParams.gpsHasToSetTime) // We need to adjust the RTC
                {
                    gParams.gpsHasToSetTime = false;
                    setTime(mktime(&dt)); // Update the RTC
                }

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
                gParams.batteryVoltage       = String(pm.getBatteryVoltage(), 2);
                gParams.batteryChargeCurrent = String(pm.getBatteryChargeDischargeCurrent(), 0);
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
            Deg2DDMMMMPosition pLat, pLong;
            char               latBuf[32];
            char               longBuf[32];

            gParams.sendPositionUpdate = false;
            gParams.nextBeaconTimeStamp = now() + (cfg.smart_beacon.active ? cfg.smart_beacon.slow_rate : (cfg.beacon.timeout * SECS_PER_MIN));

            msgStr.setSource(cfg.callsign);
            msgStr.setDestination("APLT00-1");

            // Lat/Long
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


            String altStr;
            int    altValue = std::max(-99999, std::min(999999, (int)currentAltInFeet));

            altStr = String("/A=") + (altValue < 0 ? "-" : "") + padWithZeros((unsigned int)abs(altValue), ((altValue < 0) ? 5 : 6));


            String courseAndSpeedStr;
            int    speedValue = std::max(0, std::min(999, (int)currentSpeedKnot));

            if (gParams.speedZeroSent < 3)
            {
                String speedStr    = padWithZeros(speedValue, 3);
                int    courseValue = std::max(0, std::min(360, (int)currentHeading));

                /* course in between 1..360 due to aprs spec */
                if (courseValue == 0)
                {
                    courseValue = 360;
                }

                String courseStr(padWithZeros(courseValue, 3));

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
            DlogPrintlnD(data);

#if 0
            oled.Display("<< TX >>", data);
#else
#warning split for screen
            String splitData[5];

            splitData[0] = data.substring(0, 20);
            splitData[1] = data.substring(21, 21+20);
            splitData[2] = data.substring(41, 41+20);
            splitData[3] = data.substring(61, 61+20);
            splitData[4] = data.substring(81, 81+20);
            oled.Display("<< TX >>", splitData[0], splitData[1], splitData[2], splitData[3], splitData[4]);
#endif

            if (cfg.ptt.active)
            {
                digitalWrite(cfg.ptt.io_pin, (cfg.ptt.reverse ? LOW : HIGH));
                delay(cfg.ptt.start_delay);
            }

            if (LoRa.beginPacket() != 0) // Ensure the LoRa module is not transmiting
            {
                LoRa.idle();

                // Header:
                LoRa.write('<');
                LoRa.write(0xFF);
                LoRa.write(0x01);
                // APRS Data:
                LoRa.write((const uint8_t *)data.c_str(), data.length());
                LoRa.endPacket(); // Send SYNC

                LoRa.sleep();

                //Serial.print("=====> '");
                //Serial.println(data.c_str());
            }
            else
            {
                gParams.sendPositionUpdate = true; // Try to resend on the next run
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

        oled.Display(cfg.callsign,
                formatToDateString(now()) + " " + formatToTimeString(now()),
                String("Sats: ") + (gParams.gpsHadFix ? String(gps.GetSatellites()) : "-") + " HDOP: " + (gParams.gpsHadFix ? String(gps.GetHDOP()) : "--.--"),
                String("Nxt Bcn: ") + (gParams.gpsHadFix ? (cfg.smart_beacon.active ? "~" : "") + formatToTimeString(gParams.nextBeaconTimeStamp) : "--:--:--"),
                (gParams.batteryIsConnected ? (String("Bat: ") + gParams.batteryVoltage + "V, " + gParams.batteryChargeCurrent + "mA") : "Powered via USB"),
                String("Smart Beacon: " + getOnOff(cfg.smart_beacon.active)));
#if 0
        //if (gParams.gpsHadFix)
        {
            Serial.println(cfg.callsign);
            Serial.println(formatToDateString(now()) + " " + formatToTimeString(now()));
            Serial.println(String("Sats: ") + String(gpsHasFix ? String(gps.GetSatellites()) : "-") + " HDOP: " + (gParams.gpsHadFix ? String(gps.GetHDOP()) : "--.--"));
            Serial.println(String("Nxt Bcn: ") + (gParams.gpsHadFix ? (cfg.smart_beacon.active ? "~" : "") + formatToTimeString(gParams.nextBeaconTimeStamp) : "--:--:--"));
            Serial.println((gParams.batteryIsConnected ? (String("Bat: ") + gParams.batteryVoltage + "V, " + gParams.batteryChargeCurrent + "mA") : "Powered via USB"));
            Serial.println(String("Smart Beacon: " + getOnOff(cfg.smart_beacon.active)));
        }
#endif

        if (timeIsValid)
        {
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


        // GPS Sleep/Awake
        if ((gpsStillHasToSleep == false) && gpsHasFix && ((gParams.gpsFixTime >= 0) && ((millis() - gParams.gpsFixTime) > AWAKE_TIME_MS)))
        {
            bool res = gps.SetLowPower(true, SLEEP_TIME_MS);

            if (res)
            {
                gParams.gpsFixTime = 0;
            }
        }
        else if (gps.IsSleeping() && (gps.StillHasToSleep() == false))
        {
            bool res = gps.SetLowPower(false, 0);

            if (res)
            {
                gParams.gpsFixTime = 0;
            }
        }
    }

#if 1

    // EPS Light Sleep
    if (gParams.awakeButtonTime == 0 || ((millis() - gParams.awakeButtonTime) > 5000)) // Wait 5s after awaken from the button before going to sleep.
    {
        esp_sleep_wakeup_cause_t cause;
        uint32_t gpsRemainingSleepTime = gps.GetRemainingSleepTime();

        // If the display has a timeout value set, sleep as long as the GPS, otherwise for 500ms (clock accuracy)
        cause = execLightSleepIfPossible(((cfg.display_timeout > 0) && (gpsRemainingSleepTime > 500)) ? gpsRemainingSleepTime : 500);

        switch (cause)
        {
            case ESP_SLEEP_WAKEUP_TIMER:
                gParams.awakeButtonTime = 0;
                break;

            case ESP_SLEEP_WAKEUP_UART:
                gParams.awakeButtonTime = millis();
                break;

            case ESP_SLEEP_WAKEUP_UNDEFINED:
            default:
            {
                bool pressed = !digitalRead(BUTTON_PIN);

                gParams.awakeButtonTime = millis();

                if (pressed)
                {
                    userBtn.tick(true);
                }
            }
            break;

        }

    }
#endif

}

