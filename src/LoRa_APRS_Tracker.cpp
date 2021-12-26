#include <APRS-Decoder.h>
#include <Arduino.h>
#include <LoRa.h>
#include <OneButton.h>
#include <TimeLib.h>
#include <WiFi.h>

#include <thread>
#include <chrono>

#include "dummylogger.h"
#include "configuration.h"
#include "gps.h"
#include "display.h"
#include "pins.h"
#include "power_management.h"
#include "Deg2DDMMMM.h"


// Function prototype
static void buttonThread();


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
    private:
        struct GPSPosHDOP
        {
            GPSPosHDOP()
            {
                Reset();
            }
            ~GPSPosHDOP() { }

            void Reset()
            {
                latitude = NAN;
                longitude = NAN;
                hdop = 99.99;
                satellites = 0;
            }

            bool PositionIsValid()
            {
                return ((isnan(latitude) == false) && (isnan(longitude) == false));
            }

            double latitude;
            double longitude;
            double hdop;
            uint8_t satellites;
        };

    public:
        enum BUTTON_CLICKED
        {
            BUTTON_CLICKED_NONE  = 0,
            BUTTON_CLICKED_ONCE  = 1,
            BUTTON_CLICKED_TWICE = 2,
            BUTTON_CLICKED_MULTI = 3
        };

    public:
        GlobalParameters() :
            hasStarted(false),
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
            lightSleepExitTime(0),
            awakenTimePeriod(0),
            btnInterrupts(0),
            btnClicks(BUTTON_CLICKED_NONE),
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
            if (oled.IsActivated() == false)
            {
                oled.Activate(true);
            }

            m_displayTimeoutEnabled = (m_displayTimeout > 0 ? true : false);
            m_displayLastTimeout = millis();
        }

        void DisableDisplayTimeout()
        {
            m_displayTimeoutEnabled = false;
        }

        void DisplayTick()
        {
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
        }

        bool           hasStarted;
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
        unsigned long  lightSleepExitTime;
        unsigned long  awakenTimePeriod;
        volatile int   btnInterrupts;
        BUTTON_CLICKED btnClicks;
#ifdef TTGO_T_Beam_V1_0
        unsigned long  batteryLastCheckTime;
#endif
        GPSPosHDOP     lastValidGPS;

    private:
        uint32_t       m_displayTimeout;
        unsigned long  m_displayLastTimeout;
        bool           m_displayTimeoutEnabled;
};

static GlobalParameters gParams;

std::thread counter_loop_thread(buttonThread);

// OneButton's callbacks (not doing much, as they are called withing a thread)
static void buttonClickCallback()
{
    gParams.btnClicks = GlobalParameters::BUTTON_CLICKED_ONCE;
}

static void buttonDoubleClickCallback()
{
    gParams.btnClicks = GlobalParameters::BUTTON_CLICKED_TWICE;
}

static void buttonMultiPressCallback()
{
    gParams.btnClicks = GlobalParameters::BUTTON_CLICKED_MULTI;
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
    LoRa.sleep();

    DlogPrintlnI("LoRa init done!");
    oled.Display("INFO", "LoRa init done!", 2000);
}

// WARNING: don't use this one in *printf()
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

static esp_sleep_wakeup_cause_t execLightSleep(uint64_t sleepMS)
{
    uint64_t sleepUS = sleepMS * 1000ULL;

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

// Worker thread that handles OneButton::click() calls
static void buttonThread()
{
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (gParams.hasStarted)
        {
            userBtn.tick();
        }
    }
}

// cppcheck-suppress unusedFunction
void setup()
{
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

    oled.Display("OE5BPA", "LoRa APRS Tracker", "by Peter Buchegger", emptyString, "Mods: Daniel, F1RMB", "               v0.400");

    loadConfiguration();

    if (gps.Initialize(ss) == false)
    {
        oled.Display("GPS INIT", "Initialization failed");

#ifdef TTGO_T_Beam_V1_0 // Power cycle the GPS module
        pm.GPSDeactivate();
        delay(2000);
        ESP.restart(); // Reboot
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

    // OneButton config
    userBtn.attachClick(buttonClickCallback);
    userBtn.attachDoubleClick(buttonDoubleClickCallback);
    userBtn.attachMultiClick(buttonMultiPressCallback);
    attachInterrupt(BUTTON_PIN, [] { gParams.btnInterrupts++; }, FALLING);

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

    oled.Display("INFO", "Running...");

    gParams.ResetDisplayTimeout(); // Enable OLED timeout
    gParams.hasStarted = true; // main loop will start, unlock the userButton thread
}

// cppcheck-suppress unusedFunction
void loop()
{
    gParams.DisplayTick();

    if ((millis() - gParams.lastUpdateTime) >= 1000) // Update each 1 second
    {
        gParams.lastUpdateTime = millis();
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

        //
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
            //Serial.println("GOT FIX");
            gParams.gpsHadFix = true;
        }
        else if (gParams.gpsHadFix && ((gpsStillHasToSleep == false) && (gpsHasFix == false)))
        {
            //Serial.println("LOST FIX");
            gParams.gpsHadFix = false;

            gParams.lastValidGPS.Reset();
        }

        if (gpsHasFix)
        {
            gParams.lastValidGPS.latitude  = currentLat  = gps.GetLatitude();
            gParams.lastValidGPS.longitude = currentLong = gps.GetLongitude();
            gParams.lastValidGPS.hdop = gps.GetHDOP();
            gParams.lastValidGPS.satellites = gps.GetSatellites();
            currentHeading   = gps.GetHeading();
            currentAltInFeet = gps.GetAltitudeFT();
            currentSpeedKnot = gps.GetSpeedKT();

            // Check GPS clock, handle beaconing based on time (+ heading when smart beaconing is enabled)
            struct tm dt;
            if ((timeIsValid = gps.GetDateAndTime(dt)))
            {
                setTime(mktime(&dt)); // Update the RTC

                if (now() >= gParams.nextBeaconTimeStamp)
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

        // Smart beaconing, with GPS Fix
        if ((gParams.sendPositionUpdate == false) && gpsHasFix && cfg.smart_beacon.active)
        {
            uint32_t lastTx = (millis() - gParams.lastTxTime);

            gParams.currentHeading = currentHeading;
            gParams.lastTxDistance = int32_t(gps.DistanceBetweenTwoCoords(currentLat, currentLong, gParams.lastTxLat, gParams.lastTxLong));

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
                    // Check for heading more than config's **turn_min** degrees
                    if ((headingDelta > double(cfg.smart_beacon.turn_min)) && (gParams.lastTxDistance > cfg.smart_beacon.min_tx_dist))
                    {
                        gParams.sendPositionUpdate = true;
                    }
                }
            }
        }

        // Battery reading
#ifdef TTGO_T_Beam_V1_0
        // Update the battery on then first iteration, or just before transmitting, or every 60 seconds as it's way enough
        if ((gParams.sendPositionUpdate && gpsHasFix) ||
                ((gParams.batteryLastCheckTime == 0) || ((millis() - gParams.batteryLastCheckTime) >= 60000)))
        {
            gParams.batteryIsConnected = pm.isBatteryConnected();
            gParams.batteryLastCheckTime = millis();

            if (gParams.batteryIsConnected)
            {
                gParams.batteryVoltage       = String(pm.getBatteryVoltage(), 2);
                gParams.batteryChargeCurrent = String(pm.getBatteryChargeDischargeCurrent(), 0);
            }
        }
#endif

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
            int    altValue = std::max(-99999, std::min(999999, int(currentAltInFeet)));

            altStr = String("/A=") + (altValue < 0 ? "-" : "") + padWithZeros((unsigned int)abs(altValue), ((altValue < 0) ? 5 : 6));


            String courseAndSpeedStr;
            int    speedValue = std::max(0, std::min(999, int(currentSpeedKnot)));

            if (gParams.speedZeroSent < 3)
            {
                String speedStr    = padWithZeros(speedValue, 3);
                int    courseValue = std::max(0, std::min(360, int(currentHeading)));

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

#if 1
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

                Serial.print("TX ==> '");
                Serial.print(data.c_str());
                Serial.println("'");
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

        time_t n = now();
        bool posIsValid = gParams.lastValidGPS.PositionIsValid();
        oled.Display(cfg.callsign,
                formatToDateString(n) + " " + formatToTimeString(n),
                String("Sats: ") + (posIsValid ? String(gParams.lastValidGPS.satellites) : "-") + " HDOP: " + (posIsValid ? String(gParams.lastValidGPS.hdop) : "--.--"),
                String("Nxt Bcn: ") + (gParams.gpsHadFix ? (cfg.smart_beacon.active ? "~" : "") + formatToTimeString(gParams.nextBeaconTimeStamp) : "--:--:--"),
                (gParams.batteryIsConnected ? (String("Bat: ") + gParams.batteryVoltage + "V, " + gParams.batteryChargeCurrent + "mA") : "Powered via USB"),
                String("Smart Beacon: " + getOnOff(cfg.smart_beacon.active)));

#if 0
        Serial.println(String("Sats: ") + String(gParams.lastValidGPS.satellites) + " HDOP: " + gParams.lastValidGPS.hdop +
                " GPS: " + (gpsStillHasToSleep ? "Sleeping" : "Awake") + (gpsHasFix ? " FIX" : " NOFIX"));
        Serial.println(String("Sats: ") + String(gParams.lastValidGPS.satellites) + " HDOP: " + gParams.lastValidGPS.hdop +
                " GPS: " + (gpsStillHasToSleep ? "Sl" : "Ake"));
        Serial.println(String("Nxt Bcn: ") + (cfg.smart_beacon.active ? "~" : "") + formatToTimeString(gParams.nextBeaconTimeStamp) + " / " + formatToTimeString(n));
#endif

        if (timeIsValid)
        {
            if (cfg.smart_beacon.active)
            {
                // Change the Tx internal based on the current speed
                int currentSpeed = int(gps.GetSpeedKPH());

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
                    gParams.txInterval = std::min(cfg.smart_beacon.slow_rate, (cfg.smart_beacon.fast_speed * cfg.smart_beacon.fast_rate) / currentSpeed) * 1000;
                }
            }
        }


        // GPS Sleep/Awake cycling
        if ((gpsStillHasToSleep == false) && gpsHasFix && ((gParams.gpsFixTime >= 0) && ((millis() - gParams.gpsFixTime) > AWAKE_TIME_MS)))
        {
            gps.SetLowPower(true, SLEEP_TIME_MS);
            gParams.gpsFixTime = 0;
        }
        else if (gps.IsSleeping() && (gps.StillHasToSleep() == false))
        {
            gps.SetLowPower(false, 0);
            gParams.gpsFixTime = 0;
        }
    }

    if (gParams.btnInterrupts > 0)
    {
        if (cfg.display_timeout > 0)
        {
            gParams.ResetDisplayTimeout();
        }
        gParams.btnInterrupts = 0;
    }

    // User button has been clicked
    if (gParams.btnClicks != GlobalParameters::BUTTON_CLICKED_NONE)
    {
        switch (gParams.btnClicks)
        {
            case GlobalParameters::BUTTON_CLICKED_ONCE:
                {
                    bool oledWasOn = oled.IsActivated();

                    //oled.Display("SINGLE", 500);

                    gParams.ResetDisplayTimeout(); // Reset the OLED timeout on any button event

                    if (oledWasOn == false) // The OLED was off, hence we won't go further this time
                    {
                        break;
                    }

                    // Send a frame if the screen is already lit.
                    if (cfg.beacon.button_tx)
                    {
                        gParams.sendPositionUpdate = true;
                    }
                }
                break;

            case GlobalParameters::BUTTON_CLICKED_TWICE:
                //oled.Display("DOUBLE", 500);
                break;

            case GlobalParameters::BUTTON_CLICKED_MULTI:
                //oled.Display("MULTI", 500);
                break;

            default:
                break;
        }

        gParams.btnClicks = GlobalParameters::BUTTON_CLICKED_NONE;
    }


    // ESP32 Light Sleep
    if ((userBtn.isIdle() && (gParams.btnInterrupts == 0)) && // User button is released, no button interrupt pending
            ((cfg.display_timeout == 0) || (oled.IsActivated() == false)) && // Screen is OFF (if timeout is set)
            ((gParams.lightSleepExitTime == 0) || ((millis() - gParams.lightSleepExitTime) > gParams.awakenTimePeriod))) // Wait 5s after awaken from the button before going to sleep.
    {
        esp_sleep_wakeup_cause_t cause;
        //uint32_t gpsRemainingSleepTime = gps.GetRemainingSleepTime();

#if 0
        // If the display has a timeout value set, sleep as long as the GPS, otherwise for 500ms (clock accuracy)
        uint64_t sleepTime = (((cfg.display_timeout > 0) && (gpsRemainingSleepTime > 500)) ? gpsRemainingSleepTime : 500);
        Serial.println(uint32_t(sleepTime));
#else
        uint64_t sleepTime = 800;
#endif

        setCpuFrequencyMhz(20);
        cause = execLightSleep(sleepTime);
        setCpuFrequencyMhz(80);

        //Serial.println("X");

        gParams.lightSleepExitTime = millis();

        switch (cause)
        {
            case ESP_SLEEP_WAKEUP_TIMER:
                gParams.awakenTimePeriod = 300; // 300ms
                break;

            case ESP_SLEEP_WAKEUP_UART:
                gParams.awakenTimePeriod = 300; // 300ms
                break;

            default:
            {
                gParams.awakenTimePeriod = ((cfg.display_timeout > 0) ? cfg.display_timeout : 2000);
            }
            break;

        }
    }
}

