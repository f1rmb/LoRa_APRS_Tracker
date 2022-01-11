#include <APRS-Decoder.h>
#include <Arduino.h>
#include <LoRa.h>
#include <OneButton.h>
#include <TimeLib.h>
#include <WiFi.h>

#include <esp_sleep.h>
#include <driver/uart.h>
#include <soc/rtc_wdt.h>

#include <thread>

#include "dummyLogger.h"
#include "Configuration.h"
#include "GPSDevice.h"
#include "Display.h"
#include "Pins.h"
#include "PowerManagement.h"
#include "BeaconManager.h"
#include "Deg2DDMMMM.h"

#define MCU_FREQ_ASLEEP       10U // 10MHz
#define MCU_FREQ_AWAKE        20U // 20MHz
#define MCU_FREQ_AWAKE_NEO6   40U // Needs more processing

#define PROGRAM_VERSION  "0.81"


// Function prototype
static void buttonThread();


static const unsigned long millisAfterPvtToEnterSleep[2][2] =
{
        {  25,  26 }, // M8N
        { 140, 141 }  // Neo-6M
};


static Configuration     cfg;
static BeaconManager     bcm;
static OLEDDisplay       oled;
static PowerManagement   pm;
static OneButton         userBtn(BUTTON_PIN, true, true);
static HardwareSerial    ss(1);
static GPSDevice         gps;


struct GlobalParameters
{
    private:
        struct GPSInformations
        {
            GPSInformations()
            {
                Reset();
            }
            ~GPSInformations() { }

            void Reset()
            {
                latitude = NAN;
                longitude = NAN;
                altitude = NAN;
                hdop = 99.99;
                satellites = 0;
            }

            bool PositionIsValid()
            {
                return ((isnan(latitude) == false) && (isnan(longitude) == false) && (isnan(altitude) == false));
            }

            double latitude;
            double longitude;
            double altitude; // in feet
            double hdop;
            uint8_t satellites;
        };

        struct GPSState
        {
                GPSState() :
                    hasFix(false),
                    m_fixCount(0)
                {

                }

                void Reset()
                {
                    hasFix = false;
                    m_fixCount = 0;
                }

                void Tick(bool gpsFixState)
                {
                    if (gpsFixState)
                    {
                        if (hasFix == false)
                        {
                            m_fixCount++;

                            if (m_fixCount == 3)
                            {
                                hasFix = true;
                                gps.SetPowerSaving(true);
                            }
                        }
                        else
                        {
                            if (m_fixCount < 3)
                            {
                                m_fixCount++;
                            }
                        }
                    }
                    else
                    {
                        if (m_fixCount > 0)
                        {
                            m_fixCount--;

                            if (m_fixCount == 0)
                            {
                                hasFix = false;
                                gps.SetPowerSaving(false);
                            }
                        }
                    }
                }

                bool           hasFix;

            private:
                uint32_t       m_fixCount;
        };

    public:
        enum BUTTON_CLICKED
        {
            BUTTON_CLICKED_NONE      = 0,
            BUTTON_CLICKED_ONCE      = 1,
            BUTTON_CLICKED_TWICE     = 2,
            BUTTON_CLICKED_MULTI     = 3,
            BUTTON_CLICKED_LONGPRESS = 4
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
            lastUpdateTime(millis()),
            lightSleepExitTime(0),
            awakenTimePeriod(0),
            btnClicks(GlobalParameters::BUTTON_CLICKED_NONE),
            locationFromGPS(true),
            forceScreenRefresh(false),
            outputPowerdBm(-30),
            outputPowerWatt(0.001),
            pmillisAfterPvtToEnterSleep((unsigned long *)&millisAfterPvtToEnterSleep[0][0]),
            mcuFreqAwake(MCU_FREQ_AWAKE),
            lastPvtMillis(0),
            canGoSleeping(false),
#ifdef TTGO_T_Beam_V1_0
            batteryLastCheckTime(0),
#endif
            m_displayTimeout(Configuration::CONFIGURATION_DISPLAY_TIMEOUT),
            m_displayLastTimeout(millis()),
            m_displayTimeoutEnabled(true)
        {
        }

        uint32_t GetDisplayTimeout()
        {
            return m_displayTimeout;
        }

        void SetDisplayTimeout(uint32_t timeout)
        {
            m_displayTimeout = timeout;
            m_displayTimeoutEnabled = (timeout > 0); // If timeout is set to 0ms, disable it
        }

        void SetOutputPower(int32_t dBm)
        {
            outputPowerdBm = dBm;
            outputPowerWatt = pow(10.0, (dBm - 30.0) / 10.0);
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
                    if ((millis() - m_displayLastTimeout) > (m_displayTimeout * 1000))
                    {
                        oled.Activate(false);
                    }
                }
            }
        }

        bool             hasStarted;
        bool             sendPositionUpdate;
        time_t           nextBeaconTimeStamp;
        double           currentHeading;
        double           previousHeading;
        uint32_t         rateLimitMessageText;
        double           lastTxLat;
        double           lastTxLong;
        int32_t          lastTxDistance;
        uint32_t         txInterval;
        unsigned long    lastTxTime;
        int              speedZeroSent;
        bool             batteryIsConnected;
        String           batteryVoltage;
        String           batteryChargeCurrent;
        unsigned long    lastUpdateTime;
        unsigned long    lightSleepExitTime;
        unsigned long    awakenTimePeriod;
        BUTTON_CLICKED   btnClicks;
        bool             locationFromGPS;
        bool             forceScreenRefresh;
        int32_t          outputPowerdBm;
        double           outputPowerWatt;
        unsigned long   *pmillisAfterPvtToEnterSleep;
        uint32_t         mcuFreqAwake;
        unsigned long    lastPvtMillis;
        bool             canGoSleeping;
#ifdef TTGO_T_Beam_V1_0
        unsigned long    batteryLastCheckTime;
#endif
        GPSInformations  lastValidGPS;
        GPSState         gpsState;

    private:
        uint32_t         m_displayTimeout;
        unsigned long    m_displayLastTimeout;
        bool             m_displayTimeoutEnabled;
};

static GlobalParameters gParams;

std::thread buttonLoopThread(buttonThread);

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

static void buttonLongPressCallback()
{
    gParams.btnClicks = GlobalParameters::BUTTON_CLICKED_LONGPRESS;
}

static void loadConfiguration()
{
    ConfigurationManagement confmg("/tracker.json");
    cfg = confmg.readConfiguration();
    bcm.loadConfig(cfg.beacons);
}

static void gpsInitialize()
{
    oled.Display(" GPS INIT", emptyString, "Initialize...", 10);

    if (gps.Initialize(ss) == false)
    {
        DlogPrintlnE("GPS Init Failed!");
#ifdef TTGO_T_Beam_V1_0 // Power cycle the GPS module
        oled.Display(" GPS INIT", emptyString, "Initialization Failed",  "   Power cycling &", "     rebooting...");
        pm.GPSDeactivate();
        delay(5000);
        ESP.restart(); // Reboot
#else
        oled.Display(" GPS INIT", emptyString, "Initialization Failed", " Please power cycle. ");
        setCpuFrequencyMhz(MCU_FREQ_ASLEEP);
        while (true) { delay(10); }
#endif
    }
}

static void loraInitialize()
{
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

    long freq = cfg.lora.frequencyTx;

    oled.Display(" LoRa INIT", emptyString, "Initialize...", "Freq: " + String(freq) + "Hz", 2000);

    if (LoRa.begin(freq) == false)
    {
        DlogPrintlnE("LoRa Init Failed!");
        oled.Display(" LoRa INIT", emptyString, "Initialization Failed", " Please power cycle. ");

        setCpuFrequencyMhz(MCU_FREQ_ASLEEP);
        while (true) { delay(10); }
    }

    LoRa.setSpreadingFactor(cfg.lora.spreadingFactor);
    LoRa.setSignalBandwidth(cfg.lora.signalBandwidth);
    LoRa.setCodingRate4(cfg.lora.codingRate4);
    LoRa.enableCrc();

    LoRa.setTxPower(cfg.lora.power);
    LoRa.sleep();

    oled.Display(" LoRa INIT", emptyString, "Initialization OK", 2000);
}

// WARNING: don't use this one in *printf() because of static buffer
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

#if 0
    // enable or disable INT for GPS TX pin.
    if (gParams.locationFromGPS)
    {
        gpio_wakeup_enable((gpio_num_t)GPS_TX, GPIO_INTR_LOW_LEVEL);
    }
    else
    {
        gpio_wakeup_disable((gpio_num_t)GPS_TX);
    }
#endif

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
        delay(10);

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
    if (pm.begin(Wire))
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

    loadConfiguration();

    oled.Init(cfg.display.invert, cfg.display.rotation, cfg.display.contrast);

#if defined(USE_BOOTSCREEN)
    oled.ShowBootscreen("v" + String(PROGRAM_VERSION), 98, 56, BLACK, 4000);
#else
    oled.Display("  OE5BPA", "  LoRa APRS Tracker", " by  Peter Buchegger", emptyString, " Mods: Daniel, F1RMB", "                v" + String(PROGRAM_VERSION), 2000);
#endif
    // Check the callsign setting validity
    if ((bcm.getCurrentBeaconConfig()->callsign.length() == 0) ||
            bcm.getCurrentBeaconConfig()->callsign.startsWith("NOCALL"))
    {
        DlogPrintlnE("You have to change your settings in 'data/tracker.json' and "
                "upload it via \"Upload File System image\"!");
        oled.Display("  ERROR!", "You have to change your settings in 'data/tracker.json' and "
                "upload it via \"Upload File System image\"!");

        setCpuFrequencyMhz(MCU_FREQ_ASLEEP);
        while (true) { delay(10); }
    }

    gpsInitialize();
    loraInitialize();

    if (cfg.ptt.active)
    {
        pinMode(cfg.ptt.io_pin, OUTPUT);
        digitalWrite(cfg.ptt.io_pin, cfg.ptt.reverse ? HIGH : LOW);
    }

    gParams.SetDisplayTimeout(cfg.display.timeout);
    gParams.SetOutputPower(cfg.lora.power);

    // Select to right milliseconds entry in the array.
    // Neo-6M has to process more UBX packets than the M8N
    gParams.pmillisAfterPvtToEnterSleep = (unsigned long *)(gps.IsNeo6M() ? &millisAfterPvtToEnterSleep[1][0] : &millisAfterPvtToEnterSleep[0][0]);

    // Neo-6M needs more processing power (due to UBX processing), using 20MHz make the screen flickering.
    gParams.mcuFreqAwake = (gps.IsNeo6M() ? MCU_FREQ_AWAKE_NEO6 : MCU_FREQ_AWAKE);

    // make sure wifi and bt are off as we don't need it:
    WiFi.mode(WIFI_OFF);
    btStop();

    // OneButton config
    userBtn.attachClick(buttonClickCallback);
    userBtn.attachDoubleClick(buttonDoubleClickCallback);
    userBtn.attachMultiClick(buttonMultiPressCallback);
    userBtn.attachLongPressStart(buttonLongPressCallback);

    DlogPrintlnI("Smart Beacon is " + getOnOff(bcm.getCurrentBeaconConfig()->smart_beacon.active));
    oled.Display("   INFO", emptyString, "Smart Beacon is " + getOnOff(bcm.getCurrentBeaconConfig()->smart_beacon.active), 1000);
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

    oled.Display("   INFO", emptyString, "Running...");

    gParams.ResetDisplayTimeout(); // Enable OLED timeout
    gParams.hasStarted = true; // main loop will start, unlock the userButton thread
    setCpuFrequencyMhz(gParams.mcuFreqAwake);
}


// cppcheck-suppress unusedFunction
void loop()
{
    bool goToSleep = false;

    gParams.DisplayTick();

    // Check if a PVT data is available, when GPS is in use
    bool gpsHasPVT = gParams.locationFromGPS ? gps.GetPVT() : false;
    bool forceEntering = false;

    // Process incoming UBX packets
    if (gParams.locationFromGPS)
    {
        gps.Tick();

        // We didn't had a PVT packet in the last 10s, flush the GNSS module (it stops sending packets after a loss of the fix)
        if ((gpsHasPVT == false) && ((millis() - gParams.lastUpdateTime) > 10000))
        {
            gParams.gpsState.Reset();
            gps.FlushAndSetAutoPVT();
            forceEntering = true; // Update the screen
            gParams.lastUpdateTime = millis(); // update last action time
        }
    }
    else
    {
        // force updates every seconds when the GNSS is OFF
        forceEntering = ((millis() - gParams.lastUpdateTime) >= 1000);
    }

#if 0
    while (Serial.available())
    {
        char c = Serial.read();

        if (c == 'T')
        {
            Serial.println("TOGGLE GPS ***************");
            gParams.ResetDisplayTimeout();
            gParams.btnClicks = GlobalParameters::BUTTON_CLICKED_MULTI;
            //forceEntering = true;
        }
        else if (c == 'R')
        {
            Serial.println("REBOOT ***************");
            ESP.restart();
        }
    }
#endif

    if (gpsHasPVT || forceEntering || gParams.forceScreenRefresh)
    {
        gParams.lastUpdateTime = millis();
        gParams.forceScreenRefresh = false;
        gParams.lastPvtMillis = millis();
        gParams.canGoSleeping = true;

        bool     gpsPVT             = gpsHasPVT;
        bool     gpsFix             = gParams.locationFromGPS ? false : true;
        double   currentLat         = NAN;
        double   currentLong        = NAN;
        double   currentHeading     = NAN;
        double   currentAltInFeet   = NAN;
        double   currentSpeedKnot   = NAN;
        bool     timeIsValid        = false;
        bool     gpsHasFix          = (gParams.locationFromGPS ? ((gpsPVT = gpsHasPVT) && (gpsFix = gps.HasFix())) : true);

        // Reset stored location when the Fix is lost.
        if (gParams.locationFromGPS && ((gpsFix == false) && gParams.lastValidGPS.PositionIsValid()))
        {
            gParams.lastValidGPS.Reset();
        }

        // Handles fix state and GNSS powersaving mode
        if (gParams.locationFromGPS)
        {
            gParams.gpsState.Tick(gpsHasFix);
        }

        if (gpsHasFix)
        {
            if (gParams.locationFromGPS)
            {
                gParams.lastValidGPS.latitude  = currentLat      = gps.GetLatitude();
                gParams.lastValidGPS.longitude = currentLong     = gps.GetLongitude();
                gParams.lastValidGPS.hdop                        = gps.GetHDOP();
                gParams.lastValidGPS.satellites                  = gps.GetSatellites();
                gParams.lastValidGPS.altitude = currentAltInFeet = gps.GetAltitudeFT();
                currentHeading                                   = gps.GetHeading();
                currentSpeedKnot                                 = gps.GetSpeedKT();
            }
            else
            {
                currentLat       = gParams.lastValidGPS.latitude;
                currentLong      = gParams.lastValidGPS.longitude;
                currentAltInFeet = double(gParams.lastValidGPS.altitude);
                currentHeading   = gParams.previousHeading;
                currentSpeedKnot = 0.0;
            }

            // Check GPS clock, handle beaconing based on time (+ heading when smart beaconing is enabled)
            struct tm dt;
            if ((gParams.locationFromGPS == false) || (timeIsValid = gps.GetDateAndTime(dt)))
            {
                if (timeIsValid)
                {
                    time_t tGPS = mktime(&dt);

                    if (now() != tGPS) // Update on clock skew only
                    {
                        setTime(tGPS);
                    }
                }

                if (now() >= gParams.nextBeaconTimeStamp)
                {
                    gParams.sendPositionUpdate = true;

                    if (gParams.locationFromGPS && bcm.getCurrentBeaconConfig()->smart_beacon.active)
                    {
                        gParams.currentHeading = currentHeading;
                        // enforce message text on slowest smart_beacon.slow_rate
                        gParams.rateLimitMessageText = 0;
                    }
                    else
                    {
                        // enforce message text every n's beacon.timeout frame
                        if ((bcm.getCurrentBeaconConfig()->timeout * gParams.rateLimitMessageText) > 30)
                        {
                            gParams.rateLimitMessageText = 0;
                        }
                    }
                }
            }
        }

        // Smart beaconing, with GPS Fix
        if ((gParams.sendPositionUpdate == false) && gpsHasFix && (gParams.locationFromGPS && bcm.getCurrentBeaconConfig()->smart_beacon.active))
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

                if (lastTx > (bcm.getCurrentBeaconConfig()->smart_beacon.min_bcn * 1000))
                {
                    // Check for heading more than config's **turn_min** degrees
                    if ((headingDelta > double(bcm.getCurrentBeaconConfig()->smart_beacon.turn_min)) &&
                            (gParams.lastTxDistance > bcm.getCurrentBeaconConfig()->smart_beacon.min_tx_dist))
                    {
                        gParams.sendPositionUpdate = true;
                    }
                }
            }
        }

        // Battery reading
#ifdef TTGO_T_Beam_V1_0
        // Update the battery on then first iteration, or just before transmitting, or every 60 seconds as it's way enough
        if (((gParams.sendPositionUpdate && gpsHasFix) /*|| (oled.IsActivated())*/ ) ||
                ((gParams.batteryLastCheckTime == 0) || ((millis() - gParams.batteryLastCheckTime) >= 60000)))
        {
            gParams.batteryIsConnected = pm.isBatteryConnected();
            gParams.batteryLastCheckTime = millis();

            if (gParams.batteryIsConnected)
            {
                int32_t mA = int32_t(pm.getBatteryChargeDischargeCurrent());

                gParams.batteryVoltage       = String(pm.getBatteryVoltage(), 2);
                gParams.batteryChargeCurrent = String(mA);
            }
        }
#endif

        // Time to send an APRS frame
        if (gParams.sendPositionUpdate && gpsHasFix)
        {
            APRSMessage        msgAprs;
            Deg2DDMMMMPosition pLat, pLong;
            char               latBuf[32];
            char               longBuf[32];

            gParams.sendPositionUpdate = false;
            gParams.nextBeaconTimeStamp = now() + ((gParams.locationFromGPS && bcm.getCurrentBeaconConfig()->smart_beacon.active) ?
                    bcm.getCurrentBeaconConfig()->smart_beacon.slow_rate : (bcm.getCurrentBeaconConfig()->timeout * SECS_PER_MIN));

            msgAprs.setSource(bcm.getCurrentBeaconConfig()->callsign);
            msgAprs.setPath(bcm.getCurrentBeaconConfig()->path);
            msgAprs.setDestination("APLT00-1");

            // Lat/Long
            Deg2DDMMMM::Convert(pLat, currentLat, bcm.getCurrentBeaconConfig()->enhance_precision);
            Deg2DDMMMM::Convert(pLong, currentLong, bcm.getCurrentBeaconConfig()->enhance_precision);

            String             latStr(Deg2DDMMMM::Format(latBuf, pLat, false));
            String             longStr(Deg2DDMMMM::Format(longBuf, pLong, true));
            String             daoStr;

            if (bcm.getCurrentBeaconConfig()->enhance_precision)
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


            String aprsmsgStr("!" + latStr + (gParams.locationFromGPS ? bcm.getCurrentBeaconConfig()->overlay : cfg.location.overlay) + longStr +
                    (gParams.locationFromGPS ? bcm.getCurrentBeaconConfig()->symbol : cfg.location.symbol) + courseAndSpeedStr + altStr);

            // message_text every 10's packet (i.e. if we have beacon rate 1min at high
            // speed -> every 10min). May be enforced above (at expirey of smart beacon
            // rate (i.e. every 30min), or every third packet on static rate (i.e.
            // static rate 10 -> every third packet)
            if ((gParams.rateLimitMessageText++ % 10) == 0)
            {
                aprsmsgStr += (gParams.locationFromGPS ? bcm.getCurrentBeaconConfig()->message : cfg.location.message);
            }

            if (gParams.batteryIsConnected)
            {
                aprsmsgStr += " -  _Bat.: " + gParams.batteryVoltage + "V - Cur.: " + gParams.batteryChargeCurrent + "mA";

                if (bcm.getCurrentBeaconConfig()->add_power)
                {
                    aprsmsgStr += " - Pwr: " + String((gParams.outputPowerWatt * 1e3), 0) + "mW";
                }
            }
            else
            {
                if (bcm.getCurrentBeaconConfig()->add_power)
                {
                    aprsmsgStr += " - Pwr: " + String((gParams.outputPowerWatt * 1e3), 0) + "mW";
                }
            }


            if (bcm.getCurrentBeaconConfig()->enhance_precision && (daoStr.length() > 0))
            {
                aprsmsgStr += " " + daoStr;
            }

            msgAprs.getBody()->setData(aprsmsgStr);
            String data(msgAprs.encode());
            DlogPrintlnD(data);

            oled.Display(" << TX >>", data);

            if (cfg.ptt.active)
            {
                digitalWrite(cfg.ptt.io_pin, (cfg.ptt.reverse ? LOW : HIGH));
                delay(cfg.ptt.start_delay);
            }

            if (LoRa.beginPacket() != 0) // Ensure the LoRa module is not transmiting
            {
                // Header:
                LoRa.write('<');
                LoRa.write(0xFF);
                LoRa.write(0x01);
                // APRS Data:
                LoRa.write((const uint8_t *)data.c_str(), data.length());
                LoRa.endPacket(); // Send SYNC

#if 0
#warning DISABLE FRAME TRACE TXING
                Serial.print("TX ==> '");
                Serial.print(data.c_str());
                Serial.println("'");
#endif
            }
            else
            {
                gParams.sendPositionUpdate = true; // Try to resend on the next run
            }

            if (gParams.locationFromGPS && bcm.getCurrentBeaconConfig()->smart_beacon.active)
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

        static bool blink = false; // Yeah, ugly...
        time_t n = now();
        bool posIsValid = gParams.lastValidGPS.PositionIsValid();
        String dtStr = (formatToDateString(n) + " " + formatToTimeString(n));

        if (gParams.locationFromGPS && gpsFix)
        {
            if (blink)
            {
                dtStr += " *";
            }

            blink = !blink;
        }

        oled.Display(bcm.getCurrentBeaconConfig()->callsign,
                dtStr,
                String("Sats: ") + (posIsValid ? (gParams.locationFromGPS ? String(gParams.lastValidGPS.satellites) : "F" ) : "-") + " HDOP: " + (posIsValid ? String(gParams.lastValidGPS.hdop) : "--.--"),
                String("Nxt Bcn: ") + (posIsValid ? ((gParams.locationFromGPS && bcm.getCurrentBeaconConfig()->smart_beacon.active) ? "~" : "") + formatToTimeString(gParams.nextBeaconTimeStamp) : "--:--:--"),
                (gParams.batteryIsConnected ? (String("Bat: ") + gParams.batteryVoltage + "V, " + gParams.batteryChargeCurrent + "mA") : "Powered via USB"),
                String("S-Beacon: " + getOnOff(gParams.locationFromGPS && bcm.getCurrentBeaconConfig()->smart_beacon.active)) + ", " + String((gParams.outputPowerWatt * 1e3), 0) + "mW");

#if 0
        Serial.println(String("Sats: ") + String(gParams.lastValidGPS.satellites) + " HDOP: " + gParams.lastValidGPS.hdop +
                " GPS: " + (posIsValid ? "VALID" : "INVALID") + (gpsHasFix ? " FIX" : " NOFIX") + (" f:") + String(gpsFix) + "  type:" + String(gpsFixType));
//        Serial.println(String("Sats: ") + String(gParams.lastValidGPS.satellites) + " HDOP: " + gParams.lastValidGPS.hdop +
//                " GPS: " + (posIsValid ? "Sl" : "Ake"));
        Serial.println(String("Nxt Bcn: ") + (bcm.getCurrentBeaconConfig()->smart_beacon.active ? "~" : "") + formatToTimeString(gParams.nextBeaconTimeStamp) + " / " + formatToTimeString(n));
#else
        //Serial.println(String("Sats: ") + (posIsValid ? String(gParams.lastValidGPS.satellites) : "-") + " HDOP: " + (posIsValid ? String(gParams.lastValidGPS.hdop) : "--.--"));
        //Serial.println(String("Nxt Bcn: ") + (posIsValid ? (bcm.getCurrentBeaconConfig()->smart_beacon.active ? "~" : "") + formatToTimeString(gParams.nextBeaconTimeStamp) : "--:--:--"));
#endif

        // Beacon's TX interval adjustement
        if (timeIsValid)
        {
            if (gParams.locationFromGPS && bcm.getCurrentBeaconConfig()->smart_beacon.active)
            {
                // Change the Tx interval based on the current speed
                int currentSpeed = int((currentSpeedKnot / 1.9438444924));

                if (currentSpeed < bcm.getCurrentBeaconConfig()->smart_beacon.slow_speed)
                {
                    gParams.txInterval = (bcm.getCurrentBeaconConfig()->smart_beacon.slow_rate * 1000);
                }
                else if (currentSpeed > bcm.getCurrentBeaconConfig()->smart_beacon.fast_speed)
                {
                    gParams.txInterval = (bcm.getCurrentBeaconConfig()->smart_beacon.fast_rate * 1000);
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
                    gParams.txInterval = std::min(bcm.getCurrentBeaconConfig()->smart_beacon.slow_rate,
                            (bcm.getCurrentBeaconConfig()->smart_beacon.fast_speed * bcm.getCurrentBeaconConfig()->smart_beacon.fast_rate) / currentSpeed) * 1000;
                }
            }
        }
    }

    // User button has been clicked
    if (gParams.btnClicks != GlobalParameters::BUTTON_CLICKED_NONE)
    {
        bool oledWasOn = oled.IsActivated();

        gParams.ResetDisplayTimeout(); // Reset the OLED timeout on any button event

        if (oledWasOn)
        {
            switch (gParams.btnClicks)
            {
                case GlobalParameters::BUTTON_CLICKED_ONCE:
                    // Send a frame if the screen is already lit.
                    if (cfg.button.tx)
                    {
                        gParams.sendPositionUpdate = true;
                    }
                    break;

                case GlobalParameters::BUTTON_CLICKED_TWICE:
                    if (cfg.display.timeout > 0)
                    {
                        uint32_t dispTo = ((gParams.GetDisplayTimeout() == cfg.display.timeout) ? 0 : cfg.display.timeout);

                        gParams.ResetDisplayTimeout();
                        oled.Display("  SCREEN", emptyString, "Timeout: " + ((dispTo > 0) ? (String(dispTo) + "s") : "disabled"), 2000);
                        gParams.SetDisplayTimeout(dispTo);
                        gParams.forceScreenRefresh = true;
                    }
                    break;

                case GlobalParameters::BUTTON_CLICKED_MULTI:
                    if (gParams.locationFromGPS)
                    {
                        bool gpsLocationIsValid = gParams.lastValidGPS.PositionIsValid();

                        if (gpsLocationIsValid == false)
                        {
                            gParams.lastValidGPS.latitude = cfg.location.latitude;
                            gParams.lastValidGPS.longitude = cfg.location.longitude;
                            gParams.lastValidGPS.altitude = (double(cfg.location.altitude) * 3.2808399); // meters to feet
                            gParams.lastValidGPS.hdop = 0.00;
                            gParams.lastValidGPS.satellites = 0;
                        }

                        gParams.currentHeading = gParams.previousHeading = 0; // We won't move, reset heading.

                        oled.Display(" LOCATION", emptyString, "-- Fixed Position --",
                                String("Lat:  ") + String(gParams.lastValidGPS.latitude, 6),
                                String("Long: ") + String(gParams.lastValidGPS.longitude, 6),
                                String("Alt:  ") + String(int(gParams.lastValidGPS.altitude / 3.2808399)) + "m");
#ifdef TTGO_T_Beam_V1_0
                        gps.Stop();
                        pm.GPSDeactivate();
#endif
#ifdef TTGO_T_Beam_V0_7
                        gps.SetPowerOff(true, 0);
#endif
                        delay(2000);
                    }
                    else
                    {
                        oled.Display(" LOCATION", emptyString, "   -- Using GPS --");
#ifdef TTGO_T_Beam_V1_0
                        // It takes too long to initialize the GNSS, hence
                        // disable the watchdog while the process is running
                        // prevents it to kicks in
                        rtc_wdt_protect_off();
                        rtc_wdt_disable();
                        rtc_wdt_protect_on();
                        pm.GPSActivate();
                        delay(5000);
                        gpsInitialize();
                        delay(3000);
                        rtc_wdt_protect_off();
                        rtc_wdt_enable(); // All done, reenable the WDT.
                        rtc_wdt_protect_on();
#endif
#ifdef TTGO_T_Beam_V0_7
                        gps.SetPowerOff(false, 0);
                        gps.FlushAndSetAutoPVT();
#endif
                        gParams.lastValidGPS.Reset();
                        gps.Tick();
                        gParams.gpsState.Reset();
                    }
                    gParams.locationFromGPS = !gParams.locationFromGPS;
                    gParams.forceScreenRefresh = true;
                    break;

                case GlobalParameters::BUTTON_CLICKED_LONGPRESS:
                    if (cfg.button.alt_message)
                    {
                        bcm.loadNextBeacon();
                        oled.Display(bcm.getCurrentBeaconConfig()->callsign, emptyString, bcm.getCurrentBeaconConfig()->message, 2000);
                        gParams.forceScreenRefresh = true;
                    }
                    break;

                default:
                    break;
            }

            gParams.ResetDisplayTimeout();
        }

        gParams.btnClicks = GlobalParameters::BUTTON_CLICKED_NONE;
    }

    if (gParams.locationFromGPS)
    {
        unsigned long m = (millis() - gParams.lastPvtMillis);

        // Do we meet the conditions to execute a LightSleep ?
        if (gParams.canGoSleeping && (gParams.lastPvtMillis > 0) && (ss.available() == 0) &&
                ((m >= *gParams.pmillisAfterPvtToEnterSleep) && (m <= *(gParams.pmillisAfterPvtToEnterSleep + 1))))
        {
            goToSleep = true;
            gParams.canGoSleeping = false;
            gParams.lastPvtMillis = 0;
        }
    }

    // ESP32 Light Sleep
    if (userBtn.isIdle() && (gParams.forceScreenRefresh == false) && // Button is idling, no refresh screen is pending
            ((gParams.locationFromGPS == false) || (goToSleep)) && // GNSS is OFF or we have a small time window to sleep
            ((gParams.GetDisplayTimeout() == 0) || (oled.IsActivated() == false)) && // Screen is OFF (if timeout is set)
            ((gParams.lightSleepExitTime == 0) || ((millis() - gParams.lightSleepExitTime) > gParams.awakenTimePeriod))) // Wait 5s after awaken from the button before going to sleep.
    {
        esp_sleep_wakeup_cause_t cause;
        uint64_t sleepTime = gParams.locationFromGPS ? 700 : 800;

        setCpuFrequencyMhz(MCU_FREQ_ASLEEP);
        cause = execLightSleep(sleepTime);
        setCpuFrequencyMhz(gParams.mcuFreqAwake);

        switch (cause)
        {
            case ESP_SLEEP_WAKEUP_TIMER:
                gParams.awakenTimePeriod = 200; // 200ms
                break;

            case ESP_SLEEP_WAKEUP_UART:
                gParams.awakenTimePeriod = 200; // 200ms
                break;

            case ESP_SLEEP_WAKEUP_GPIO:
                //gParams.ResetDisplayTimeout(); // fallthrough
            default:
            {
                gParams.awakenTimePeriod = ((cfg.display.timeout > 0) ? cfg.display.timeout : 2000);
            }
            break;
        }

        gParams.lightSleepExitTime = millis();
    }
}
