#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <list>

#include <Arduino.h>

class Configuration
{
    public:
        static const uint32_t constexpr CONFIGURATION_DISPLAY_TIMEOUT = 5000; // 5 seconds

        class Beacon
        {
            public:
                Beacon() :
                    message("LoRa Tracker, Info: github.com/lora-aprs/LoRa_APRS_Tracker"),
                    timeout(1),
                    button_tx(false),
                    symbol("["),
                    overlay("/")
                {
                }

                String   message;
                uint32_t timeout;
                bool     button_tx;
                String   symbol;
                String   overlay;
        };

        class Smart_Beacon
        {
            public:
                Smart_Beacon() :
                    active(false),
                    turn_min(25),
                    slow_rate(300),
                    slow_speed(10),
                    fast_rate(60),
                    fast_speed(100),
                    min_tx_dist(100),
                    min_bcn(5)
                {
                }

                bool      active;
                uint32_t  turn_min;
                uint32_t  slow_rate;
                uint32_t  slow_speed;
                uint32_t  fast_rate;
                uint32_t  fast_speed;
                uint32_t  min_tx_dist;
                uint32_t  min_bcn;
        };

        class LoRa
        {
            public:
                LoRa() :
                    frequencyRx(433775000),
                    frequencyTx(433775000),
                    power(20),
                    spreadingFactor(12),
                    signalBandwidth(125000),
                    codingRate4(5)
                {
                }

                unsigned long frequencyRx;
                unsigned long frequencyTx;
                uint32_t      power;
                uint32_t      spreadingFactor;
                unsigned long signalBandwidth;
                uint32_t      codingRate4;
        };

        class PTT
        {
            public:
                PTT() :
                    active(false),
                    io_pin(4),
                    start_delay(0),
                    end_delay(0),
                    reverse(false)
                {
                }

                bool      active;
                uint8_t   io_pin;
                uint32_t  start_delay;
                uint32_t  end_delay;
                bool      reverse;
        };

        class Location
        {
            public:
                Location() :
                    latitude(0.00000),
                    longitude(0.00000),
                    altitude(0),
                    symbol("i"),
                    overlay("/")
                {

                }

                double latitude;
                double longitude;
                uint32_t altitude;
                String   symbol;
                String   overlay;
        };

        Configuration() :
            callsign("NOCALL-10"),
            debug(false),
            enhance_precision(true),
            display_timeout(CONFIGURATION_DISPLAY_TIMEOUT)
        {

        };

        String       callsign;
        bool         debug;
        bool         enhance_precision;
        uint32_t     display_timeout;
        Beacon       beacon;
        Smart_Beacon smart_beacon;
        LoRa         lora;
        PTT          ptt;
        Location     location;
};

class ConfigurationManagement
{
    public:
        explicit ConfigurationManagement(String FilePath);

        Configuration readConfiguration();
        void          writeConfiguration(Configuration conf);
#if 0
        static void   dump(const Configuration &conf);
#endif

    private:
        const String m_FilePath;
};

#endif
