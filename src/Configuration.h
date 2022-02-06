#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <iterator>
#include <list>

#include <Arduino.h>

class Configuration
{
    public:
        static const uint32_t constexpr CONFIGURATION_DISPLAY_TIMEOUT = 10; // 10 seconds

        class Beacon
        {
            public:
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

                Beacon() :
                    callsign("NOCALL-10"),
                    path("WIDE1-1"),
                    message("LoRa Tracker"),
                    add_power(true),
                    timeout(1),
                    symbol("["),
                    overlay("/"),
                    enhance_precision(true),
                    lora_power(20)
                {
                }

                String       callsign;
                String       path;
                String       message;
                bool         add_power;
                uint32_t     timeout;
                String       symbol;
                String       overlay;
                Smart_Beacon smart_beacon;
                bool         enhance_precision;
                int32_t      lora_power;
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
                int32_t       power;
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
                    message("LoRa Tracker - GPS Off"),
                    symbol("i"),
                    overlay("/")
                {

                }

                double   latitude;
                double   longitude;
                uint32_t altitude;
                String   message;
                String   symbol;
                String   overlay;
        };

        class Display
        {
            public:
                Display() :
                    invert(false),
                    rotation(0), // 0 .. 3
                    contrast(0xCF),
                    timeout(CONFIGURATION_DISPLAY_TIMEOUT)
                {
                }

                bool     invert;
                uint8_t  rotation;
                uint8_t  contrast;
                uint32_t timeout;

        };

        class Button {
            public:
                Button() :
                    tx(false),
                    alt_message(false)
                {
                }

                bool tx;
                bool alt_message;
        };

        Configuration() :
            debug(false)
        {
        }

        bool              debug;
        std::list<Beacon> beacons;
        Button            button;
        Location          location;
        LoRa              lora;
        PTT               ptt;
        Display           display;
};

class ConfigurationManagement
{
    public:
        explicit ConfigurationManagement(const String &FilePath, const String &defaultFilePath);

        Configuration readConfiguration();
        void          writeConfiguration(Configuration conf);

    private:
        String m_FilePath;
};

#endif
