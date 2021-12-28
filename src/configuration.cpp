#include <SPIFFS.h>

#ifndef CPPCHECK
#include <ArduinoJson.h>
#endif

#include "dummylogger.h"
#include "configuration.h"

ConfigurationManagement::ConfigurationManagement(String FilePath) :
m_FilePath(FilePath)
{
    if (!SPIFFS.begin(true))
    {
        DlogPrintlnE("Mounting SPIFFS was not possible. Trying to format SPIFFS...");
        SPIFFS.format();
        if (!SPIFFS.begin())
        {
            DlogPrintlnE("Formating SPIFFS was not okay!");
        }
    }
}

static bool toDouble(const String &s, double *v)
{
    return (sscanf(s.c_str(), "%lf", v) == 1);
}

// cppcheck-suppress unusedFunction
Configuration ConfigurationManagement::readConfiguration()
{
    File file = SPIFFS.open(m_FilePath);

    if (!file)
    {
        DlogPrintlnE("Failed to open file for reading...");
        return Configuration();
    }


    DynamicJsonDocument  data(2048);
    DeserializationError error = deserializeJson(data, file);

    if (error)
    {
        DlogPrintlnE("Failed to read file, using default configuration.");
    }

    file.close();


    Configuration conf;

    if (data.containsKey("callsign"))
    {
        conf.callsign                 = data["callsign"].as<String>();
    }

    conf.debug                        = data["debug"] | false;
    conf.enhance_precision            = data["enhance_precision"] | false;
    conf.display_timeout              = data["display_timeout"] | Configuration::CONFIGURATION_DISPLAY_TIMEOUT;

    if (data.containsKey("beacon"))
    {
        if (data["beacon"].containsKey("message"))
        {
            conf.beacon.message       = data["beacon"]["message"].as<String>();
        }

        conf.beacon.timeout = data["beacon"]["timeout"] | 1;

        if (data["beacon"].containsKey("button_tx"))
        {
            conf.beacon.button_tx     = data["beacon"]["button_tx"] | false;
        }

        if (data["beacon"].containsKey("symbol"))
        {
            conf.beacon.symbol        = data["beacon"]["symbol"].as<String>();
        }

        if (data["beacon"].containsKey("overlay"))
        {
            conf.beacon.overlay       = data["beacon"]["overlay"].as<String>();
        }
    }

    if (data.containsKey("smart_beacon"))
    {
        conf.smart_beacon.active      = data["smart_beacon"]["active"] | false;
        conf.smart_beacon.turn_min    = data["smart_beacon"]["turn_min"] | 25;
        conf.smart_beacon.slow_rate   = data["smart_beacon"]["slow_rate"] | 300;
        conf.smart_beacon.slow_speed  = data["smart_beacon"]["slow_speed"] | 10;
        conf.smart_beacon.fast_rate   = data["smart_beacon"]["fast_rate"] | 60;
        conf.smart_beacon.fast_speed  = data["smart_beacon"]["fast_speed"] | 100;
        conf.smart_beacon.min_tx_dist = data["smart_beacon"]["min_tx_dist"] | 100;
        conf.smart_beacon.min_bcn     = data["smart_beacon"]["min_bcn"] | 5;
    }

    if (data.containsKey("lora"))
    {
        conf.lora.frequencyRx         = data["lora"]["frequency_rx"] | 433775000;
        conf.lora.frequencyTx         = data["lora"]["frequency_tx"] | 433775000;
        conf.lora.power               = data["lora"]["power"] | 20;
        conf.lora.spreadingFactor     = data["lora"]["spreading_factor"] | 12;
        conf.lora.signalBandwidth     = data["lora"]["signal_bandwidth"] | 125000;
        conf.lora.codingRate4         = data["lora"]["coding_rate4"] | 5;
    }

    if (data.containsKey("ptt_output"))
    {
        conf.ptt.active               = data["ptt_output"]["active"] | false;
        conf.ptt.io_pin               = data["ptt_output"]["io_pin"] | 4;
        conf.ptt.start_delay          = data["ptt_output"]["start_delay"] | 0;
        conf.ptt.end_delay            = data["ptt_output"]["end_delay"] | 0;
        conf.ptt.reverse              = data["ptt_output"]["reverse"] | false;
    }

    if (data.containsKey("location"))
    {
        // JSON to double looses precision, hence use sscanf()
        if (toDouble(data["location"]["latitude"].as<String>(), &conf.location.latitude) == false)
        {
            conf.location.latitude  = data["location"]["latitude"];
        }

        // JSON to double looses precision, hence use sscanf()
        if (toDouble(data["location"]["longitude"].as<String>(), &conf.location.longitude) == false)
        {
            conf.location.longitude = data["location"]["longitude"];
        }
        conf.location.altitude      = data["location"]["altitude"];

        if (data["location"].containsKey("symbol"))
        {
            conf.location.symbol        = data["location"]["symbol"].as<String>();
        }

        if (data["location"].containsKey("overlay"))
        {
            conf.location.overlay       = data["location"]["overlay"].as<String>();
        }
    }

    if (data.containsKey("display"))
    {
        conf.display.invert             = data["display"]["invert"] | false;
        conf.display.rotation           = data["display"]["rotation"] | 0;
    }

    return conf;
}

// cppcheck-suppress unusedFunction
void ConfigurationManagement::writeConfiguration(Configuration conf)
{
    File file = SPIFFS.open(m_FilePath, "w");

    if (!file)
    {
        DlogPrintlnE("Failed to open file for writing...");
        return;
    }

    DynamicJsonDocument data(2048);

    data["callsign"]                    = conf.callsign;
    data["debug"]                       = conf.debug;
    data["enhance_precision"]           = conf.enhance_precision;
    data["display_timeout"]             = conf.display_timeout;
    data["beacon"]["message"]           = conf.beacon.message;
    data["beacon"]["timeout"]           = conf.beacon.timeout;
    data["beacon"]["button_tx"]         = conf.beacon.button_tx;
    data["beacon"]["symbol"]            = conf.beacon.symbol;
    data["beacon"]["overlay"]           = conf.beacon.overlay;
    data["smart_beacon"]["active"]      = conf.smart_beacon.active;
    data["smart_beacon"]["turn_min"]    = conf.smart_beacon.turn_min;
    data["smart_beacon"]["slow_rate"]   = conf.smart_beacon.slow_rate;
    data["smart_beacon"]["slow_speed"]  = conf.smart_beacon.slow_speed;
    data["smart_beacon"]["fast_rate"]   = conf.smart_beacon.fast_rate;
    data["smart_beacon"]["fast_speed"]  = conf.smart_beacon.fast_speed;
    data["smart_beacon"]["min_tx_dist"] = conf.smart_beacon.min_tx_dist;
    data["smart_beacon"]["min_bcn"]     = conf.smart_beacon.min_bcn;

    data["lora"]["frequency_rx"]        = conf.lora.frequencyRx;
    data["lora"]["frequency_tx"]        = conf.lora.frequencyTx;
    data["lora"]["power"]               = conf.lora.power;
    data["lora"]["spreading_factor"]    = conf.lora.spreadingFactor;
    data["lora"]["signal_bandwidth"]    = conf.lora.signalBandwidth;
    data["lora"]["coding_rate4"]        = conf.lora.codingRate4;

    data["ptt_out"]["active"]           = conf.ptt.active;
    data["ptt_out"]["io_pin"]           = conf.ptt.io_pin;
    data["ptt_out"]["start_delay"]      = conf.ptt.start_delay;
    data["ptt_out"]["end_delay"]        = conf.ptt.end_delay;
    data["ptt_out"]["reverse"]          = conf.ptt.reverse;

    data["location"]["latitude"]        = conf.location.latitude;
    data["location"]["longitude"]       = conf.location.longitude;
    data["location"]["altitude"]        = conf.location.altitude;
    data["location"]["symbol"]          = conf.location.symbol;
    data["location"]["overlay"]         = conf.location.overlay;

    data["display"]["invert"]           = conf.display.invert;
    data["display"]["rotation"]         = conf.display.rotation;

    serializeJson(data, file);
    file.close();
}

#if 0
static void dprint(const String &n, const String &v)
{
    Serial.print(n); Serial.print(">s: ");
    Serial.println(v);
}
static void dprint(const String &n, const bool &v)
{
    Serial.print(n); Serial.print(">b: ");
    Serial.println(v);
}
static void dprint(const String &n, const double &v)
{
    Serial.print(n); Serial.print(">d: ");
    Serial.println(v, 6);
}
static void dprint(const String &n, const uint32_t &v)
{
    Serial.print(n); Serial.print(">u32: ");
    Serial.println(v);
}
static void dprint(const String &n, const uint8_t &v)
{
    Serial.print(n); Serial.print(">u8: ");
    Serial.println(v);
}
static void dprint(const String &n, const unsigned long &v)
{
    Serial.print(n); Serial.print(">ul: ");
    Serial.println(v);
}

void ConfigurationManagement::dump(const Configuration &conf)
{
    dprint("[callsign]", conf.callsign);

    dprint("[debug]", conf.debug);
    dprint("[enhance_precision]", conf.enhance_precision);
    dprint("[display_timeout]", conf.display_timeout);

    dprint("[beacon][message]", conf.beacon.message);
    dprint("[beacon][timeout]", conf.beacon.timeout);

    dprint("[beacon][symbol]", conf.beacon.symbol);
    dprint("[beacon][overlay]", conf.beacon.overlay);
    dprint("[beacon][button_tx]", conf.beacon.button_tx);

    dprint("[smart_beacon][active]", conf.smart_beacon.active);
    dprint("[smart_beacon][turn_min]", conf.smart_beacon.turn_min);
    dprint("[smart_beacon][slow_rate]", conf.smart_beacon.slow_rate);
    dprint("[smart_beacon][slow_speed]", conf.smart_beacon.slow_speed);
    dprint("[smart_beacon][fast_rate]", conf.smart_beacon.fast_rate);
    dprint("[smart_beacon][fast_speed]", conf.smart_beacon.fast_speed);
    dprint("[smart_beacon][min_tx_dist]", conf.smart_beacon.min_tx_dist);
    dprint("[smart_beacon][min_bcn]", conf.smart_beacon.min_bcn);
    dprint("[lora][frequency_rx]", conf.lora.frequencyRx);
    dprint("[lora][frequency_tx]", conf.lora.frequencyTx);
    dprint("[lora][power]", conf.lora.power);
    dprint("[lora][spreading_factor]", conf.lora.spreadingFactor);
    dprint("[lora][signal_bandwidth]", conf.lora.signalBandwidth);
    dprint("[lora][coding_rate4]", conf.lora.codingRate4);
    dprint("[ptt_output][active]", conf.ptt.active);
    dprint("[ptt_output][io_pin]", conf.ptt.io_pin);
    dprint("[ptt_output][start_delay]", conf.ptt.start_delay);
    dprint("[ptt_output][end_delay]", conf.ptt.end_delay);
    dprint("[ptt_output][reverse]", conf.ptt.reverse);
    dprint("[location][latitude]", conf.location.latitude);
    dprint("[location][longitude]", conf.location.longitude);
    dprint("[location][altitude]", conf.location.altitude);
}
#endif
