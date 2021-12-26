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

    conf.debug                        = data["debug"].as<bool>() | false;
    conf.enhance_precision            = data["enhance_precision"].as<bool>() | false;
    conf.display_timeout              = data["display_timeout"].as<uint32_t>() | Configuration::CONFIGURATION_DISPLAY_TIMEOUT;

    if (data.containsKey("beacon"))
    {
        if (data["beacon"].containsKey("message"))
        {
            conf.beacon.message       = data["beacon"]["message"].as<String>();
        }

        conf.beacon.timeout = data["beacon"]["timeout"].as<uint32_t>() | 1;

        if (data["beacon"].containsKey("symbol"))
        {
            conf.beacon.symbol        = data["beacon"]["symbol"].as<String>();
        }

        if (data["beacon"].containsKey("overlay"))
        {
            conf.beacon.overlay       = data["beacon"]["overlay"].as<String>();
        }

        if (data["beacon"].containsKey("button_tx"))
        {
            conf.beacon.button_tx     = data["beacon"]["button_tx"].as<bool>() | false;
        }
    }

    if (data.containsKey("smart_beacon"))
    {
        conf.smart_beacon.active      = data["smart_beacon"]["active"].as<bool>() | false;
        conf.smart_beacon.turn_min    = data["smart_beacon"]["turn_min"].as<uint32_t>() | 25;
        conf.smart_beacon.slow_rate   = data["smart_beacon"]["slow_rate"].as<uint32_t>() | 300;
        conf.smart_beacon.slow_speed  = data["smart_beacon"]["slow_speed"].as<uint32_t>() | 10;
        conf.smart_beacon.fast_rate   = data["smart_beacon"]["fast_rate"].as<uint32_t>() | 60;
        conf.smart_beacon.fast_speed  = data["smart_beacon"]["fast_speed"].as<uint32_t>() | 100;
        conf.smart_beacon.min_tx_dist = data["smart_beacon"]["min_tx_dist"].as<uint32_t>() | 100;
        conf.smart_beacon.min_bcn     = data["smart_beacon"]["min_bcn"].as<uint32_t>() | 5;
    }

    if (data.containsKey("lora"))
    {
        conf.lora.frequencyRx         = data["lora"]["frequency_rx"].as<unsigned long>() | 433775000;
        conf.lora.frequencyTx         = data["lora"]["frequency_tx"].as<unsigned long>() | 433775000;
        conf.lora.power               = data["lora"]["power"].as<uint32_t>() | 20;
        conf.lora.spreadingFactor     = data["lora"]["spreading_factor"].as<uint32_t>() | 12;
        conf.lora.signalBandwidth     = data["lora"]["signal_bandwidth"].as<unsigned long>() | 125000;
        conf.lora.codingRate4         = data["lora"]["coding_rate4"].as<uint32_t>() | 5;
    }

    if (data.containsKey("ptt_output"))
    {
        conf.ptt.active               = data["ptt_output"]["active"].as<bool>() | false;
        conf.ptt.io_pin               = data["ptt_output"]["io_pin"].as<uint8_t>() | 4;
        conf.ptt.start_delay          = data["ptt_output"]["start_delay"].as<uint32_t>() | 0;
        conf.ptt.end_delay            = data["ptt_output"]["end_delay"].as<uint32_t>() | 0;
        conf.ptt.reverse              = data["ptt_output"]["reverse"].as<bool>() | false;
    }

    if (data.containsKey("location"))
    {
        conf.location.latitude        = data["location"]["latitude"].as<double>();
        conf.location.longitude       = data["location"]["longitude"].as<double>();
        conf.location.altitude        = data["location"]["altitude"].as<uint32_t>();
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
    data["beacon"]["symbol"]            = conf.beacon.symbol;
    data["beacon"]["overlay"]           = conf.beacon.overlay;
    data["beacon"]["button_tx"]         = conf.beacon.button_tx;
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

    serializeJson(data, file);
    file.close();
}
