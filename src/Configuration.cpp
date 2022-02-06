#include <SPIFFS.h>
#include <logger.h>

#ifndef CPPCHECK
#include <ArduinoJson.h>
#endif

#include "Configuration.h"

const uint32_t Configuration::CONFIGURATION_DISPLAY_TIMEOUT;

ConfigurationManagement::ConfigurationManagement(const String &FilePath, const String &defaultFilePath)
{
    bool userFileIsValid = false;
    bool success = true;

    if (!SPIFFS.begin(true))
    {
        logPrintlnE("Mounting SPIFFS was not possible. Trying to format SPIFFS...");
        SPIFFS.format();
        if (!SPIFFS.begin())
        {
            logPrintlnE("Formating SPIFFS was not okay!");
            success = false;
        }
    }

    if (success)
    {
        File f = SPIFFS.open(FilePath);

        if (f != 0)
        {
            if (f.isDirectory() == false)
            {
                userFileIsValid = true;
            }

            f.close();
        }
    }

    m_FilePath = (userFileIsValid ? FilePath : defaultFilePath);
}

// cppcheck-suppress unusedFunction
Configuration ConfigurationManagement::readConfiguration()
{
    File file = SPIFFS.open(m_FilePath);

    if (!file)
    {
        logPrintlnE("Failed to open file for reading...");
        return Configuration();
    }


    DynamicJsonDocument  data(2048);
    DeserializationError error = deserializeJson(data, file);

    if (error != DeserializationError::Ok)
    {
        logPrintlnE("Failed to read file, using default configuration.");
    }

    file.close();

    Configuration conf;

    conf.debug                          = data["debug"] | false;

    // process lora first, as power will be needed for beacon's lora_power
    if (data.containsKey("lora"))
    {
        conf.lora.frequencyRx           = data["lora"]["frequency_rx"] | 433775000;
        conf.lora.frequencyTx           = data["lora"]["frequency_tx"] | 433775000;
        conf.lora.power                 = data["lora"]["power"] | 20;
        conf.lora.spreadingFactor       = data["lora"]["spreading_factor"] | 12;
        conf.lora.signalBandwidth       = data["lora"]["signal_bandwidth"] | 125000;
        conf.lora.codingRate4           = data["lora"]["coding_rate4"] | 5;
    }

    JsonArray beacons = data["beacons"].as<JsonArray>();
    for (JsonVariant v : beacons)
    {
        Configuration::Beacon beacon;

        if (v.containsKey("callsign"))
        {
            beacon.callsign             = v["callsign"].as<String>();
        }

        if (v.containsKey("path"))
        {
            beacon.path                 = v["path"].as<String>();
        }

        if (v.containsKey("message"))
        {
            beacon.message              = v["message"].as<String>();
        }

        beacon.add_power                = v["add_power"] | true;
        beacon.timeout = v["timeout"] | 1;

        if (v.containsKey("symbol"))
        {
            beacon.symbol               = v["symbol"].as<String>();
        }

        if (v.containsKey("overlay"))
        {
            beacon.overlay              = v["overlay"].as<String>();
        }

        beacon.smart_beacon.active      = v["smart_beacon"]["active"] | false;
        beacon.smart_beacon.turn_min    = v["smart_beacon"]["turn_min"] | 25;
        beacon.smart_beacon.slow_rate   = v["smart_beacon"]["slow_rate"] | 300;
        beacon.smart_beacon.slow_speed  = v["smart_beacon"]["slow_speed"] | 10;
        beacon.smart_beacon.fast_rate   = v["smart_beacon"]["fast_rate"] | 60;
        beacon.smart_beacon.fast_speed  = v["smart_beacon"]["fast_speed"] | 100;
        beacon.smart_beacon.min_tx_dist = v["smart_beacon"]["min_tx_dist"] | 100;
        beacon.smart_beacon.min_bcn     = v["smart_beacon"]["min_bcn"] | 5;

        beacon.enhance_precision        = v["enhance_precision"] | false;
        beacon.lora_power               = v["lora_power"] | conf.lora.power;

        conf.beacons.push_back(beacon);
    }

    if (data.containsKey("button"))
    {
        conf.button.tx                      = data["button"]["tx"] | false;
        conf.button.alt_message             = data["button"]["alt_message"] | false;
    }

    if (data.containsKey("ptt_output"))
    {
        conf.ptt.active                 = data["ptt_output"]["active"] | false;
        conf.ptt.io_pin                 = data["ptt_output"]["io_pin"] | 4;
        conf.ptt.start_delay            = data["ptt_output"]["start_delay"] | 0;
        conf.ptt.end_delay              = data["ptt_output"]["end_delay"] | 0;
        conf.ptt.reverse                = data["ptt_output"]["reverse"] | false;
    }

    if (data.containsKey("location"))
    {
        conf.location.latitude          = data["location"]["latitude"] | 0.0;
        conf.location.longitude         = data["location"]["longitude"] | 0.0;
        conf.location.altitude          = data["location"]["altitude"] | 0;

        if (data["location"].containsKey("symbol"))
        {
            conf.location.symbol        = data["location"]["symbol"].as<String>();
        }

        if (data["location"].containsKey("overlay"))
        {
            conf.location.overlay       = data["location"]["overlay"].as<String>();
        }

        if (data["location"].containsKey("message"))
        {
            conf.location.message       = data["location"]["message"].as<String>();
        }
    }

    if (data.containsKey("display"))
    {
        conf.display.invert             = data["display"]["invert"] | false;
        conf.display.rotation           = data["display"]["rotation"] | 0;
        conf.display.contrast           = data["display"]["contrast"] | 0xCF;
        conf.display.timeout            = data["display"]["timeout"] | Configuration::CONFIGURATION_DISPLAY_TIMEOUT;
    }

    return conf;
}

// cppcheck-suppress unusedFunction
void ConfigurationManagement::writeConfiguration(Configuration conf)
{
    File file = SPIFFS.open(m_FilePath, "w");

    if (!file)
    {
        logPrintlnE("Failed to open file for writing...");
        return;
    }

    DynamicJsonDocument data(2048);

    JsonArray beacons = data.createNestedArray("beacons");
    for (Configuration::Beacon beacon : conf.beacons) {
        JsonObject v = beacons.createNestedObject();

        v["callsign"]                    = beacon.callsign;
        v["path"]                        = beacon.path;
        v["message"]                     = beacon.message;
        v["add_power"]                   = beacon.add_power;
        v["timeout"]                     = beacon.timeout;
        v["symbol"]                      = beacon.symbol;
        v["overlay"]                     = beacon.overlay;

        v["smart_beacon"]["active"]      = beacon.smart_beacon.active;
        v["smart_beacon"]["turn_min"]    = beacon.smart_beacon.turn_min;
        v["smart_beacon"]["slow_rate"]   = beacon.smart_beacon.slow_rate;
        v["smart_beacon"]["slow_speed"]  = beacon.smart_beacon.slow_speed;
        v["smart_beacon"]["fast_rate"]   = beacon.smart_beacon.fast_rate;
        v["smart_beacon"]["fast_speed"]  = beacon.smart_beacon.fast_speed;
        v["smart_beacon"]["min_tx_dist"] = beacon.smart_beacon.min_tx_dist;
        v["smart_beacon"]["min_bcn"]     = beacon.smart_beacon.min_bcn;

        v["enhance_precision"]           = beacon.enhance_precision;
        v["lora_power"]                  = beacon.lora_power;
    }

    data["debug"]                       = conf.debug;

    data["button"]["tx"]                = conf.button.tx;
    data["button"]["alt_message"]       = conf.button.alt_message;

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
    data["location"]["message"]         = conf.location.message;

    data["display"]["invert"]           = conf.display.invert;
    data["display"]["rotation"]         = conf.display.rotation;
    data["display"]["contrast"]         = conf.display.contrast;
    data["display"]["timeout"]          = conf.display.timeout;

    serializeJson(data, file);
    file.close();
}
