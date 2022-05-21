#ifndef POWER_MANAGEMENT_H_
#define POWER_MANAGEMENT_H_

#include <Arduino.h>
#include <axp20x.h>

class PowerManagement {
    public:
        PowerManagement();
        bool begin(TwoWire &port);

        void Tick();

        void LoRaActivate();
        void LoRaDeactivate();
        bool isLoRaActivated();

        void GPSActivate();
        void GPSDeactivate();
        bool isGPSActivated();

        void OLEDActivate();
        void OLEDDeactivate();
        bool isOLEDActivated();

        void MeasurementsActivate();
        void MeasurementsDeactivate();

        double getBatteryVoltage();
        double getBatteryChargeDischargeCurrent();

        bool isBatteryConnected();

        bool isCharging();
        void setChargingLED(bool charging);


    private:
        AXP20X_Class axp;
        bool         ledState;
};

#endif
