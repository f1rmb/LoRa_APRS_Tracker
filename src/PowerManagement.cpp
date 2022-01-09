
#include "PowerManagement.h"

// cppcheck-suppress uninitMemberVar
PowerManagement::PowerManagement()
{
}

// cppcheck-suppress unusedFunction
bool PowerManagement::begin(TwoWire &port)
{
    bool result = (axp.begin(port, AXP192_SLAVE_ADDRESS) == AXP_PASS);

    if (result)
    {
        result = (axp.setDCDC1Voltage(3300) == AXP_PASS);
    }

    return result;
}

// cppcheck-suppress unusedFunction
void PowerManagement::LoRaActivate()
{
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
}

// cppcheck-suppress unusedFunction
void PowerManagement::LoRaDeactivate()
{
    axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
}

// cppcheck-suppress unusedFunction
bool PowerManagement::isLoRaActivated()
{
    return axp.isLDO2Enable();
}

// cppcheck-suppress unusedFunction
void PowerManagement::GPSActivate()
{
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
}

// cppcheck-suppress unusedFunction
void PowerManagement::GPSDeactivate()
{
    axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
}

// cppcheck-suppress unusedFunction
bool PowerManagement::isGPSActivated()
{
    return axp.isLDO3Enable();
}

// cppcheck-suppress unusedFunction
void PowerManagement::OLEDActivate()
{
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
}

// cppcheck-suppress unusedFunction
void PowerManagement::OLEDDeactivate()
{
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
}

// cppcheck-suppress unusedFunction
bool PowerManagement::isOLEDActivated()
{
    return axp.isDCDC1Enable();
}

// cppcheck-suppress unusedFunction
void PowerManagement::MeasurementsActivate()
{
    axp.adc1Enable(AXP202_BATT_CUR_ADC1 | AXP202_BATT_VOL_ADC1, true);
}

// cppcheck-suppress unusedFunction
void PowerManagement::MeasurementsDeactivate()
{
    axp.adc1Enable(AXP202_BATT_CUR_ADC1 | AXP202_BATT_VOL_ADC1, false);
}

// cppcheck-suppress unusedFunction
double PowerManagement::getBatteryVoltage()
{
    return axp.getBattVoltage() / 1000.0;
}

// cppcheck-suppress unusedFunction
double PowerManagement::getBatteryChargeDischargeCurrent()
{
    if (axp.isChargeing())
    {
        return axp.getBattChargeCurrent();
    }

    return -1.0 * axp.getBattDischargeCurrent();
}

bool PowerManagement::isBatteryConnected()
{
    return axp.isBatteryConnect();
}
