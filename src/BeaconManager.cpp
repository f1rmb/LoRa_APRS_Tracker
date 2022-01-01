#include "BeaconManager.h"

BeaconManager::BeaconManager() :
m_currentBeaconConfig(m_beaconConfig.end())
{
}

// cppcheck-suppress unusedFunction
void BeaconManager::loadConfig(const std::list<Configuration::Beacon> &beaconConfig)
{
    m_beaconConfig        = beaconConfig;
    m_currentBeaconConfig = m_beaconConfig.begin();
}

// cppcheck-suppress unusedFunction
std::list<Configuration::Beacon>::iterator BeaconManager::getCurrentBeaconConfig() const
{
    return m_currentBeaconConfig;
}

// cppcheck-suppress unusedFunction
void BeaconManager::loadNextBeacon()
{
    m_currentBeaconConfig++;

    if (m_currentBeaconConfig == m_beaconConfig.end())
    {
        m_currentBeaconConfig = m_beaconConfig.begin();
    }
}
