#ifndef BEACON_MANAGER_H_
#define BEACON_MANAGER_H_

#include "Configuration.h"

class BeaconManager
{
    public:
        BeaconManager();

        void loadConfig(const std::list<Configuration::Beacon> &beaconConfig);

        std::list<Configuration::Beacon>::iterator getCurrentBeaconConfig() const;
        void                                       loadNextBeacon();

    private:
        std::list<Configuration::Beacon>           m_beaconConfig;
        std::list<Configuration::Beacon>::iterator m_currentBeaconConfig;
};

#endif
