/*
 *  RATSConfigs.h
 *  Author:  Alex St. Clair
 *  Created: April 2020
 *
 *  This class manages configuration storage in EEPROM on the PIB
 *
 *  To add a configuration value:
 *    1) Add a public EEPROMData<T> object in the header file
 *    2) Set the hard-coded backup value in the constructor
 *    3) Register the object in the RegisterAll method
 *    *note* maintain the order of objects in all three locations
 */

#ifndef RATSCONFIG_H
#define RATSCONFIG_H

#include "TeensyEEPROM.h"

class RATSConfigs : public TeensyEEPROM {
private:
    void RegisterAll();

public:
    RATSConfigs();

    // constants, manually change version number here to force update
    static const uint16_t CONFIG_VERSION = 0x0007;
    static const uint16_t BASE_ADDRESS = 0x0000;

    // ------------------ Configurations ------------------
    EEPROMData<uint16_t> sample_rate_secs;
    EEPROMData<uint16_t> data_proc_method;
    EEPROMData<float> deploy_velocity;   // revs/min
    EEPROMData<float> retract_velocity;  // revs/min
    EEPROMData<uint16_t> motion_timeout;
    // MCB TM mode
    EEPROMData<bool> real_time_mcb;
    // ----------------------------------------------------

};

#endif /* RATSCONFIG_H */