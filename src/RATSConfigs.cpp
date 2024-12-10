/*
 *  This class manages configuration storage in EEPROM on RATS
 */

#include "RATSConfigs.h"
#include "StratoGroundPort.h"

RATSConfigs::RATSConfigs()
    : TeensyEEPROM(CONFIG_VERSION, BASE_ADDRESS),
    // ------------ Hard-Coded Config Defaults ------------
    // TODO Assign correct default values here
    sampleRateSecs(60),
    dataProcMethod(1),
    deployRevs(100),
    deploySpeed(10),
    retractRevs(100),
    retractSpeed(10),
    motorCurrentLimit(10),
    motorTorqueLimit(10),
    real_time_mcb(false)

    // ----------------------------------------------------
{ }

void RATSConfigs::RegisterAll()
{
    bool success = true;

    success &= Register(&sampleRateSecs);
    success &= Register(&dataProcMethod);
    success &= Register(&deployRevs);
    success &= Register(&deploySpeed);
    success &= Register(&retractRevs);
    success &= Register(&retractSpeed);
    success &= Register(&motorCurrentLimit);
    success &= Register(&motorTorqueLimit);

    if (!success) {
        debug_serial->println("Error registering EEPROM configs");
    }
}