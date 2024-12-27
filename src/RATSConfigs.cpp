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
    deploy_velocity(250.0f),
    retract_velocity(250.0f),
    retractRevs(100),
    deployRevs(100),
    motion_timeout(30),
    real_time_mcb(false)

    // ----------------------------------------------------
{ }

void RATSConfigs::RegisterAll()
{
    // Called from the base class Initialize method,
    // this method registers all EEPROMData objects
    // and thus determines the length of the bufferized data

    bool success = true;

    success &= Register(&sampleRateSecs);
    success &= Register(&dataProcMethod);
    success &= Register(&deployRevs);
    success &= Register(&deploy_velocity);
    success &= Register(&retractRevs);
    success &= Register(&retract_velocity);
    success &= Register(&real_time_mcb);

    if (!success) {
        debug_serial->println("Error registering EEPROM configs");
    }
}