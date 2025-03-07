/*
 *  This class manages configuration storage in EEPROM on RATS
 */

#include "RATSConfigs.h"
#include "StratoGroundPort.h"

RATSConfigs::RATSConfigs()
    : TeensyEEPROM(CONFIG_VERSION, BASE_ADDRESS),
    // ------------ Hard-Coded Config Defaults ------------
    // TODO Assign correct default values here
    data_proc_method(1),
    deploy_velocity(10.0f),
    retract_velocity(10.0f),
    motion_timeout(10),
    real_time_mcb(false)

    // ----------------------------------------------------
{ }

void RATSConfigs::RegisterAll()
{
    // Called from the base class Initialize method,
    // this method registers all EEPROMData objects
    // and thus determines the length of the bufferized data

    bool success = true;

    success &= Register(&data_proc_method);
    success &= Register(&deploy_velocity);
    success &= Register(&retract_velocity);
    success &= Register(&motion_timeout);
    success &= Register(&real_time_mcb);

    if (!success) {
        debug_serial->println("Error registering EEPROM configs");
    }
}