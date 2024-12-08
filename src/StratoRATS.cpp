#include "StratoRATS.h"

StratoRATS::StratoRATS()
    : StratoCore(&ZEPHYR_SERIAL, INSTRUMENT)
{
}

void StratoRATS::InstrumentSetup()
{   

}

void StratoRATS::InstrumentLoop()
{
    WatchFlags();
}

// The telecommand handler must return ACK/NAK
bool StratoRATS::TCHandler(Telecommand_t telecommand)
{
    String dbg_msg = "";

    switch (telecommand) {
    case RATSSAMPERATESECS:
        Set_sampleRateSecs = ratsParam.sampleRateSecs;
        log_nominal("TC: Set sample rate");
        ZephyrLogFine("TC: Set sample rate");
        break;
    case RATSDATAPROCTYPE:
        Set_dataProcMethod = ratsParam.dataProcMethod;
        log_nominal("TC: Set processing mode");
        ZephyrLogFine("TC: Set processing mode");
        break;
    case RATSTSENONOFF:
        Set_tsenOn = ratsParam.tsenOn;
        log_nominal("TC: TSEN enable");
        ZephyrLogFine("TC: TSEN enable");
        break;
    case RATSRS41ONOFF:
        Set_rs41On = ratsParam.rs41On;
        log_nominal("TC: RS41 enable");
        ZephyrLogFine("TC: RS41 enable");
        break;
    case RATSRS41REGEN:
        Set_rs41regen = true;
        log_nominal("TC: RS41 regen");
        ZephyrLogFine("TC: RS41 regen");
        break;
    case RATSDEPLOY:
        Set_deployRevs = ratsParam.deployRevs;
        Set_deploySpeed = ratsParam.deploySpeed;
        log_nominal("TC: ECU deploy");
        ZephyrLogFine("TC: ECU deploy");
        break;
    case RATSRETRACT:
        Set_retractRevs = ratsParam.retractRevs;
        Set_retractSpeed = ratsParam.retractSpeed;
        log_nominal("TC: ECU retract");
        ZephyrLogFine("TC: ECU retract");
        break;
    case RATSHOME:
        Set_motorHome = true;
        log_nominal("TC: ECU Home");
        ZephyrLogFine("TC: ECU Home");
        break;
    case RATSMOTORLIMITS:
        Set_motorCurrentLimit = ratsParam.motorCurrentLimit;
        Set_motorTorqueLimit = ratsParam.motorTorqueLimit;
        log_nominal("TC: Motor limits");
        ZephyrLogFine("TC: Motor limits");
        break;
    case RATSMOTORRESET:
        Set_motorReset = true;
        log_nominal("TC: Motor reset");
        ZephyrLogFine("TC: Motor reset");
        break;
    default:
        ZephyrLogWarn("Unknown TC received");
        break;
    }
    return true;
}

void StratoRATS::ActionHandler(uint8_t action)
{
    // for safety, ensure index doesn't exceed array size
    if (action >= NUM_ACTIONS) {
        log_error("Out of bounds action flag access");
        return;
    }

    // set the flag and reset the stale count
    action_flags[action].flag_value = true;
    action_flags[action].stale_count = 0;
}

bool StratoRATS::CheckAction(uint8_t action)
{
    // for safety, ensure index doesn't exceed array size
    if (action >= NUM_ACTIONS) {
        log_error("Out of bounds action flag access");
        return false;
    }

    // check and clear the flag if it is set, return the value
    if (action_flags[action].flag_value) {
        action_flags[action].flag_value = false;
        action_flags[action].stale_count = 0;
        return true;
    } else {
        return false;
    }
}

void StratoRATS::WatchFlags()
{
    // monitor for and clear stale flags
    for (int i = 0; i < NUM_ACTIONS; i++) {
        if (action_flags[i].flag_value) {
            action_flags[i].stale_count++;
            if (action_flags[i].stale_count >= FLAG_STALE) {
                action_flags[i].flag_value = false;
                action_flags[i].stale_count = 0;
            }
        }
    }
}

void StratoRATS::RATS_Shutdown()
{
}
