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
        log_nominal("TC: Set sample rate");
        ZephyrLogFine("TC: Set sample rate");
        break;
    case RATSDATAPROCTYPE:
        log_nominal("TC: Set processing mode");
        ZephyrLogFine("TC: Set processing mode");
        break;
    case RATSTSENONOFF:
        log_nominal("TC: TSEN enable");
        ZephyrLogFine("TC: TSEN enable");
        break;
    case RATSRS41ONOFF:
        log_nominal("TC: RS41 enable");
        ZephyrLogFine("TC: RS41 enable");
        break;
    case RATSRS41REGEN:
        log_nominal("TC: RS41 regen");
        ZephyrLogFine("TC: RS41 regen");
        break;
    case RATSDEPLOY:
        log_nominal("TC: ECU deploy");
        ZephyrLogFine("TC: ECU deploy");
        break;
    case RATSRETRACT:
        log_nominal("TC: ECU retract");
        ZephyrLogFine("TC: ECU retract");
        break;
    case RATSHOME:
        log_nominal("TC: ECU Home");
        ZephyrLogFine("TC: ECU Home");
        break;
    case RATSMOTORLIMITS:
        log_nominal("TC: Motor limits");
        ZephyrLogFine("TC: Motor limits");
        break;
    case RATSMOTORRESET:
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
