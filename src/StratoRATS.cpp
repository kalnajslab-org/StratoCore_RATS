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

    // Set up the TC summary message
    String msg;
    String comma(",");
    LOG_LEVEL_t summary_level = LOG_NOMINAL;

    switch (telecommand) {
    case RATSSAMPERATESECS:
        Set_sampleRateSecs = ratsParam.sampleRateSecs;
        msg = (String("TC set sample rate")+comma+String(Set_sampleRateSecs));
        break;
    case RATSDATAPROCTYPE:
        Set_dataProcMethod = ratsParam.dataProcMethod;
        msg = (String("TC set processing mode")+comma+String(Set_dataProcMethod));
        break;
    case RATSTSENONOFF:
        Set_tsenOn = ratsParam.tsenOn;
        msg = (String("TC TSEN enable")+comma+String(Set_tsenOn));
        break;
    case RATSRS41ONOFF:
        Set_rs41On = ratsParam.rs41On;
        msg = (String("TC RS41 enable")+comma+String(Set_rs41On));
        break;
    case RATSRS41REGEN:
        Set_rs41regen = true;
        msg = String("TC RS41 regen");
        break;
    case RATSDEPLOY:
        Set_deployRevs = ratsParam.deployRevs;
        Set_deploySpeed = ratsParam.deploySpeed;
        msg = (String("TC ECU deploy")+comma+String(Set_deployRevs)+comma+String(Set_deploySpeed));
        break;
    case RATSRETRACT:
        Set_retractRevs = ratsParam.retractRevs;
        Set_retractSpeed = ratsParam.retractSpeed;
        msg = (String("TC ECU retract")+comma+String(Set_retractRevs)+comma+String(Set_retractSpeed));
        break;
    case RATSHOME:
        Set_motorHome = true;
        msg = String("TC ECU Home");
        break;
    case RATSMOTORLIMITS:
        Set_motorCurrentLimit = ratsParam.motorCurrentLimit;
        Set_motorTorqueLimit = ratsParam.motorTorqueLimit;
        msg = (String("TC motor limits")+comma+String(Set_motorCurrentLimit)+comma+String(Set_motorTorqueLimit));
        break;
    case RATSMOTORRESET:
        Set_motorReset = true;
        msg = String("TC motor reset");
        break;
    default:
        msg = String("Unknown TC received");
        summary_level = LOG_ERROR;
        break;
    }

    // Send TC summary to the StratoCore log and as a TM
    switch (summary_level) {
        case LOG_DEBUG:
            log_debug(msg.c_str());
            break;
        case LOG_NOMINAL:
            log_nominal(msg.c_str());
            ZephyrLogFine(msg.c_str());
            break;
        case LOG_ERROR:
            log_error(msg.c_str());
            ZephyrLogWarn(msg.c_str());
            break;
        default:
            log_error(msg.c_str());
            ZephyrLogWarn(msg.c_str());
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
