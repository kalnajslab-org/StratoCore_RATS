#include "StratoRATS.h"

// The telecommand handler must return ACK/NAK
bool StratoRATS::TCHandler(Telecommand_t telecommand)
{
    // Set up the TC summary message
    String msg("TC reply undefined");
    String comma(",");
    LOG_LEVEL_t summary_level = LOG_NOMINAL;

    switch (telecommand) {
    case GETMCBEEPROM:
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Motion ongoing, request MCB EEPROM later");
        } else {
            mcbComm.TX_ASCII(MCB_GET_EEPROM);
            Serial.println("Request MCBEEPROM");
        }
        msg = String("TC get MCB EEPROM");
        break;
    case RATSSAMPERATESECS:
        Set_sampleRateSecs = ratsParam.sampleRateSecs;
        ratsConfigs.sampleRateSecs.Write(ratsParam.sampleRateSecs);
        msg = String("TC set sample rate")+comma+String(Set_sampleRateSecs);
        break;
    case RATSDATAPROCTYPE:
        Set_dataProcMethod = ratsParam.dataProcMethod;
        ratsConfigs.dataProcMethod.Write(ratsParam.dataProcMethod);
        msg = String("TC set processing mode")+comma+String(Set_dataProcMethod);
        break;
    case RATSDEPLOY:
        Set_deployRevs = ratsParam.deployRevs;
        Set_deploySpeed = ratsParam.deploySpeed;
        ratsConfigs.deployRevs.Write(ratsParam.deployRevs);
        ratsConfigs.deploySpeed.Write(ratsParam.deploySpeed);
        msg = String("TC ECU deploy")+comma+String(Set_deployRevs)+comma+String(Set_deploySpeed);
        break;
    case RATSRETRACT:
        Set_retractRevs = ratsParam.retractRevs;
        Set_retractSpeed = ratsParam.retractSpeed;
        ratsConfigs.retractRevs.Write(ratsParam.retractRevs);
        ratsConfigs.retractSpeed.Write(ratsParam.retractSpeed);
        msg = String("TC ECU retract")+comma+String(Set_retractRevs)+comma+String(Set_retractSpeed);
        break;
    case RATSHOME:
        Set_motorHome = true;
        msg = String("TC ECU Home");
        break;
    case RATSMOTORLIMITS:
        Set_motorCurrentLimit = ratsParam.motorCurrentLimit;
        Set_motorTorqueLimit = ratsParam.motorTorqueLimit;
        ratsConfigs.motorCurrentLimit.Write(ratsParam.motorCurrentLimit);
        ratsConfigs.motorTorqueLimit.Write(ratsParam.motorTorqueLimit);
        msg = String("TC motor limits")+comma+String(Set_motorCurrentLimit)+comma+String(Set_motorTorqueLimit);
        break;
    case RATSMOTORRESET:
        Set_motorReset = true;
        msg = String("TC motor reset");
        break;
    case GETRATSEEPROM:
        msg = String("TC get RATS EEPROM");
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Motion ongoing, request RATS EEPROM later");
        } else {
            SendRATSEEPROM();
        }
        break;

    default:
        msg = String("Unknown TC ") + String(telecommand) + String(" received");
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

