#include "StratoRATS.h"

// The telecommand handler must return ACK/NAK
bool StratoRATS::TCHandler(Telecommand_t telecommand)
{
    // Set up the TC summary message
    String msg("Unhandled TC " + String(telecommand) + " received");
    String comma(",");
    LOG_LEVEL_t summary_level = LOG_NOMINAL;

    switch (telecommand) {
    // MCB Telecommands -----------------------------------
    case DEPLOYx:
        deploy_length = mcbParam.deployLen;
        SetAction(ACTION_REEL_OUT);
        msg = String("TC Deploy Length");
        break;
    case DEPLOYv:
        ratsConfigs.deploy_velocity.Write(mcbParam.deployVel);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set deploy_velocity: %f", ratsConfigs.deploy_velocity.Read());
        ZephyrLogFine(log_array);
        msg = String("TC Deploy Velocity");
        break;
    case DEPLOYa:
        if (!mcbComm.TX_Out_Acc(mcbParam.deployAcc)) {
            ZephyrLogWarn("Error sending deploy acc to MCB");
        }
        msg = String("TC Deploy Acceleration");
        break;
    case RETRACTx:
        retract_length = mcbParam.retractLen;
        SetAction(ACTION_REEL_IN); // will be ignored if wrong mode
        msg = String("TC Retract Length");
        break;
    case RETRACTv:
        ratsConfigs.retract_velocity.Write(mcbParam.retractVel);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set retract_velocity: %f", ratsConfigs.retract_velocity.Read());
        ZephyrLogFine(log_array);
        msg = String("TC Retract Velocity");
        break;
    case RETRACTa:
        if (!mcbComm.TX_In_Acc(mcbParam.retractAcc)) {
            ZephyrLogWarn("Error sending retract acc to MCB");
        }
        msg = String("TC Retract Acceleration");
        break;
    case FULLRETRACT:
        // todo: determine implementation
        msg = String("TC Full Retract");
        break;
    case CANCELMOTION:
        mcbComm.TX_ASCII(MCB_CANCEL_MOTION); // no matter what, attempt to send (irrespective of mode)
        SetAction(ACTION_MOTION_STOP);
        msg = String("TC Cancel Motion");
        break;
    case ZEROREEL:
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Can't zero reel, motion ongoing");
        }
        mcbComm.TX_ASCII(MCB_ZERO_REEL);
        msg = String("TC Zero Reel");
        break;
    case TORQUELIMITS:
        if (!mcbComm.TX_Torque_Limits(mcbParam.torqueLimits[0],mcbParam.torqueLimits[1])) {
            ZephyrLogWarn("Error sending torque limits to MCB");
        }
                msg = String("TC Torque Limits");
        break;
    case CURRLIMITS:
        if (!mcbComm.TX_Curr_Limits(mcbParam.currLimits[0],mcbParam.currLimits[1])) {
            ZephyrLogWarn("Error sending curr limits to MCB");
        }
        msg = String("TC Current Limits");
        break;
    case IGNORELIMITS:
        mcbComm.TX_ASCII(MCB_IGNORE_LIMITS);
        msg = String("TC Ignore Limits");
        break;
    case USELIMITS:
        mcbComm.TX_ASCII(MCB_USE_LIMITS);
        msg = String("TC Use Limits");
        break;
    case GETMCBEEPROM:
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Motion ongoing, request MCB EEPROM later");
        } else {
            // Request the MCB EEPROM. MCBRouter will handle the response
            mcbComm.TX_ASCII(MCB_GET_EEPROM);
        }
        msg = String("TC get MCB EEPROM");
        break;
    case GETMCBVOLTS:
        mcbComm.TX_ASCII(MCB_GET_VOLTAGES);
        msg = String("TC get MCB voltages");
        break;
    case RATSSAMPERATESECS:
        Set_sampleRateSecs = ratsParam.sample_rate_secs;
        ratsConfigs.sample_rate_secs.Write(ratsParam.sample_rate_secs);
        msg = String("TC set sample rate")+comma+String(Set_sampleRateSecs);
        break;
    case RATSDATAPROCTYPE:
        Set_dataProcMethod = ratsParam.data_proc_method;
        ratsConfigs.data_proc_method.Write(ratsParam.data_proc_method);
        msg = String("TC set processing mode")+comma+String(Set_dataProcMethod);
        break;
    case RATSDEPLOY:
        Set_deployRevs = ratsParam.deploy_revs;
        Set_deploySpeed = ratsParam.deploy_velocity;
        ratsConfigs.deploy_revs.Write(ratsParam.deploy_revs);
        ratsConfigs.deploy_velocity.Write(ratsParam.deploy_velocity);
        msg = String("TC ECU deploy")+comma+String(Set_deployRevs)+comma+String(Set_deploySpeed);
        break;
    case RATSRETRACT:
        Set_retractRevs = ratsParam.retract_revs;
        Set_retractSpeed = ratsParam.retract_velocity;
        ratsConfigs.retract_revs.Write(ratsParam.retract_revs);
        ratsConfigs.retract_velocity.Write(ratsParam.retract_velocity);
        msg = String("TC ECU retract")+comma+String(Set_retractRevs)+comma+String(Set_retractSpeed);
        break;
    case GETRATSEEPROM:
        msg = String("TC get RATS EEPROM");
        summary_level = LOG_NOMINAL;

        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Motion ongoing, request RATS EEPROM later");
        } else {
            SendRATSEEPROM();
        }
        break;

    default:
        summary_level = LOG_ERROR;
        msg = String("Unknown TC ") + String(telecommand) + String(" received");
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

