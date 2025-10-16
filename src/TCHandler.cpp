#include <ArduinoJson.h>
#include "StratoRATS.h"

static JsonDocument ecu_json;
static char ecu_json_str[ECU_LORA_DATA_BUFSIZE];

// The telecommand handler must return ACK/NAK
bool StratoRATS::TCHandler(Telecommand_t telecommand)
{
    // Set up the TC summary message
    String msg("Unhandled TC " + String(telecommand) + " received");
    LOG_LEVEL_t summary_level = LOG_NOMINAL;

    switch (telecommand) {
    // MCB Telecommands -----------------------------------
    case DEPLOYx:
        msg = "TC Deploy Length";
        if (inst_substate == FL_MEASURE) {
            deploy_length = mcbParam.deployLen;
            msg += ": " + String(deploy_length, 1) + " revs";
            SetAction(ACTION_REEL_OUT);
        } else {
            msg = "Cannot deploy, not in FL_MEASURE";
            summary_level = LOG_ERROR;
        }
        break;
    case DEPLOYv:
        msg = "TC Deploy Velocity: " + String(mcbParam.deployVel);
        ratsConfigs.deploy_velocity.Write(mcbParam.deployVel);
        break;
    case DEPLOYa:
        msg = "TC Deploy Acceleration: " + String(mcbParam.deployAcc);
        if (!mcbComm.TX_Out_Acc(mcbParam.deployAcc)) {
            msg = "Error sending deploy acc to MCB";
        }
        break;
    case RETRACTx:
        msg = "TC Retract Length";
        if (inst_substate == FL_MEASURE) {
            retract_length = mcbParam.retractLen;
            SetAction(ACTION_REEL_IN);
            msg +=  ": " + String(retract_length, 1) + " revs";
        } else {
            msg = "Cannot retract, not in FL_MEASURE";
            summary_level = LOG_ERROR;
        }
        break;
    case RETRACTv:
        msg = "TC Retract Velocity: " + String(mcbParam.retractVel);
        ratsConfigs.retract_velocity.Write(mcbParam.retractVel);
        break;
    case RETRACTa:
        msg = "TC Retract Acceleration: " + String(mcbParam.retractAcc);
        if (!mcbComm.TX_In_Acc(mcbParam.retractAcc)) {
            msg = "Error sending retract acc to MCB";
        }
        break;
    case FULLRETRACT:
        // todo: determine implementation
        msg = "TC Full Retract";
        break;
    case CANCELMOTION:
        msg = "TC Cancel Motion";
        mcbComm.TX_ASCII(MCB_CANCEL_MOTION); // no matter what, attempt to send (irrespective of mode)
        SetAction(ACTION_MOTION_STOP);
        break;
    case ZEROREEL:
        msg = "TC Zero Reel";
        if (mcb_motion_ongoing) {
            msg = "Can't zero reel, motion ongoing";
            summary_level = LOG_ERROR;
        } else {
            mcbComm.TX_ASCII(MCB_ZERO_REEL);
        }
        break;
    case TORQUELIMITS:
        msg = "TC Torque Limits";
        if (!mcbComm.TX_Torque_Limits(mcbParam.torqueLimits[0],mcbParam.torqueLimits[1])) {
            msg = "Error sending torque limits to MCB";
            summary_level = LOG_ERROR;
        }
        break;
    case CURRLIMITS:
        msg = "TC Current Limits";
        if (!mcbComm.TX_Curr_Limits(mcbParam.currLimits[0],mcbParam.currLimits[1])) {
            msg = "Error sending curr limits to MCB";
            summary_level = LOG_ERROR;
        }
        break;
    case IGNORELIMITS:
        msg = "TC Ignore Limits";
        mcbComm.TX_ASCII(MCB_IGNORE_LIMITS);
        break;
    case USELIMITS:
        msg = "TC Use Limits";
        mcbComm.TX_ASCII(MCB_USE_LIMITS);
        break;
    case GETMCBEEPROM:
        msg = "TC get MCB EEPROM";
        if (mcb_motion_ongoing) {
            msg = "Motion ongoing, request MCB EEPROM later";
            summary_level = LOG_ERROR;
        } else {
            // Request the MCB EEPROM. MCBRouter will handle the response
            mcbComm.TX_ASCII(MCB_GET_EEPROM);
        }
        break;
    case GETMCBVOLTS:
        msg = "TC get MCB voltages";
        mcbComm.TX_ASCII(MCB_GET_VOLTAGES);
        break;
    case CONTROLLERSON:
        msg = "TC MCB controllers on";
        mcbComm.TX_ASCII(MCB_CONTROLLERS_ON);
        break;
    case CONTROLLERSOFF:
        msg = "TC MCB controllers off";
        mcbComm.TX_ASCII(MCB_CONTROLLERS_OFF);
        break;

    // RATS Telecommands -----------------------------------
    case RATSDATAPROCTYPE:
        msg = "TC set processing mode" + String(ratsParam.data_proc_method);
        ratsConfigs.data_proc_method.Write(ratsParam.data_proc_method);
        break;
    case RATSREALTIMEMCBON:
        msg = "Enabled real-time MCB mode";
        if (mcb_motion_ongoing) {
            msg = "Cannot start real-time MCB mode, motion ongoing";
            summary_level = LOG_ERROR;
        } else {
            ratsConfigs.real_time_mcb.Write(true);
        }
        break;
    case RATSREALTIMEMCBOFF:
        msg = "Disabled real-time MCB mode";
        if (mcb_motion_ongoing) {
            msg = "Cannot start real-time MCB mode, motion ongoing";
            summary_level = LOG_ERROR;
        } else {
            ratsConfigs.real_time_mcb.Write(false);
        }
        break;
    case RATSLORATXTESTON:
        if (my_inst_mode != MODE_STANDBY) {
            msg = "TC Cannot start LoRa TX test, not in standby mode";
            summary_level = LOG_ERROR;
            break;
        }
        lora_tx_test = true;
        scheduler.AddAction(ACTION_LORA_TX_TEST, 1);
        msg = "TC LoRa TX test on";
        break;
    case RATSLORATXTESTOFF:
        lora_tx_test = false;
        msg = "TC LoRa TX test off";
        break;
    case RATSGETEEPROM:
        msg = "TC get RATS EEPROM";
        if (mcb_motion_ongoing) {
            msg = "Motion ongoing, request RATS EEPROM later";
            summary_level = LOG_ERROR;
        } else {
            SendRATSEEPROM();
        }
        break;
    case RATSECUTEMP:
        msg = "TC set ECU temp: " + String(ratsParam.ecu_tempC);
        // Save the ECU temp to EEPROM
        ratsConfigs.ecu_tempC.Write(ratsParam.ecu_tempC);
        if (my_inst_mode == MODE_FLIGHT || my_inst_mode == MODE_STANDBY) {
            ecu_json.clear();
            ecu_json["tempC"] = ratsConfigs.ecu_tempC.Read();
            serializeJson(ecu_json, ecu_json_str);
            // Don't forget that the message will not be sent until we receive a message from the ECU.
            // So it will not work to try to send two messages back-to-back.
            // Also keep in mind that the ECU might not even be powered up right now.
            ecu_lora_tx((uint8_t*)ecu_json_str, strlen(ecu_json_str));
        }
        break;
    case RATSECUPWRON:
        msg = "TC ECU power on";
        if (my_inst_mode == MODE_FLIGHT || my_inst_mode == MODE_STANDBY) {
            ECUControl(true);
        } else {
            msg = "Cannot power on ECU, not in FLIGHT or STANDBY mode";
            summary_level = LOG_ERROR;
        }
        break;
    case RATSECUPWROFF:
        msg = "TC ECU power off";
        // Turn off the ECU
        ECUControl(false);
        break;
    case RATSRS41REGEN:
        msg = "TC RS41 regen";
        ecu_json.clear();
        ecu_json["rs41Regen"] = true;
        serializeJson(ecu_json, ecu_json_str);
        // Don't forget that the message will not be sent until we receive a message from the ECU.
        // So it will not work to try to send two messages back-to-back.
        // Also keep in mind that the ECU might not even be powered up right now.
        ecu_lora_tx((uint8_t*)ecu_json_str, strlen(ecu_json_str));
        break;
    default:
        summary_level = LOG_ERROR;
        msg = "Unknown TC " + String(telecommand) + " received";
        break;
    }

    // Send TC summary to the StratoCore log and as a TM
    switch (summary_level) {
        case LOG_DEBUG:
            log_debug(msg.c_str());
            ZephyrLogFine(msg.c_str());
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

