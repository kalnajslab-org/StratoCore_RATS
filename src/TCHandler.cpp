#include <ArduinoJson.h>
#include "StratoRATS.h"

static JsonDocument ecu_json;
static char ecu_json_str[ECU_LORA_DATA_BUFSIZE];

void sendEcuJson(uint8_t paired_ecu) {
    // Serialize and send the ECU JSON message.
    // Note: ecu_json must already be populated.
    serializeJson(ecu_json, ecu_json_str);
    // Don't forget that the message will not be sent until we receive a message from the ECU.
    // So it will not work to try to send two messages back-to-back.
    // Also keep in mind that the ECU might not even be powered up right now.

    std::array<uint8_t, ECU_LORA_DATA_BUFSIZE> payload = {0};

    int json_len = strlen(ecu_json_str);

    // Set the first byte to zero
    payload[0] = 0;

    // Set the second byte to paired_ecu
    payload[1] = static_cast<uint8_t>(paired_ecu);

    // Copy the command string into the payload starting at byte 2
    for (int i = 0; i < json_len && (i + 2) < ECU_LORA_DATA_BUFSIZE; i++) {
        payload[i + 2] = static_cast<uint8_t>(ecu_json_str[i]);
    }
    ecu_lora_tx(payload.data(), json_len + 2);
}

// The telecommand handler must return ACK/NAK
bool StratoRATS::TCHandler(Telecommand_t telecommand)
{
    // Set up the TC summary message
    String msg2("");
    String msg3("");

    StateFlag_t msg1_flag = FINE;
    bool send_rats_eeprom = false;
    bool send_mcm_eeprom = false;

    switch (telecommand) {
    // MCB Telecommands -----------------------------------
    case DEPLOYx:
        msg2 = "TC Deploy Length";
        if (inst_substate == FL_MEASURE) {
            deploy_length = mcbParam.deployLen;
            msg2 += ": " + String(deploy_length, 1) + " revs";
            SetAction(ACTION_REEL_OUT);
        } else {
            msg3 = "Cannot deploy, not in FL_MEASURE";
            msg1_flag = WARN;
        }
        break;
    case DEPLOYv:
        msg2 = "TC Deploy Velocity: " + String(mcbParam.deployVel);
        ratsConfigs.deploy_velocity.Write(mcbParam.deployVel);
        break;
    case DEPLOYa:
        msg2 = "TC Deploy Acceleration: " + String(mcbParam.deployAcc);
        if (!mcbComm.TX_Out_Acc(mcbParam.deployAcc)) {
            msg3 = "Error sending deploy acc to MCB";
        }
        break;
    case RETRACTx:
        msg2 = "TC Retract Length";
        if (inst_substate == FL_MEASURE) {
            retract_length = mcbParam.retractLen;
            SetAction(ACTION_REEL_IN);
            msg2 +=  ": " + String(retract_length, 1) + " revs";
        } else {
            msg3 = "Cannot retract, not in FL_MEASURE";
            msg1_flag = WARN;
        }
        break;
    case RETRACTv:
        msg2 = "TC Retract Velocity: " + String(mcbParam.retractVel);
        ratsConfigs.retract_velocity.Write(mcbParam.retractVel);
        break;
    case RETRACTa:
        msg2 = "TC Retract Acceleration: " + String(mcbParam.retractAcc);
        if (!mcbComm.TX_In_Acc(mcbParam.retractAcc)) {
            msg3 = "Error sending retract acc to MCB";
            msg1_flag = WARN;
        }
        break;
    case FULLRETRACT:
        // todo: determine implementation
        msg2 = "TC Full Retract";
        msg3 = "TC Full Retract not implemented";
        msg1_flag = WARN;
        break;
    case CANCELMOTION:
        msg2 = "TC Cancel Motion";
        mcbComm.TX_ASCII(MCB_CANCEL_MOTION); // no matter what, attempt to send (irrespective of mode)
        SetAction(ACTION_MOTION_STOP);
        break;
    case ZEROREEL:
        msg2 = "TC Zero Reel";
        if (mcb_motion_ongoing) {
            msg3 = "Can't zero while reel is in motion";
            msg1_flag = WARN;
        } else {
            mcbComm.TX_ASCII(MCB_ZERO_REEL);
        }
        break;
    case TORQUELIMITS:
        msg2 = "TC Torque Limits";
        if (!mcbComm.TX_Torque_Limits(mcbParam.torqueLimits[0],mcbParam.torqueLimits[1])) {
            msg3 = "Error sending torque limits to MCB";
            msg1_flag = CRIT;
        }
        break;
    case CURRLIMITS:
        msg2 = "TC Current Limits";
        if (!mcbComm.TX_Curr_Limits(mcbParam.currLimits[0],mcbParam.currLimits[1])) {
            msg3 = "Error sending curr limits to MCB";
            msg1_flag = CRIT;
        }
        break;
    case IGNORELIMITS:
        msg2 = "TC Ignore Limits";
        mcbComm.TX_ASCII(MCB_IGNORE_LIMITS);
        break;
    case USELIMITS:
        msg2 = "TC Use Limits";
        mcbComm.TX_ASCII(MCB_USE_LIMITS);
        break;
    case GETMCBEEPROM:
        msg2 = "TC get MCB EEPROM";
        if (mcb_motion_ongoing) {
            msg3 = "Can't get MCB EEPROM while reel is in motion";
            msg1_flag = WARN;
        } else {
            send_mcm_eeprom = true;
        }
        break;
    case GETMCBVOLTS:
        msg2 = "TC get MCB voltages";
        mcbComm.TX_ASCII(MCB_GET_VOLTAGES);
        break;
    case CONTROLLERSON:
        msg2 = "TC MCB controllers on";
        mcbComm.TX_ASCII(MCB_CONTROLLERS_ON);
        break;
    case CONTROLLERSOFF:
        msg2 = "TC MCB controllers off";
        mcbComm.TX_ASCII(MCB_CONTROLLERS_OFF);
        break;

    // RATS Telecommands -----------------------------------
    case RATSDATAPROCTYPE:
        msg2 = "TC set processing mode" + String(ratsParam.data_proc_method);
        ratsConfigs.data_proc_method.Write(ratsParam.data_proc_method);
        break;
    case RATSREALTIMEMCBON:
        msg2 = "Enabled real-time MCB mode";
        if (mcb_motion_ongoing) {
            msg3 = "Can't start real-time MCB mode while reel is in motion";
            msg1_flag = WARN;
        } else {
            ratsConfigs.real_time_mcb.Write(true);
        }
        break;
    case RATSREALTIMEMCBOFF:
        msg2 = "Disabled real-time MCB mode";
        if (mcb_motion_ongoing) {
            msg3 = "Can't start real-time MCB mode off while reel is in motion";
            msg1_flag = WARN;
        } else {
            ratsConfigs.real_time_mcb.Write(false);
        }
        break;
    case RATSLORATXTESTON:
        msg2 = "TC LoRa TX test on";
        if (my_inst_mode != MODE_STANDBY) {
            msg3 = "Cannot start LoRa TX test, not in standby mode";
            msg1_flag = WARN;
            break;
        }
        lora_tx_test = true;
        scheduler.AddAction(ACTION_LORA_TX_TEST, 1);
        break;
    case RATSLORATXTESTOFF:
        lora_tx_test = false;
        msg2 = "TC LoRa TX test off";
        break;
    case RATSGETEEPROM:
        msg2 = "TC get RATS EEPROM";
        send_rats_eeprom = true;
        break;
    case RATSECUTEMP:
        msg2 = "TC set ECU temp: " + String(ratsParam.ecu_tempC);
        // Save the ECU temp to EEPROM
        ratsConfigs.ecu_tempC.Write(ratsParam.ecu_tempC);
        if (IsECUPowerEnabled()) {
            ecu_json.clear();
            ecu_json["tempC"] = ratsConfigs.ecu_tempC.Read();
            sendEcuJson(paired_ecu);
        } else {
            msg3 = "Cannot send ECU temp, ECU power is off";
            msg1_flag = WARN;
        }
        break;
    case RATSECUPWRON:
        msg2 = "TC ECU power on";
        if (my_inst_mode == MODE_FLIGHT || my_inst_mode == MODE_STANDBY) {
            ECUPowerControl(true);
        } else {
            msg3 = "Cannot power on ECU, not in FLIGHT or STANDBY mode";
            msg1_flag = WARN;
        }
        break;
    case RATSECUPWROFF:
        msg2 = "TC ECU power off";
        // Turn off the ECU
        ECUPowerControl(false);
        break;
    case RATSRS41REGEN:
        msg2 = "TC RS41 regen";
        if (IsECUPowerEnabled()) {
            ecu_json.clear();
            ecu_json["rs41Regen"] = true;
            sendEcuJson(paired_ecu);
            LoRaTx(ecu_json_str);
        } else {
            msg3 = "Cannot send RS41 regen, ECU power is off";
            msg1_flag = WARN;
        }
        break;
    case RATSECURS41METADATA:
        msg2 = "TC RS41 metadata";
        if (IsECUPowerEnabled()) {
            ecu_json.clear();
            ecu_json["rs41Metadata"] = true;
            sendEcuJson(paired_ecu);
        } else {
            msg3 = "Cannot send RS41 metadata request, ECU power is off";
            msg1_flag = WARN;
        }
        break;
    case RATSRS41ENON:
        msg2 = "TC RS41 enable on";
        if (IsECUPowerEnabled()) {
            ecu_json.clear();
            ecu_json["rs41Enable"] = true;
            sendEcuJson(paired_ecu);
        } else {
            msg3 = "Cannot send RS41 enable, ECU power is off";
            msg1_flag = WARN;
        }
        break;
    case RATSRS41ENOFF:
        msg2 = "TC RS41 enable off";
        if (IsECUPowerEnabled()) {
            ecu_json.clear();
            ecu_json["rs41Enable"] = false;
            sendEcuJson(paired_ecu);
        } else {
            msg3 = "Cannot send RS41 enable off, ECU power is off";
            msg1_flag = WARN;
        }
        break;
    case RATSTSENPOWON:
        msg2 = "TC TSEN power on";
        if (IsECUPowerEnabled()) {
            ecu_json.clear();
            ecu_json["tsenPower"] = true;
            sendEcuJson(paired_ecu);
        } else {
            msg3 = "Cannot send TSEN power on, ECU power is off";
            msg1_flag = WARN;
        }
        break;
    case RATSTSENPOWOFF:
        msg2 = "TC TSEN power off";
        if (IsECUPowerEnabled()) {
            ecu_json.clear();
            ecu_json["tsenPower"] = false;
            sendEcuJson(paired_ecu);
        } else {
            msg3 = "Cannot send TSEN power off, ECU power is off";
            msg1_flag = WARN;
        }
        break;
    case RATSPAIREDCEU:
        ratsConfigs.paired_ecu.Write(ratsParam.paired_ecu);
        paired_ecu = ratsParam.paired_ecu;
        msg2 = "TC set the paired ECU ID: " + String(ratsConfigs.paired_ecu.Read());
        break;
    default:
        msg1_flag = CRIT;
        msg3 = "Unknown TC " + String(telecommand) + " received";
        break;
    }

    // Send an acknowledgement TM
    zephyrTX.clearTm();
    zephyrTX.setStateDetails(1, "RATSTCACK");
    zephyrTX.setStateFlagValue(1, msg1_flag);

    zephyrTX.setStateDetails(2, msg2);
    zephyrTX.setStateFlagValue(2, FINE);

    zephyrTX.setStateDetails(3, msg3);
    zephyrTX.setStateFlagValue(3, FINE);

    TM_ack_flag = NO_ACK;
    zephyrTX.TM();

    // Log the TC summary message
    switch (msg1_flag) {
        case FINE:
            log_nominal(msg2.c_str());
            break;
        case WARN:
            log_error(msg2.c_str());
            break;
        case CRIT:
            log_error(msg2.c_str());
            break;
        default:
            log_debug(msg2.c_str());
    }

    if (send_rats_eeprom) {
        SendRATSEEPROM();
    }

    if (send_mcm_eeprom) {
        // Request the MCB EEPROM. MCBRouter will handle the response
        mcbComm.TX_ASCII(MCB_GET_EEPROM);
    }

    return true;
}

