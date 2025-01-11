#include "StratoRATS.h"
#include <SPI.h>

StratoRATS::StratoRATS()
    : StratoCore(&ZEPHYR_SERIAL, INSTRUMENT)
    , mcbComm(&MCB_SERIAL)
{
}

void StratoRATS::InstrumentSetup()
{   
    // safe pin required by Zephyr
    pinMode(SAFE_PIN, OUTPUT);
    digitalWrite(SAFE_PIN, LOW);
    
    if (!ECULoRaInit(SS_PIN, RESET_PIN, INTERRUPT_PIN, &SPI1, LORA_SCK, LORA_MISO, LORA_MOSI)) {
        log_error("WARN: LoRa Initialization Failed");
        ZephyrLogWarn("WARN: LoRa Initialization Failed");
    } else {
        log_nominal("LoRa Initialized");
    }; 

    if (!ratsConfigs.Initialize()) {
        ZephyrLogWarn("Error loading from EEPROM! Reconfigured");
    }

    mcbComm.AssignBinaryRXBuffer(binary_mcb, MCB_BINARY_BUFFER_SIZE);

}

void StratoRATS::InstrumentLoop()
{
    WatchFlags();
    LoRaRX();
}


void StratoRATS::LoRaRX()
{
    if (ecu_lora_get_msg(&lora_msg)) {
        total_lora_count++;
        if (lora_msg.count != total_lora_count) {
            log_error(String(String("LoRa message count mismatch ") + String(lora_msg.count) + " " + String(total_lora_count)).c_str());
            total_lora_count = lora_msg.count;
        }
#if EXTRA_LOGGING
        if (lora_msg.count % 30 == 0) {
            snprintf(log_array, LOG_ARRAY_SIZE,
                "LoRa rx n:%ld id:%ld rssi:%d snr:%.1f ferr:%ld",
                lora_msg.count, lora_msg.id, ecu_lora_rssi(), ecu_lora_snr(), ecu_lora_frequency_error());

            log_nominal(log_array);
        }
#endif
    }
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

void StratoRATS::SetAction(uint8_t action)
{
    action_flags[action].flag_value = true;
    action_flags[action].stale_count = 0;
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

void StratoRATS::statusMsgCheck(int repeat_secs) {
    if (CheckAction(ACTION_SEND_STATUS)) {
        log_nominal("Send status");
        sendTMstatusMsg();
        scheduler.AddAction(ACTION_SEND_STATUS, repeat_secs);
    }
}

void StratoRATS::sendTMstatusMsg() {
    
    // Create the binary status payload
    uint8_t status[8];
    for (uint i = 0; i < sizeof(status); i++) {
        status[i] = i+1;
    }
    status[0] = flight_mode_substate; 

    String Message = "";

    // First
    Message = "RATS status";
    zephyrTX.setStateFlagValue(1, FINE);
    zephyrTX.setStateDetails(1, Message);

    // Second
    Message = "Other";
    zephyrTX.setStateFlagValue(2, FINE);
    zephyrTX.setStateDetails(2, Message);

    // Third: GPS Position
    Message = "";   
    zephyrTX.setStateFlagValue(3, FINE);
    Message.concat(zephyrRX.zephyr_gps.latitude);
    Message.concat(',');
    Message.concat(zephyrRX.zephyr_gps.longitude);
    Message.concat(',');
    Message.concat(zephyrRX.zephyr_gps.altitude);
    zephyrTX.setStateDetails(3, Message);
    Message = "";
    
    // Add the initial timestamp
    zephyrTX.addTm((uint32_t)now());

    // And the status size (bytes)
    zephyrTX.addTm(uint16_t(sizeof(status)));
    
    // Add the samples
    for (uint i = 0; i < sizeof(status); i++)
    {
        zephyrTX.addTm(status[i]);
    }

    zephyrTX.TM();
    MCB_TM_buffer_idx = 0; //reset the MCB buffer pointer

}

bool StratoRATS::StartMCBMotion()
{
    bool success = false;

    String msg;

    switch (mcb_motion) {
    case MOTION_REEL_IN:
        success = mcbComm.TX_Reel_In(retract_length, ratsConfigs.retract_velocity.Read());
        max_profile_seconds = 60 * (retract_length / ratsConfigs.retract_velocity.Read()) + ratsConfigs.motion_timeout.Read();
        msg = String("Reel in ") + String(retract_length,1) 
            + " revs, timeout " + String(max_profile_seconds) 
            + "s, velocity " + String(ratsConfigs.retract_velocity.Read(),1);
        break;
    case MOTION_REEL_OUT:
        success = mcbComm.TX_Reel_Out(deploy_length, ratsConfigs.deploy_velocity.Read());
        max_profile_seconds = 60 * (deploy_length / ratsConfigs.deploy_velocity.Read()) + ratsConfigs.motion_timeout.Read();
        msg = String("Reel out ") + String(deploy_length,1) 
            + " revs, timeout " + String(max_profile_seconds) 
            + "s, velocity " + String(ratsConfigs.deploy_velocity.Read(),1);
        break;
    case MOTION_IN_NO_LW:
        success = mcbComm.TX_In_No_LW(retract_length, ratsConfigs.retract_velocity.Read());
        max_profile_seconds = 60 * (retract_length / ratsConfigs.retract_velocity.Read()) + ratsConfigs.motion_timeout.Read();
        msg = String("Reel in (no LW) ") + String(retract_length,1) 
            + " revs, timeout " + String(max_profile_seconds) 
            + "s, velocity " + String(ratsConfigs.retract_velocity.Read(),1);
        break;
    default:
        mcb_motion = NO_MOTION;
        log_error("Unknown motion type to start");
        return false;
    }

    ZephyrLogFine(msg.c_str());
    log_nominal(msg.c_str());

    return success;
}

void StratoRATS::AddMCBTM()
{
    // make sure it's the correct size
    if (mcbComm.binary_rx.bin_length != MOTION_TM_SIZE) {
        log_error("invalid motion TM size");
        return;
    }

    // add each byte of data to the message
    for (int i = 0; i < MOTION_TM_SIZE; i++) {
        MCB_TM_buffer[MCB_TM_buffer_idx++] = mcbComm.binary_rx.bin_buffer[i];
    }

    // Send the TM packet
    zephyrTX.addTm(MCB_TM_buffer,MCB_TM_buffer_idx);
    zephyrTX.setStateDetails(1, log_array);
    zephyrTX.setStateFlagValue(1, FINE);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);
    zephyrTX.TM();
    MCB_TM_buffer_idx = 0; //reset the MCB buffer pointer

    snprintf(log_array, LOG_ARRAY_SIZE, "MCB motion TM");
    log_nominal(log_array);
}

void StratoRATS::InitMotion()
{
    mcb_motion_ongoing = true;
    profile_start = millis();

    zephyrTX.clearTm(); // empty the TM buffer for incoming MCB motion data
    zephyrTX.addTm((uint32_t) now()); // as a header, add the current seconds since epoch
}

void StratoRATS::SendMCBTM(StateFlag_t state_flag, const char * message)
{
    // use only the first flag to report the motion
    zephyrTX.addTm(MCB_TM_buffer,MCB_TM_buffer_idx);
    zephyrTX.setStateDetails(1, message);
    zephyrTX.setStateFlagValue(1, state_flag);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    TM_ack_flag = NO_ACK;
    zephyrTX.TM();
    MCB_TM_buffer_idx = 0; //reset the MCB buffer pointer
    if (!WriteFileTM("MCB")) {
        log_error("Unable to write MCB TM to SD file");
    }
}

void StratoRATS::SendMCBEEPROM()
{
    // the binary buffer has been prepared by the MCBRouter
    zephyrTX.clearTm();
    zephyrTX.addTm(mcbComm.binary_rx.bin_buffer, mcbComm.binary_rx.bin_length);

    // use only the first flag to preface the contents
    zephyrTX.setStateDetails(1, "MCB EEPROM Contents");
    zephyrTX.setStateFlagValue(1, FINE);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    // send as TM
    TM_ack_flag = NO_ACK;
    zephyrTX.TM();
    MCB_TM_buffer_idx = 0; //reset the MCB buffer pointer

    log_nominal("MCB EEPROM TM");
}

void StratoRATS::SendRATSEEPROM()
{
    // create a buffer from the EEPROM (cheat, and use the preallocated MCBComm Binary RX buffer)
    mcbComm.binary_rx.bin_length = ratsConfigs.Bufferize(mcbComm.binary_rx.bin_buffer, MAX_MCB_BINARY);

    if (0 == mcbComm.binary_rx.bin_length) {
        log_error("Unable to bufferize RATS EEPROM");
        return;
    }

    // prepare the TM buffer
    zephyrTX.clearTm();
    zephyrTX.addTm(mcbComm.binary_rx.bin_buffer, mcbComm.binary_rx.bin_length);

    // use only the first flag to preface the contents
    zephyrTX.setStateDetails(1, String("RATS EEPROM data; length ") + String(mcbComm.binary_rx.bin_length));
    zephyrTX.setStateFlagValue(1, FINE);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    // send as TM
    TM_ack_flag = NO_ACK;
    zephyrTX.TM();
    MCB_TM_buffer_idx = 0; //reset the MCB buffer pointer

    log_nominal("Sent RATS EEPROM as TM");
}

    uint32_t StratoRATS::lora_count_check(bool reset) {
        if (reset) {
            lora_count = total_lora_count;;
        } 
        return total_lora_count - lora_count;
    }
