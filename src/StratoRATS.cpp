#include "StratoRATS.h"

StratoRATS::StratoRATS()
    : StratoCore(&ZEPHYR_SERIAL, INSTRUMENT)
    , mcbComm(&MCB_SERIAL)
{
}

void StratoRATS::InstrumentSetup()
{   
    if (!ratsConfigs.Initialize()) {
        ZephyrLogWarn("Error loading from EEPROM! Reconfigured");
    }

    mcbComm.AssignBinaryRXBuffer(binary_mcb, MCB_BINARY_BUFFER_SIZE);

}

void StratoRATS::InstrumentLoop()
{
    WatchFlags();
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
    if (CheckAction(SEND_STATUS)) {
        log_nominal("Send status");
        sendTMstatusMsg();
        scheduler.AddAction(SEND_STATUS, repeat_secs);
    }
}

void StratoRATS::sendTMstatusMsg() {
    
    // Create the binary status payload
    uint8_t status[8];
    for (int i = 0; i < sizeof(status); i++) {
        status[i] = i+1;
    }
    
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
    for (int i = 0; i < sizeof(status); i++)
    {
        zephyrTX.addTm(status[i]);
    }

    zephyrTX.TM();

}

void StratoRATS::NoteProfileStart()
{
    mcb_motion_ongoing = true;
    profile_start = millis();

    if (MOTION_DOCK == mcb_motion || MOTION_IN_NO_LW == mcb_motion) mcb_dock_ongoing = true;

    mcb_tm_counter = 0;

    zephyrTX.clearTm(); // empty the TM buffer for incoming MCB motion data

    // Add the start time to the MCB TM Header if not in real-time mode
    // TODO What is real-time mcb mode?
    if (!ratsConfigs.real_time_mcb.Read()) {
        zephyrTX.addTm((uint32_t) now()); // as a header, add the current seconds since epoch
    }
}

void StratoRATS::AddMCBTM()
{
    // make sure it's the correct size
    if (mcbComm.binary_rx.bin_length != MOTION_TM_SIZE) {
        log_error("invalid motion TM size");
        return;
    }

    // if not in real-time mode, add the sync and time
    if (!ratsConfigs.real_time_mcb.Read()) {
        // sync byte        
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) 0xA5;
                
        // tenths of seconds since start
        uint16_t elapsed_time = (uint16_t)((millis() - profile_start) / 100);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (elapsed_time >> 8);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (elapsed_time & 0xFF);
    }

    // add each byte of data to the message
    for (int i = 0; i < MOTION_TM_SIZE; i++) {
        MCB_TM_buffer[MCB_TM_buffer_idx++] = mcbComm.binary_rx.bin_buffer[i];
    }

    // if real-time mode, send the TM packet
    if (ratsConfigs.real_time_mcb.Read()) {
        snprintf(log_array, LOG_ARRAY_SIZE, "MCB TM Packet %u", ++mcb_tm_counter);
        zephyrTX.addTm(MCB_TM_buffer,MCB_TM_buffer_idx);
        zephyrTX.setStateDetails(1, log_array);
        zephyrTX.setStateFlagValue(1, FINE);
        zephyrTX.setStateFlagValue(2, NOMESS);
        zephyrTX.setStateFlagValue(3, NOMESS);
        zephyrTX.TM();
        log_nominal(log_array);
        MCB_TM_buffer_idx = 0; //reser the MCB buffer pointer
    }
}

void StratoRATS::SendMCBEEPROM()
{
    // the binary buffer has been prepared by the MCBRouter
    zephyrTX.clearTm();
    zephyrTX.addTm(mcbComm.binary_rx.bin_buffer, mcbComm.binary_rx.bin_length);
    Serial.print("MCB EEPROM bytes: ");
    Serial.println(mcbComm.binary_rx.bin_length);

    // use only the first flag to preface the contents
    zephyrTX.setStateDetails(1, "MCB EEPROM Contents");
    zephyrTX.setStateFlagValue(1, FINE);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    // send as TM
    TM_ack_flag = NO_ACK;
    zephyrTX.TM();

    log_nominal("Sent MCB EEPROM as TM");
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

    log_nominal(log_array);
    if (!WriteFileTM("MCB")) {
        log_error("Unable to write MCB TM to SD file");
    }
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
    zephyrTX.setStateDetails(1, "RATS EEPROM Contents");
    zephyrTX.setStateFlagValue(1, FINE);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    // send as TM
    TM_ack_flag = NO_ACK;
    log_nominal((String("Sending TM with EEPROM bytes:")+String(mcbComm.binary_rx.bin_length)).c_str());
    zephyrTX.TM();

    log_nominal("Sent RATS EEPROM as TM");
}
