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

    // ECU power enable
    pinMode(ECU_PWR_EN, OUTPUT);
    digitalWrite(ECU_PWR_EN, LOW);

    // LoRa initialization
    if (!ECULoRaInit(
        LORA_FOLLOWER, 
        1000, 
        RATS_LORA_CS, 
        RATS_LORA_RST, 
        RATS_LORA_INT, 
        &SPI1, 
        RATS_LORA_SCK, 
        RATS_LORA_MISO, 
        RATS_LORA_MOSI,
        FREQUENCY,
        BANDWIDTH,
        SF,
        TX_POWER
    )) {
        log_error("WARN: LoRa Initialization Failed");
        ZephyrLogWarn("WARN: LoRa Initialization Failed");
    } else {
        log_nominal("LoRa Initialized");
    }; 

    if (!ratsConfigs.Initialize()) {
        ZephyrLogWarn("Error loading from EEPROM! Reconfigured");
    }

    mcbComm.AssignBinaryRXBuffer(binary_mcb, MCB_BINARY_BUFFER_SIZE);

    // Initialize the RATSReport.
    last_rats_report = now();

    rats_report_header.header_size_bytes = RATS_HEADER_SIZE_BYTES;
    rats_report_header.num_ecu_records = 0;
    rats_report_header.ecu_pwr_on = 0;
    rats_report_header.v56 = 0;
    rats_report_header.ecu_record_size_bytes = ECU_REPORT_SIZE_BYTES;
}

void StratoRATS::InstrumentLoop()
{
    static uint32_t serial_keepalive_millis = 0;
    
    WatchFlags();
    LoRaRX();

    // Keep the serial port alive. The MAX3381 has a 30 second timeout,
    // so we send a character every 29 seconds to keep it alive.
    int32_t millis_delta = (int32_t) (millis() - serial_keepalive_millis);
    if (millis_delta > 29000 || millis_delta < 0) {
        ZEPHYR_SERIAL.write('\n');
        serial_keepalive_millis = millis();
    }
}

void StratoRATS::LoRaRX()
{
    if (ecu_lora_rx(&lora_msg)) {
        total_lora_count++;
        if (lora_msg.count != total_lora_count) {
            log_error(String(String("LoRa message count mismatch ") + String(lora_msg.count) + " " + String(total_lora_count)).c_str());
            total_lora_count = lora_msg.count;
        }
//#if EXTRA_LOGGING
    ECUReportBytes_t payload;
    for (uint8_t i = 0; i < lora_msg.data_len; i++) {
        payload[i] = lora_msg.data[i];
    }

    // Add the LoRa message to the RATS report.
    ratsReportAccumulate(payload);

    if (lora_msg.count % 30 == 0) {
            snprintf(log_array, LOG_ARRAY_SIZE,
                "LoRa rx n:%ld id:%ld rssi:%d snr:%.1f ferr:%ld",
                lora_msg.count, lora_msg.id, ecu_lora_rssi(), ecu_lora_snr(), ecu_lora_frequency_error());
            ECUReport_t ecu_report = ecu_report_deserialize(payload);
            ecu_report_print(ecu_report);
            log_nominal(log_array);
        }
//#endif
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
    // Turn off the ECU
    ECUControl(false);
}

void StratoRATS::ratsReportCheck(bool timed_check)
{
    if (!timed_check)
    {
        if (CheckAction(ACTION_RATS_REPORT))
        {
            ratsReportTM();
            last_rats_report = now();
            scheduler.AddAction(ACTION_RATS_REPORT, RATS_REPORT_PERIOD_SECS);
        }
    }
    else
    {
        if (rats_report_header.num_ecu_records >= NUM_ECU_REPORTS || ((now() - last_rats_report) > RATS_REPORT_PERIOD_SECS))
        {
            ratsReportTM();
            last_rats_report = now();
        }
    }
}

void StratoRATS::ECUControl(bool enable)
{
    if (enable) {
        digitalWrite(ECU_PWR_EN, HIGH);
        log_nominal("ECU Power Enabled");
    } else {
        digitalWrite(ECU_PWR_EN, LOW);
        log_nominal("ECU Power Disabled");
    }
}

void::StratoRATS::ratsReportAccumulate(ECUReportBytes_t& ecu_report_bytes) {
    if (rats_report_header.num_ecu_records < NUM_ECU_REPORTS) {
        memcpy(&rats_report_tm.records[rats_report_header.num_ecu_records], &ecu_report_bytes, ECU_REPORT_SIZE_BYTES);
        rats_report_header.num_ecu_records++;
    } else {
        log_error("RATS report buffer full");
    }
}

void StratoRATS::ratsReportTM() {

    zephyrTX.clearTm();

    // Fill in the RATS report header
    rats_report_header.ecu_pwr_on = digitalRead(ECU_PWR_EN);
    rats_report_header.v56 = 1000*analogRead(V56_MON) * (3.3 / 1024.0) * (R8 + R9) / R8;
    rats_report_header.ecu_record_size_bytes = ECU_REPORT_SIZE_BYTES;

    String Message = "";

    // First
    Message = "RATSReport";
    zephyrTX.setStateFlagValue(1, FINE);
    zephyrTX.setStateDetails(1, Message);

    // Second
    zephyrTX.setStateFlagValue(2, FINE);
    zephyrTX.setStateDetails(2, String(rats_report_header.num_ecu_records) + " records");

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

    // Add binary sections to the TM

    // Serialize rats_report_header into rats_report_tm.header_bytes
    etl::span<uint8_t> header_span(rats_report_tm.header_bytes.data(), rats_report_tm.header_bytes.size());
    etl::bit_stream_writer writer(header_span, etl::endian::big);
    writer.write_unchecked(rats_report_header.header_size_bytes, 8);       // Set in InstrumentSetup()
    writer.write_unchecked(rats_report_header.num_ecu_records, 16);        // Set in ratsReportAccumulate()
    writer.write_unchecked(rats_report_header.ecu_record_size_bytes, 16);
    writer.write_unchecked(rats_report_header.ecu_pwr_on, 1);
    writer.write_unchecked(rats_report_header.v56, 13);

    // Add the RATSReportHeader to the TM
    for (uint i = 0; i < rats_report_tm.header_bytes.size(); i++) {
        zephyrTX.addTm(rats_report_tm.header_bytes.at(i));
    }

    // Add the ECURecords to the TM
    for (uint i = 0; i < rats_report_header.num_ecu_records; i++) {
        for (uint j = 0; j < rats_report_header.ecu_record_size_bytes; j++) {
            zephyrTX.addTm(rats_report_tm.records[i].at(j));
        }
    }

    // Send the TM!
    zephyrTX.TM();

    rats_report_header.num_ecu_records = 0;

    MCB_TM_buffer_idx = 0; //reset the MCB buffer pointer

}

bool StratoRATS::StartMCBMotion()
{
    bool success = false;

    String msg;

    switch (mcb_motion) {
    case MOTION_REEL_IN:
        success = mcbComm.TX_Reel_In(retract_length, ratsConfigs.retract_velocity.Read());
        max_reel_seconds = 60 * (retract_length / ratsConfigs.retract_velocity.Read()) + ratsConfigs.motion_timeout.Read();
        msg = String("Reel in ") + String(retract_length,1) 
            + " revs, timeout " + String(max_reel_seconds) 
            + "s, velocity " + String(ratsConfigs.retract_velocity.Read(),1);
        break;
    case MOTION_REEL_OUT:
        success = mcbComm.TX_Reel_Out(deploy_length, ratsConfigs.deploy_velocity.Read());
        max_reel_seconds = 60 * (deploy_length / ratsConfigs.deploy_velocity.Read()) + ratsConfigs.motion_timeout.Read();
        msg = String("Reel out ") + String(deploy_length,1) 
            + " revs, timeout " + String(max_reel_seconds) 
            + "s, velocity " + String(ratsConfigs.deploy_velocity.Read(),1);
        break;
    case MOTION_IN_NO_LW:
        success = mcbComm.TX_In_No_LW(retract_length, ratsConfigs.retract_velocity.Read());
        max_reel_seconds = 60 * (retract_length / ratsConfigs.retract_velocity.Read()) + ratsConfigs.motion_timeout.Read();
        msg = String("Reel in (no LW) ") + String(retract_length,1) 
            + " revs, timeout " + String(max_reel_seconds) 
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
void StratoRATS::InitMCBMotionTracking()
{
    mcb_motion_ongoing = true;
    reel_motion_start = millis();

    mcb_tm_counter = 0;
    //zephyrTX.clearTm(); // empty the TM buffer for incoming MCB motion data
    MCB_TM_buffer_idx = 0;
    // Add the start time to the MCB TM Header if not in real-time mode
    if (!ratsConfigs.real_time_mcb.Read()) {
        //zephyrTX.addTm((uint32_t) now()); // as a header, add the current seconds since epoch
        uint32_t ProfileStartEpoch  = now();
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (ProfileStartEpoch >> 24);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (ProfileStartEpoch >> 16);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (ProfileStartEpoch >> 8);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (ProfileStartEpoch & 0xFF);
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
        uint16_t elapsed_time = (uint16_t)((millis() - reel_motion_start) / 100);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (elapsed_time >> 8);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (elapsed_time & 0xFF);
    }

    // add each byte of data to the message
    for (int i = 0; i < MOTION_TM_SIZE; i++) {
        MCB_TM_buffer[MCB_TM_buffer_idx++] = mcbComm.binary_rx.bin_buffer[i];
    }

    // if real-time mode, send the TM packet
    if (ratsConfigs.real_time_mcb.Read()) {
        snprintf(log_array, LOG_ARRAY_SIZE, "MCB TM (Packet %u)", ++mcb_tm_counter);
        zephyrTX.addTm(MCB_TM_buffer, MCB_TM_buffer_idx);
        zephyrTX.setStateDetails(1, log_array);
        zephyrTX.setStateFlagValue(1, FINE);
        zephyrTX.setStateDetails(2, (String("Reel: ") + String(reel_pos, 2)).c_str());
        zephyrTX.setStateFlagValue(2, FINE);
        zephyrTX.setStateDetails(3, "");
        zephyrTX.setStateFlagValue(3, NOMESS);
        zephyrTX.TM();
        log_nominal(log_array);
        //reset the MCB buffer pointer
        MCB_TM_buffer_idx = 0; 
    }

}

void StratoRATS::SendMCBTM(StateFlag_t state_flag, const char * message)
{

    // use only the first flag to report the motion
    zephyrTX.clearTm();
    zephyrTX.addTm(MCB_TM_buffer,MCB_TM_buffer_idx);
    zephyrTX.setStateDetails(1, message);
    zephyrTX.setStateFlagValue(1, state_flag);
    if (state_flag == FINE) {
        zephyrTX.setStateDetails(2, (String("Reel: ") + String(reel_pos, 2)).c_str());
        zephyrTX.setStateFlagValue(2, FINE);
    } else {
        zephyrTX.setStateDetails(2, "");
        zephyrTX.setStateFlagValue(2, NOMESS);
    }
    zephyrTX.setStateDetails(3, "");
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
