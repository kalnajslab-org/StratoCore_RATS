#include "StratoRATS.h"
#include <SPI.h>
#include <LoRa.h>

int PacketSize = 0;
//ISR for LoRa reception, needs to be outside the class for some reason
void onReceive(int Size)
{
    PacketSize = Size;
}

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

    // Set up the second SPI Port for the LoRa Module
    SPI1.setSCK(LORA_SCK);
    SPI1.setMISO(LORA_MISO);
    SPI1.setMOSI(LORA_MOSI);

    LoRa.setSPI(SPI1);
    LoRa.setPins(SS_PIN, RESET_PIN,INTERUPT_PIN);
    
    LoRaInit();  //initialize the LoRa modem

    LoRa.onReceive(onReceive);
    LoRa.receive();

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


void StratoRATS::LoRaInit()
{
   if (!LoRa.begin(FREQUENCY)){
       ZephyrLogWarn("Starting LoRa failed!");
       Serial.println("WARN: LoRa Initializtion Failed");
    }
    delay(1);
    LoRa.setSpreadingFactor(SF);
    delay(1);
    LoRa.setSignalBandwidth(BANDWIDTH);
    delay(1);
    LoRa.setTxPower(RF_POWER);
}

void StratoRATS::LoRaRX()
{
    int i = 0;

    if (PacketSize > 0) //if LoRa data is available
    {
        PacketSize = 0;
        Serial.print("Received Packet with RSSI :");
        Serial.print(LoRa.packetRssi());
        int BytesToRead = LoRa.available();
        Serial.printf(" Bytes Read: %d\n",BytesToRead);
        for (i = 0; i <  BytesToRead; i++)
           LoRa_RX_buffer[i] = LoRa.read();
        
        //for (i = 0; i< BytesToRead; i++ ) //for debug write buffer to consols
        //    Serial.write(LoRa_RX_buffer[i]);
        //Serial.println();

        if (strncmp(LoRa_RX_buffer,"ST",2) == 0)//it is a status packet
        { 
            LoRa_RX_buffer[BytesToRead] = '\0'; //null terminate buffer to make a string
            snprintf(log_array, LOG_ARRAY_SIZE, "ECU %s,%0.1f", LoRa_RX_buffer, reel_pos);
            ZephyrLogFine(LoRa_RX_buffer);

        }

        else if (strncmp(LoRa_RX_buffer,"TM",2) == 0) //it is a profile TM packet
        {
                Serial.print("TM Packet idx: ");
                Serial.println(LoRa_TM_buffer_idx);
                LoRa_rx_time = millis();  //record the time we received last LoRa TM
                if (LoRa_TM_buffer_idx + BytesToRead > 6005) //if the incomming packet will over fill a TM send what we have
                {
                    //send the LoRa PU data to zephyr as a TM
                    //snprintf(log_array, LOG_ARRAY_SIZE, "PU LoRa TM: %u.%u, %0.1f, %0.4f, %0.4f, %0.1f",pibConfigs.profile_id.Read(),++pu_tm_counter,reel_pos,profile_start_latitude, profile_start_longitude, profile_start_altitude);
                    snprintf(log_array, LOG_ARRAY_SIZE, "LoRa TM");
                    //zephyrTX.addTm(LoRa_TM_buffer,LoRa_TM_buffer_idx);
                    zephyrTX.setStateDetails(1, log_array);
                    zephyrTX.setStateFlagValue(1, FINE);
                    zephyrTX.setStateFlagValue(2, NOMESS);
                    zephyrTX.setStateFlagValue(3, NOMESS);
                    zephyrTX.TM();
                    log_nominal(log_array);
                    LoRa_TM_buffer_idx = 0; //reset the buffer
                    zephyrTX.clearTm();
                }
                
                for(i = 0; i < BytesToRead-2; i++)
                    LoRa_TM_buffer[LoRa_TM_buffer_idx++] = LoRa_RX_buffer[i+2];
                BytesToRead = 0;
            
        }

        else
        {
            snprintf(log_array, LOG_ARRAY_SIZE, "Received Unknown LoRa Packet");
            log_nominal(log_array);
        }

        
    }

    //if it has been a while since we received a LoRa TM, assume it is done and send remaining data
    if (LoRa_TM_buffer_idx > 0){
        if ((millis() - LoRa_rx_time) > LORA_TM_TIMEOUT*1000) 
        {
                    zephyrTX.addTm(LoRa_TM_buffer,LoRa_TM_buffer_idx);            
                    //send the LoRa_TM_Buffer to zephyr as a TM
                    //snprintf(log_array, LOG_ARRAY_SIZE, "ECU LoRa TM: %u.%u, %0.1f, %0.4f, %0.4f, %0.1f",ratsConfigs.profile_id.Read(),++pu_tm_counter,reel_pos,profile_start_latitude, profile_start_longitude, profile_start_altitude);
                    snprintf(log_array, LOG_ARRAY_SIZE, "LoRa TM");
                    zephyrTX.setStateDetails(1, log_array);
                    zephyrTX.setStateFlagValue(1, FINE);
                    zephyrTX.setStateFlagValue(2, NOMESS);
                    zephyrTX.setStateFlagValue(3, NOMESS);
                    zephyrTX.TM();
                    log_nominal(log_array);
                    LoRa_TM_buffer_idx = 0; //reset the buffer
                    pu_tm_counter = 0; //reset the TM counter
        }
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
    for (int i = 0; i < sizeof(status); i++) {
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
    for (int i = 0; i < sizeof(status); i++)
    {
        zephyrTX.addTm(status[i]);
    }

    zephyrTX.TM();

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

void StratoRATS::NoteProfileStart()
{
    mcb_motion_ongoing = true;
    profile_start = millis();

    if (MOTION_IN_NO_LW == mcb_motion) mcb_dock_ongoing = true;

    mcb_tm_counter = 0;

    zephyrTX.clearTm(); // empty the TM buffer for incoming MCB motion data

    // Add the start time to the MCB TM Header if not in real-time mode
    // TODO What is real-time mcb mode?
    if (!ratsConfigs.real_time_mcb.Read()) {
        zephyrTX.addTm((uint32_t) now()); // as a header, add the current seconds since epoch
    }
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

    log_nominal("Sent MCB EEPROM as TM");
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

    log_nominal("Sent RATS EEPROM as TM");
}

    uint32_t StratoRATS::lora_count_check(bool reset) {
        if (reset) {
            lora_count = 0;
            scheduler.AddAction(ACTION_SIM_LORA_MSG, 1);
        }
        if (CheckAction(ACTION_SIM_LORA_MSG)) {
            lora_count++;
            scheduler.AddAction(ACTION_SIM_LORA_MSG, 1);
        }
        return lora_count;
    }
