#include "StratoRATS.h"

enum FLStates_t : uint8_t {
    FL_ENTRY = MODE_ENTRY,
    // add any desired states between entry and shutdown
    FL_GPS_WAIT,
    FL_LORA_WAIT1,
    FL_CONFIG_ECU,
    FL_LORA_WAIT2,
    FL_MEASURE,
    FL_SEND_TELEMETRY,
    FL_ERROR,
    FL_SHUTDOWN = MODE_SHUTDOWN,
    FL_EXIT = MODE_EXIT
};

// this function is called at the defined rate
//  * when flight mode is entered, it will start in FL_ENTRY state
//  * it is then up to this function to change state as needed by updating the inst_substate variable
//  * on each loop, whichever substate is set will be perfomed
//  * when the mode is changed by the Zephyr, FL_EXIT will automatically be set
//  * it is up to the FL_EXIT logic perform any actions for leaving flight mode
void StratoRATS::FlightMode()
{
    // Send a status TM, if it is time. 
    // statusMsgCheck() will reschedule the action.
    statusMsgCheck(STATUS_MSG_PERIOD_SECS);
    switch (inst_substate) {
    case FL_ENTRY:
        log_nominal("Entering FL");
        // Register ACTION_SEND_STATUS action to trigger the first status message 
        scheduler.AddAction(ACTION_SEND_STATUS, 1);
        // Set trigger for received LOra message
        scheduler.AddAction(ACTION_SIM_LORA_MSG, 30);
        // Transition to waiting for a GPS message.
        scheduler.AddAction(ACTION_GPS_WAIT_MSG, 5);
        inst_substate = FL_GPS_WAIT;
        log_nominal("Entering FL_GPS_WAIT");
        break;
    case FL_GPS_WAIT:
        // wait for a Zephyr GPS message to set the time before moving on
        if (CheckAction(ACTION_GPS_WAIT_MSG)) {
            log_nominal("FL_GPS_WAIT waiting for GPS Time");
            scheduler.AddAction(ACTION_GPS_WAIT_MSG, 5);
        }
        // time_valid is set when StratoCore::RouteRXMessage() receives a GPS message
        if (time_valid) {
            // Transition to waiting for LoRa
            scheduler.AddAction(ACTION_LORA_WAIT_MSG, 1);
            // Reset lora count
            lora_count_check(true);
            inst_substate = FL_LORA_WAIT1;
            log_nominal("Entering FL_LORA_WAIT1");
        }
        break;
    case FL_LORA_WAIT1:
        if (CheckAction(ACTION_LORA_WAIT_MSG)) {
            log_nominal("FL_LORA_WAIT waiting for LoRa message");
            scheduler.AddAction(ACTION_LORA_WAIT_MSG, 1);
            // Wait for 6 LoRa message to arrive.
            if (lora_count_check() >= 6) { 
                log_nominal("FL_LORA_WAIT LoRa 6 messages received");
                inst_substate = FL_CONFIG_ECU;
                log_nominal("Entering FL_CONFIG_ECU");
            }
        }
        break;
    case FL_CONFIG_ECU:
        // Configure the ECU here.
        inst_substate = FL_LORA_WAIT2;
        // Reset lora count
        lora_count_check(true);
        log_nominal("Entering FL_LORA_WAIT2");
        break;
    case FL_LORA_WAIT2:
        if (CheckAction(ACTION_LORA_WAIT_MSG)) {
            log_nominal("FL_LORA_WAIT waiting for LoRa message");
            scheduler.AddAction(ACTION_LORA_WAIT_MSG, 1);
            // Wait for 6 LoRa messages to arrive.
            if (lora_count_check() >= 6) { 
                // Configure ECU here.
                log_nominal("FL_LORA_WAIT LoRa 6 messages received");
                scheduler.AddAction(ACTION_START_TELEMETRY, 0); 
                inst_substate = FL_MEASURE;
                log_nominal("Entering FL_MEASURE");
            }
        }
        break;
    case FL_MEASURE:
        if (CheckAction(ACTION_START_TELEMETRY)) {
            inst_substate = FL_SEND_TELEMETRY;
            log_nominal("Entering FL_SEND_TELEMETRY");
            break;
        }
        log_debug("FL Measure");
        break;
    case FL_SEND_TELEMETRY:
        inst_substate = FL_MEASURE;
        scheduler.AddAction(ACTION_START_TELEMETRY, 60); 
        log_nominal("Entering FL_MEASURE");
        break;
    case FL_ERROR:
        // generic error state for flight mode to go to if any error is detected
        // this state can make sure the ground is informed, and wait for ground intervention
        RATS_Shutdown();
        log_debug("In Error Sub State");
        break;
    case FL_SHUTDOWN:
        RATS_Shutdown();
        log_nominal("Shutdown warning received in FL");
        break;
    case FL_EXIT:
        RATS_Shutdown();
        log_nominal("Exiting FL");
        break;
    default:
        // todo: throw error
        log_error("Unknown substate in FL");
        inst_substate = FL_ENTRY; // reset
        break;
    }
}
