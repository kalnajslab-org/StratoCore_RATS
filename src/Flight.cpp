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
    FL_MANUAL_MOTION,

    FL_ERROR = MODE_ERROR,
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

    // Save the flight mode substate to the global variable 
    // so that it can be accessed by the status message
    flight_mode_substate= inst_substate;

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
            scheduler.AddAction(ACTION_LORA_COUNT_MSGS, 1);
            scheduler.AddAction(ACTION_LORA_WAIT_TIMEOUT, LORA_MSG_TIMEOUT);
            // Reset lora count
            lora_count_check(true);
            inst_substate = FL_LORA_WAIT1;
            log_nominal("Entering FL_LORA_WAIT1");
            log_nominal("FL_LORA_WAIT1 waiting for LoRa message");
        }
        break;
    case FL_LORA_WAIT1:
        if (CheckAction(ACTION_LORA_COUNT_MSGS)) {
            // Wait for enough LoRa message to arrive.
            if (lora_count_check() >= LORA_MSG_COUNT) { 
                log_nominal("FL_LORA_WAIT1 Expected LoRa messages received");
                inst_substate = FL_CONFIG_ECU;
                log_nominal("Entering FL_CONFIG_ECU");
            } else {
                scheduler.AddAction(ACTION_LORA_COUNT_MSGS, 1);
            }
        }
        if (CheckAction(ACTION_LORA_WAIT_TIMEOUT)) {
            log_error("FL_LORA_WAIT1 Expected LoRa messages not received");
            ZephyrLogWarn("LoRa messages not received during first wait");
            inst_substate = FL_ERROR;
            log_error("Entering FL_ERROR");
        }

        break;
    case FL_CONFIG_ECU:
        // Cancel LORA timeout action
        CheckAction(ACTION_LORA_WAIT_TIMEOUT);
        // Configure the ECU here.
        log_nominal("FL_CONFIG_ECU Configuring ECU");
        scheduler.AddAction(ACTION_LORA_COUNT_MSGS, 1);
        scheduler.AddAction(ACTION_LORA_WAIT_TIMEOUT, LORA_MSG_TIMEOUT);
        inst_substate = FL_LORA_WAIT2;
        // Reset lora count
        lora_count_check(true);
        log_nominal("Entering FL_LORA_WAIT2");
        log_nominal("FL_LORA_WAIT2 waiting for LoRa message");
        break;
    case FL_LORA_WAIT2:
        if (CheckAction(ACTION_LORA_COUNT_MSGS)) {
            // Wait for enough LoRa messages to arrive.
            if (lora_count_check() >= LORA_MSG_COUNT) { 
                // Configure ECU here.
                log_nominal("FL_LORA_WAIT2 Expected LoRa messages received");
                scheduler.AddAction(ACTION_START_TELEMETRY, 0); 
                inst_substate = FL_MEASURE;
                log_nominal("Entering FL_MEASURE");
            } else {
                scheduler.AddAction(ACTION_LORA_COUNT_MSGS, 1);
            }
        }
        if (CheckAction(ACTION_LORA_WAIT_TIMEOUT)) {
            log_error("FL_LORA_WAIT2 Expected LoRa messages not received");
            ZephyrLogWarn("LoRa messages not received during second wait");
            inst_substate = FL_ERROR;
            log_error("Entering FL_ERROR");
        }
        break;
    case FL_MEASURE:
        // Cancel LORA timeout action
        CheckAction(ACTION_LORA_WAIT_TIMEOUT);
        if (CheckAction(ACTION_START_TELEMETRY)) {
            inst_substate = FL_SEND_TELEMETRY;
            log_nominal("Entering FL_SEND_TELEMETRY");
            break;
        } else {
            if(CheckAction(ACTION_REEL_OUT)) {
                mcb_motion = MOTION_REEL_OUT;
                inst_substate = FL_MANUAL_MOTION;
                log_nominal("Entering FL_MANUAL_MOTION (reel out)");
                // START the Flight Manual Motion state machine
                Flight_ManualMotion(true);
            } else if (CheckAction(ACTION_REEL_IN)) {
                mcb_motion = MOTION_REEL_IN;
                inst_substate = FL_MANUAL_MOTION;
                log_nominal("Entering FL_MANUAL_MOTION (reel in)");
                // START the Flight Manual Motion state machine
                Flight_ManualMotion(true);
            }
        }
        log_debug("FL Measure");
        break;
    case FL_MANUAL_MOTION:
        if (Flight_ManualMotion(false)) {
            inst_substate = FL_MEASURE;
            log_nominal("Entering FL_MEASURE");
        }
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
        log_error((String("Unknown substate ") + String(inst_substate) + " in FL").c_str());
        break;
    }
}

