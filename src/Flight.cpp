#include "StratoRATS.h"

enum FLStates_t : uint8_t {
    FL_ENTRY = MODE_ENTRY,
    FL_GPS_WAIT,
    FL_WARMUP,
    FL_MEASURE,
    FL_SEND_TELEMETRY,
    FL_REEL,
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
    static uint old_inst_substate = 256;
    if (inst_substate != old_inst_substate) {
        log_nominal((String("inst_substate:" + String(inst_substate)).c_str()));
        old_inst_substate = inst_substate;
    }
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
            log_nominal("Entering FL_WARMUP");
            // Transition to waiting for LoRa
            scheduler.AddAction(ACTION_LORA_COUNT_MSGS, 1);
            scheduler.AddAction(ACTION_LORA_WAIT_TIMEOUT, LORA_MSG_TIMEOUT);
            // Initialize Flight_Warmup()
            Flight_Warmup(true);
            inst_substate = FL_WARMUP;
        }
        break;
    case FL_WARMUP:
        // Flight_Warmup() will set inst_substate to FL_ERROR if it fails
        if (Flight_Warmup(false)) {
            scheduler.AddAction(ACTION_START_TELEMETRY, 60);
            inst_substate = FL_MEASURE;
            log_nominal("Entering FL_MEASURE");
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
                inst_substate = FL_REEL;
                log_nominal("Entering FL_REEL (reel out)");
                // START the Flight Manual Motion state machine
                Flight_Reel(true);
            } else if (CheckAction(ACTION_REEL_IN)) {
                mcb_motion = MOTION_REEL_IN;
                inst_substate = FL_REEL;
                log_nominal("Entering FL_REEL (reel in)");
                // START the Flight Manual Motion state machine
                Flight_Reel(true);
            }
        }
        break;
    case FL_REEL:
        if (Flight_Reel(false)) {
            log_nominal("Entering FL_WARMUP");
            // Start the warmup sequence
            Flight_Warmup(true);
            inst_substate = FL_WARMUP;
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

