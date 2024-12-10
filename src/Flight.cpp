#include "StratoRATS.h"

enum FLStates_t : uint8_t {
    FL_ENTRY = MODE_ENTRY,
    // add any desired states between entry and shutdown
    FL_GPS_WAIT,
    FL_IDLE,
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
    statusMsgCheck(STATUS_MSG_PERIOD_SECS);
    switch (inst_substate) {
    case FL_ENTRY:
        // perform setup
        log_nominal("Entering FL");
        scheduler.AddAction(GPS_WAIT_MSG, 5);
        scheduler.AddAction(SEND_STATUS, 1);
        inst_substate = FL_GPS_WAIT;
        break;
    case FL_GPS_WAIT:
        // wait for a Zephyr GPS message to set the time before moving on
        if (CheckAction(GPS_WAIT_MSG)) {
            log_nominal("FL_GPS_WAIT waiting for GPS Time");
            scheduler.AddAction(GPS_WAIT_MSG, 5);
        }
        if (time_valid)
        {
            inst_substate = FL_IDLE; // automatically go to idle
            log_nominal("Entering FL_IDLE");
        }
        break;
    case FL_IDLE:
        // some logic here to determine when to leave idle
        inst_substate = FL_MEASURE;                
        scheduler.AddAction(START_TELEMETRY, 10); 
        log_nominal("Entering FL_MEASURE");
        break;
    case FL_MEASURE:
        if (CheckAction(START_TELEMETRY)) {
            inst_substate = FL_SEND_TELEMETRY;
            log_nominal("Entering FL_SEND_TELEMETRY");
            break;
        }
        log_debug("FL Measure");
        break;
    case FL_SEND_TELEMETRY:
        inst_substate = FL_MEASURE;
        scheduler.AddAction(START_TELEMETRY, 10); 
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
