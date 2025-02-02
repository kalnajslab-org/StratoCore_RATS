#include "StratoRATS.h"

enum SAStates_t : uint8_t {
    SA_ENTRY = MODE_ENTRY,
    
    // add any desired states between entry and shutdown
    SA_LOOP,
    SA_SEND_S,
    SA_ACK_WAIT,
    SA_SHUTDOWN = MODE_SHUTDOWN,
    SA_EXIT = MODE_EXIT
};

void StratoRATS::SafetyMode()
{
    ratsReportCheck(RATS_REPORT_PERIOD_SECS);
    switch (inst_substate) {
    case SA_ENTRY:
        RATS_Shutdown();
        // Register ACTION_RATS_REPORT to trigger the first status message 
        scheduler.AddAction(ACTION_RATS_REPORT, 1);
        log_nominal(" Shut down, Entering SA");
        inst_substate = SA_SEND_S;
        log_nominal("Entering SA_SEND_S");
        break;
    case SA_SEND_S:
        log_nominal("Sending safety message");
        zephyrTX.S();
        scheduler.AddAction(RESEND_SAFETY, 60);
        inst_substate = SA_ACK_WAIT;
        log_nominal("Entering SA_ACK_WAIT");
        break;
    case SA_ACK_WAIT:
        log_debug("Waiting on safety ack");
        // check if the ack has been received
        if (S_ack_flag == ACK) {
            // clear the ack flag and go to the loop
            S_ack_flag = NO_ACK;
            inst_substate = SA_LOOP;
            log_nominal("Entering SA_LOOP");
        } else if (S_ack_flag == NAK) {
            // just clear the ack flag -- a resend is already scheduled
            S_ack_flag = NO_ACK;
        }
        // if a minute has passed, resend safety
        if (CheckAction(RESEND_SAFETY)) {
            inst_substate = SA_SEND_S;
            log_nominal("Entering SA_SEND_S");
        }
        break;
    case SA_LOOP:
        // nominal ops
        log_debug("SA loop");
        break;
    case SA_SHUTDOWN:
        RATS_Shutdown();
        log_nominal("Shutdown warning received in SA");
        break;
    case SA_EXIT:
        // perform cleanup
        log_nominal("Exiting SA");
        break;
    default:
        // todo: throw error
        log_error("Unknown substate in SA");
        inst_substate = SA_ENTRY; // reset
        break;
    }
}
