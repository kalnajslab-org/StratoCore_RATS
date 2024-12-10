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
    statusMsgCheck(STATUS_MSG_PERIOD_SECS);
    switch (inst_substate) {
    case SA_ENTRY:
        RATS_Shutdown();
        scheduler.AddAction(SEND_STATUS, 1);
        log_nominal(" Shut down, Entering SA");
        inst_substate = SA_SEND_S;
        break;
    case SA_SEND_S:
        log_nominal("Sending safety message");
        zephyrTX.S();
        scheduler.AddAction(RESEND_SAFETY, 60);
        inst_substate = SA_ACK_WAIT;
        break;
    case SA_ACK_WAIT:
        log_debug("Waiting on safety ack");
        // check if the ack has been received
        if (S_ack_flag == ACK) {
            // clear the ack flag and go to the loop
            S_ack_flag = NO_ACK;
            inst_substate = SA_LOOP;
        } else if (S_ack_flag == NAK) {
            // just clear the ack flag -- a resend is already scheduled
            S_ack_flag = NO_ACK;
        }
        // if a minute has passed, resend safety
        if (CheckAction(RESEND_SAFETY)) {
            inst_substate = SA_SEND_S;
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
