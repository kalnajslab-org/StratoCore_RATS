#include "StratoRATS.h"

enum SBStates_t : uint8_t {
    SB_ENTRY = MODE_ENTRY,
    // add any desired states between entry and shutdown
    SB_LOOP,
    SB_SHUTDOWN = MODE_SHUTDOWN,
    SB_EXIT = MODE_EXIT
};

void StratoRATS::StandbyMode()
{
    ratsReportCheck(RATS_REPORT_PERIOD_SECS);
    switch (inst_substate) {
    case SB_ENTRY:
        log_nominal("Entering SB");
        RATS_Shutdown();
        // send mode request in first loop
        scheduler.AddAction(SEND_IMR, 0);
        // Register ACTION_RATS_REPORT to trigger the first status message 
        scheduler.AddAction(ACTION_RATS_REPORT, 1);
        inst_substate = SB_LOOP;
        log_nominal("Entering SB_LOOP");
        break;
    case SB_LOOP:
        // nominal ops
        log_debug("SB loop");
        // send a mode request if time, and schedule the next
        if (CheckAction(SEND_IMR)) {
            log_nominal("Sending mode request to OBC");
            zephyrTX.IMR();
            scheduler.AddAction(SEND_IMR, 5);
        }
        break;
    case SB_SHUTDOWN:
        // prep for shutdown
        log_nominal("Shutdown warning received in SB");
        RATS_Shutdown();
        break;
    case SB_EXIT:
        // perform cleanup
        log_nominal("Exiting SB");
        break;
    default:
        // todo: throw error
        log_error("Unknown substate in SB");
        inst_substate = SB_ENTRY; // reset
        log_nominal("Entering SB_ENTRY");
        break;
    }
}
