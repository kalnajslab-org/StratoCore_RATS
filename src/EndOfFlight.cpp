#include "StratoRATS.h"

enum EFStates_t : uint8_t {
    EF_ENTRY = MODE_ENTRY,
    
    // add any desired states between entry and shutdown
    EF_LOOP,
    
    EF_SHUTDOWN = MODE_SHUTDOWN,
    EF_EXIT = MODE_EXIT
};

void StratoRATS::EndOfFlightMode()
{
    my_inst_mode = MODE_EOF;
    ratsReportCheck();
    switch (inst_substate) {
    case EF_ENTRY:
        // perform setup
        // Register ACTION_RATS_REPORT to trigger the first status message 
        scheduler.AddAction(ACTION_RATS_REPORT, 1);
        inst_substate = EF_LOOP;
        log_nominal("Entering EF_LOOP");
        break;
    case EF_LOOP:
        // nominal ops
        log_debug("EF loop");
        break;
    case EF_SHUTDOWN:
        // prep for shutdown
        log_nominal("Shutdown warning received in EF");
        break;
    case EF_EXIT:
        // perform cleanup
        log_nominal("Exiting EF");
        break;
    default:
        // todo: throw error
        log_error("Unknown substate in EF");
        inst_substate = EF_ENTRY; // reset
        log_nominal("Entering EF_ENTRY");
        break;
    }
}
