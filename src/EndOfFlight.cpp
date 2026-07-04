#include "StratoRATS.h"

// EFStates_t (EF_ENTRY, EF_LOOP, ...) is defined in StratoRATS.h

void StratoRATS::EndOfFlightMode()
{
    my_inst_mode = MODE_EOF;
    switch (inst_substate) {
    case EF_ENTRY:
        // send immediate RATSREPORT on entry to EOF
        ratsReportCheck(true); 
        inst_substate = EF_LOOP;
        log_nominal("Entering EF_LOOP");
        break;
    case EF_LOOP:
        // nominal ops
        log_debug("EF loop");
        break;
    case EF_SHUTDOWN:
        // prep for shutdown
        static elapsedMillis shutdown_warning_timer;
        if (shutdown_warning_timer >= WARNING_INTERVAL_MS) {
            log_nominal("Shutdown warning received in EF");
            shutdown_warning_timer = 0;
        }
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
