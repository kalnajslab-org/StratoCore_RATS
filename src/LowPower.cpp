#include "StratoRATS.h"

// LPStates_t (LP_ENTRY, LP_LOOP, ...) is defined in StratoRATS.h

void StratoRATS::LowPowerMode()
{
    my_inst_mode = MODE_LOWPOWER;
    switch (inst_substate) {
    case LP_ENTRY:
        // perform setup
        log_nominal("Entering LP");
        RATS_Shutdown();
        inst_substate = LP_LOOP;
        log_nominal("Entering LP_LOOP");
        break;
    case LP_LOOP:
        // nominal ops
        log_debug("LP loop");
        break;
    case LP_SHUTDOWN:
        // prep for shutdown
        RATS_Shutdown();
        static elapsedMillis shutdown_warning_timer;
        if (shutdown_warning_timer >= WARNING_INTERVAL_MS) {
            log_nominal("Shutdown warning received in LP");
            shutdown_warning_timer = 0;
        }
        break;
    case LP_EXIT:
        // perform cleanup
        log_nominal("Exiting LP");
        break;
    default:
        // todo: throw error
        log_error("Unknown substate in LP");
        inst_substate = LP_ENTRY; // reset
        log_nominal("Entering LP_ENTRY");
        break;
    }
}
