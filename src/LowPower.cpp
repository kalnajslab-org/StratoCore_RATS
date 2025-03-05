#include "StratoRATS.h"

enum LPStates_t : uint8_t {
    LP_ENTRY = MODE_ENTRY,
    
    // add any desired states between entry and shutdown
    LP_LOOP,
    
    LP_SHUTDOWN = MODE_SHUTDOWN,
    LP_EXIT = MODE_EXIT
};

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
        log_nominal("Shutdown warning received in LP");
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
