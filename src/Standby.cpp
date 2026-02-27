#include "StratoRATS.h"

enum SBStates_t : uint8_t {
    SB_ENTRY = MODE_ENTRY,
    // add any desired states between entry and shutdown
    SB_LOOP,
    SB_SHUTDOWN = MODE_SHUTDOWN,
    SB_EXIT = MODE_EXIT
};

static bool first_standby_mode_call = true;

void StratoRATS::StandbyMode()
{
    my_inst_mode = MODE_STANDBY;

    switch (inst_substate) {
    case SB_ENTRY:
        log_nominal("Entering SB");

        // Shutdown uneeded components.
        RATS_Shutdown();

        // send mode request in first loop
        scheduler.AddAction(SEND_IMR, 0);

        // Will cause the the MCB to send a message containing the reel position.
        if (first_standby_mode_call) {
            // This is the first time we're entering standby mode since boot up.
            // Schedule the MCB init motion action. It needs to be delayed because the MCB
            // takes some time to initialize after boot.
            scheduler.AddAction(ACTION_MCB_INIT_MOTION, 8);
            // Schedule the first RATSREPORT a bit after the MCB init motion, delayed
            // because it takes a few seconds for the MCB to respond to the init motion
            // and for RATS to process the MCB message.
            scheduler.AddAction(ACTION_RATS_REPORT, 12);
            first_standby_mode_call = false;
        } else {
            // Not the first time we're entering standby mode, 
            // so we can schedule the first RATSREPORT immediately.
            scheduler.AddAction(ACTION_RATS_REPORT, 0);
        }

        // Now just loop in STANDBY mode
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
        // Enable LoRa transmit testing, for RF interference validation.
        if (CheckAction(ACTION_LORA_TX_TEST) && lora_tx_test) {
            char msg[] = "LoRa TX test message from StratoCore_RATS abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$%^&*()_+-=[]{}|;:,.<>?";
            LoRaTx(msg, true);
            scheduler.AddAction(ACTION_LORA_TX_TEST, 1);
        }
        // Trigger an MCB message
        if (CheckAction(ACTION_MCB_INIT_MOTION)) {
            InitializeReelPosition();
        }
        // Check if it's time to send a RATS report, and if so, trigger it and schedule the next one.
        if (CheckAction(ACTION_RATS_REPORT)) {
            ratsReportCheck(true);
            scheduler.AddAction(ACTION_RATS_REPORT, 600); // every 10 minutes
        }
        break;
    case SB_SHUTDOWN:
        // prep for shutdown
        log_nominal("Shutdown warning received in SB");
        RATS_Shutdown();
        lora_tx_test = false;
        break;
    case SB_EXIT:
        // perform cleanup
        log_nominal("Exiting SB");
        lora_tx_test = false;
        break;
    default:
        // todo: throw error
        log_error("Unknown substate in SB");
        inst_substate = SB_ENTRY; // reset
        log_nominal("Entering SB_ENTRY");
        break;
    }
}
