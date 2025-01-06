#include "StratoRATS.h"

enum ReelStates_t {
    REEL_ENTRY,
    REEL_SEND_RA,
    REEL_WAIT_RAACK,
    REEL_START_MOTION,
    REEL_VERIFY_MOTION,
    REEL_MONITOR_MOTION,
    REEL_TM_ACK,
};

static ReelStates_t reel_state = REEL_ENTRY;
static bool resend_attempted = false;

bool StratoRATS::Flight_Reel(bool restart_state)
{
    if (restart_state) {
        reel_state = REEL_ENTRY;
        log_nominal("Entering REEL_ENTRY");
    }
    switch (reel_state) {
    case REEL_ENTRY:
    case REEL_SEND_RA:
        RA_ack_flag = NO_ACK;
        zephyrTX.RA();
        reel_state = REEL_WAIT_RAACK;
        scheduler.AddAction(RESEND_RA, ZEPHYR_RESEND_TIMEOUT);
        log_nominal("Entering REEL_WAIT_RAACK");
        break;

    case REEL_WAIT_RAACK:
        //TODO: This code came from StratoCore_RACHUTS::Flight_Reel(), which
        // was recently added by Lars. PIBConfigs::ra_override is not defined in that system either.
        //if(RATSConfigs.ra_override.Read()) //Over Ride RA requirement in an emergency
        //    RA_ack_flag = ACK;
        if (CheckAction(ACTION_MOTION_STOP)) {
            // todo: verification of motion stop
            ZephyrLogFine("Commanded motion stop");
            return true;
            break;
        } else if (ACK == RA_ack_flag) {
            reel_state = REEL_START_MOTION;
            resend_attempted = false;
            log_nominal("Entering REEL_START_MOTION");
        } else if (NAK == RA_ack_flag) {
            resend_attempted = false;
            ZephyrLogWarn("Cannot perform motion, RA NAK");
            return true;
        } else if (CheckAction(RESEND_RA)) {
            if (!resend_attempted) {
                resend_attempted = true;
                reel_state = REEL_SEND_RA;
                log_nominal("Entering REEL_SEND_RA");
            } else {
                ZephyrLogWarn("Never received RAAck");
                resend_attempted = false;
                return true;
            }
        }
        break;

    case REEL_START_MOTION:
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Motion commanded while motion ongoing");
            log_error("Motion commanded while motion ongoing");
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            log_error("Entering MODE_ERROR");
        }

        if (StartMCBMotion()) {
            reel_state = REEL_VERIFY_MOTION;
            scheduler.AddAction(RESEND_MOTION_COMMAND, MCB_RESEND_TIMEOUT);
            log_nominal("Entering REEL_VERIFY_MOTION");
        } else {
            ZephyrLogWarn("Motion start error");
            log_error("Motion start error");
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            log_error("Entering MODE_ERROR");
        }
        break;

    case REEL_VERIFY_MOTION:
        if (mcb_motion_ongoing) { // set in the Ack handler
            log_nominal("MCB commanded motion");
            // max_profile_seconds was set in StartMCBMotion()
            scheduler.AddAction(ACTION_MOTION_TIMEOUT, max_profile_seconds);
            reel_state = REEL_MONITOR_MOTION;
            log_nominal("Entering REEL_MONITOR_MOTION");
        }

        if (CheckAction(RESEND_MOTION_COMMAND)) {
            if (!resend_attempted) {
                resend_attempted = true;
                reel_state = REEL_START_MOTION;
                log_nominal("Entering REEL_START_MOTION");
            } else {
                resend_attempted = false;
                ZephyrLogWarn("MCB never confirmed motion");
                log_error("MCB never confirmed motion");
                inst_substate = MODE_ERROR; // will force exit of Flight_Profile
                log_error("Entering MODE_ERROR");
            }
        }
        break;

    case REEL_MONITOR_MOTION:
        if (CheckAction(ACTION_MOTION_STOP)) {
            // todo: verification of motion stop
            ZephyrLogFine("Commanded motion stop");
            return true;
            break;
        }

        if (CheckAction(ACTION_MOTION_TIMEOUT)) {
            SendMCBTM(CRIT, "MCB Motion took longer than expected");
            log_error("MCB Motion took longer than expected");
            mcbComm.TX_ASCII(MCB_CANCEL_MOTION);
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            log_error("Entering MODE_ERROR");
            break;
        }

        if (!mcb_motion_ongoing) {
            SendMCBTM(FINE, "Finished commanded reel motion");
            reel_state = REEL_TM_ACK;
            scheduler.AddAction(RESEND_TM, ZEPHYR_RESEND_TIMEOUT);
            log_nominal("Entering REEL_TM_ACK");
        }
        break;

    case REEL_TM_ACK:
        if (ACK == TM_ack_flag) {
            log_nominal("Zephyr ACKed motion TM");
            return true;
        } else if (NAK == TM_ack_flag || CheckAction(RESEND_TM)) {
            // attempt one resend
            log_error("Needed to resend TM");
            zephyrTX.TM(); // message is still saved in XMLWriter, no need to reconstruct
            return true;
        }
        break;

    default:
        // unknown state, exit
        return true;
    }

    return false; // remain in this mode, unless we have changed inst_substate
}