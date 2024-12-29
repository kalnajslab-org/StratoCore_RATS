#include "StratoRATS.h"

enum ManualMotionStates_t {
    ST_ENTRY,
    ST_SEND_RA,
    ST_WAIT_RAACK,
    ST_START_MOTION,
    ST_VERIFY_MOTION,
    ST_MONITOR_MOTION,
    ST_TM_ACK,
};

static ManualMotionStates_t manualmotion_state = ST_ENTRY;
static bool resend_attempted = false;

bool StratoRATS::Flight_ManualMotion(bool restart_state)
{
    if (restart_state) {
        manualmotion_state = ST_ENTRY;
    }
    switch (manualmotion_state) {
    case ST_ENTRY:
    case ST_SEND_RA:
        RA_ack_flag = NO_ACK;
        zephyrTX.RA();
        manualmotion_state = ST_WAIT_RAACK;
        scheduler.AddAction(RESEND_RA, ZEPHYR_RESEND_TIMEOUT);
        log_nominal("Entering ST_WAIT_RAACK");
        break;

    case ST_WAIT_RAACK:
        //TODO: This code came from StratoCore_RACHUTS::Flight_ManualMotion(), which
        // was recently added by Lars. PIBConfigs::ra_override is not defined in that system either.
        //if(RATSConfigs.ra_override.Read()) //Over Ride RA requirement in an emergency
        //    RA_ack_flag = ACK;
        if (ACK == RA_ack_flag) {
            manualmotion_state = ST_START_MOTION;
            resend_attempted = false;
            log_nominal("Entering ST_START_MOTION");
        } else if (NAK == RA_ack_flag) {
            resend_attempted = false;
            ZephyrLogWarn("Cannot perform motion, RA NAK");
            return true;
        } else if (CheckAction(RESEND_RA)) {
            if (!resend_attempted) {
                resend_attempted = true;
                manualmotion_state = ST_SEND_RA;
                log_nominal("Entering ST_SEND_RA");
            } else {
                ZephyrLogWarn("Never received RAAck");
                resend_attempted = false;
                return true;
            }
        }
        break;

    case ST_START_MOTION:
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Motion commanded while motion ongoing");
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            log_nominal("Entering MODE_ERROR");
        }

        if (StartMCBMotion()) {
            manualmotion_state = ST_VERIFY_MOTION;
            scheduler.AddAction(RESEND_MOTION_COMMAND, MCB_RESEND_TIMEOUT);
            log_nominal("Entering ST_VERIFY_MOTION");
        } else {
            ZephyrLogWarn("Motion start error");
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            log_nominal("Entering MODE_ERROR");
        }
        break;

    case ST_VERIFY_MOTION:
        if (mcb_motion_ongoing) { // set in the Ack handler
            log_nominal("MCB commanded motion");
            scheduler.AddAction(ACTION_MOTION_TIMEOUT, max_profile_seconds);
            manualmotion_state = ST_MONITOR_MOTION;
            log_nominal("Entering ST_MONITOR_MOTION");
        }

        if (CheckAction(RESEND_MOTION_COMMAND)) {
            if (!resend_attempted) {
                resend_attempted = true;
                manualmotion_state = ST_START_MOTION;
                log_nominal("Entering ST_START_MOTION");
            } else {
                resend_attempted = false;
                ZephyrLogWarn("MCB never confirmed motion");
                inst_substate = MODE_ERROR; // will force exit of Flight_Profile
                log_nominal("Entering MODE_ERROR");
            }
        }
        break;

    case ST_MONITOR_MOTION:
        log_nominal("ST_MONITOR_MOTION Monitoring motion");
        if (CheckAction(ACTION_MOTION_STOP)) {
            // todo: verification of motion stop
            ZephyrLogFine("Commanded motion stop");
            return true;
            break;
        }

        if (CheckAction(ACTION_MOTION_TIMEOUT)) {
            SendMCBTM(CRIT, "MCB Motion took longer than expected");
            mcbComm.TX_ASCII(MCB_CANCEL_MOTION);
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            log_nominal("Entering MODE_ERROR");
            break;
        }

        if (!mcb_motion_ongoing) {
            SendMCBTM(FINE, "Finished commanded manual motion");
            manualmotion_state = ST_TM_ACK;
            scheduler.AddAction(RESEND_TM, ZEPHYR_RESEND_TIMEOUT);
            log_nominal("Entering ST_TM_ACK");
        }
        break;

    case ST_TM_ACK:
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

    return false; // assume incomplete
}