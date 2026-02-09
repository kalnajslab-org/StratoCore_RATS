/*
 *  MCBRouter.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 *
 *  This file implements the RACHuTS Motor Control Board message router and handlers.
 */

#include "StratoRATS.h"
#include "Serialize.h"

void StratoRATS::RunMCBRouter()
{
    SerialMessage_t rx_msg = mcbComm.RX();

    while (NO_MESSAGE != rx_msg) {
        if (ASCII_MESSAGE == rx_msg) {
            HandleMCBASCII();
        } else if (ACK_MESSAGE == rx_msg) {
            HandleMCBAck();
        } else if (BIN_MESSAGE == rx_msg) {
            HandleMCBBin();
        } else if (STRING_MESSAGE == rx_msg) {
            HandleMCBString();
        } else {
            log_error("Unknown message type from MCB");
        }

        rx_msg = mcbComm.RX();
    }
}

void StratoRATS::HandleMCBASCII()
{
    switch (mcbComm.ascii_rx.msg_id) {
    case MCB_VOLTAGES:
        float mcb_voltages[4];
        if (mcbComm.RX_Voltages(mcb_voltages, mcb_voltages+1, mcb_voltages+2, mcb_voltages+3)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "MCBASCII: MCB voltages: %.1f,%.1f,%.1f,%.1f", mcb_voltages[0], mcb_voltages[1],
                     mcb_voltages[2], mcb_voltages[3]);
            SendMCBTM(FINE, log_array);
        } else {
            SendMCBTM(CRIT, "MCBASCII: Error receiving MCB voltages");
        }
        break;
    case MCB_MOTION_FINISHED:
        CheckAction(ACTION_MOTION_TIMEOUT); // clear the timeout
        log_nominal("MCBASCII: MCB motion finished"); // state machine will report to Zephyr
        mcb_motion_ongoing = false;
        break;
    case MCB_MOTION_FAULT:
        CheckAction(ACTION_MOTION_TIMEOUT); // clear the timeout
        // if flag already cleared, assume this is the repeat
        if (!mcb_motion_ongoing) {
            return;
        }

        if (mcbComm.RX_Motion_Fault(motion_fault, motion_fault+1, motion_fault+2, motion_fault+3,
                                    motion_fault+4, motion_fault+5, motion_fault+6, motion_fault+7)) {
            // The MCB MCB_MOTION_FAULT message was successfully decoded. 
            // However, there was still a motion fault.
            mcb_motion_ongoing = false;
            snprintf(log_array, LOG_ARRAY_SIZE, "MCBASCII: MCB Fault: %x,%x,%x,%x,%x,%x,%x,%x", motion_fault[0], motion_fault[1],
                     motion_fault[2], motion_fault[3], motion_fault[4], motion_fault[5], motion_fault[6], motion_fault[7]);
            SendMCBTM(CRIT, log_array);
            inst_substate = MODE_ERROR;
            log_error(log_array);
            log_error("MCBASCII: Entering FL_ERROR MCB_MOTION_FAULT");
        } else {
            // The MCB MCB_MOTION_FAULT message was unsuccessfully decoded. 
            // However, there was still a motion fault.
            mcb_motion_ongoing = false;
            SendMCBTM(CRIT, "MCBASCII: MCB fault, error receiving MCB parameters, motion terminated");
            inst_substate = MODE_ERROR;
            log_error("MCBASCII: MCB fault, error receiving parameters");
            log_error("MCBASCII: Entering FL_ERROR");
        }
        break;
    default:
        log_error("Unknown MCB ASCII message received");
        break;
    }
}

void StratoRATS::HandleMCBAck()
{
    switch (mcbComm.ack_id) {
    case MCB_CANCEL_MOTION:
        log_nominal("MCBACK: acked cancel motion");
        mcb_motion = NO_MOTION;
        mcb_motion_ongoing = false;
        break;
    case MCB_GO_LOW_POWER:
        log_nominal("MCBACK: acked in low power");
        mcb_low_power = true;
        break;
    case MCB_REEL_IN:
        if (MOTION_REEL_IN == mcb_motion) { 
            InitMCBMotionTracking();
        }
        break;
    case MCB_REEL_OUT:
        if (MOTION_REEL_OUT == mcb_motion) {
            InitMCBMotionTracking();
        }
        break;
    case MCB_IN_NO_LW:
        if (MOTION_IN_NO_LW == mcb_motion) {
            InitMCBMotionTracking();
        }
        break;
    case MCB_FULL_RETRACT:
        mcb_reeling_in = true;
        break;
    case MCB_IN_ACC:
        SendMCBTM(FINE, "MCBACK: acked retract acc");
        break;
    case MCB_OUT_ACC:
        SendMCBTM(FINE, "MCBACK: acked deploy acc");
        break;
    case MCB_ZERO_REEL:
        SendMCBTM(FINE, "MCBACK: acked zero reel");
        break;
    case MCB_TEMP_LIMITS:
        SendMCBTM(FINE, "MCBACK: acked temp limits");
        break;
    case MCB_TORQUE_LIMITS:
        SendMCBTM(FINE, "MCBACK: acked torque limits");
        break;
    case MCB_CURR_LIMITS:
        SendMCBTM(FINE, "MCBACK: acked curr limits");
        break;
    case MCB_IGNORE_LIMITS:
        SendMCBTM(FINE, "MCBACK: acked ignore limits");
        break;
    case MCB_USE_LIMITS:
        SendMCBTM(FINE, "MCBACK: acked use limits");
        break;
    case MCB_GET_EEPROM:
        SendMCBTM(FINE, "MCBACK: acked get MCB eeprom");
        break;
    case MCB_GET_VOLTAGES:
        SendMCBTM(FINE, "MCBACK: acked get MCB voltages");
        break;
    default:
        log_error(String(String("MCBACK: Unexpected MCB ACK received:")+String(mcbComm.ack_id)).c_str());
        break;
    }
}

void StratoRATS::HandleMCBBin()
{
    // reel_pos_index is the location in the binary data sent from the MCB. 
    // That buffer is filled in MonitorMCB::SendMotionData(void),
    // look in that code to determine the offset.
    uint16_t reel_pos_index = 21; // todo: don't hard-code this

    switch (mcbComm.binary_rx.bin_id) {
    case MCB_MOTION_TM:
        if (BufferGetFloat(&reel_pos, mcbComm.binary_rx.bin_buffer, mcbComm.binary_rx.bin_length, &reel_pos_index)) {
            log_nominal((String("Reel pos: ") + String(reel_pos)).c_str());
        } else {
            log_nominal("Received MCB bin: unable to read position");
        }
        AddMCBTM();
        break;
    case MCB_EEPROM:
        SendMCBEEPROM();
        break;
    default:
        log_error("Unknown MCB bin received");
    }
}

void StratoRATS::HandleMCBString()
{
    switch (mcbComm.string_rx.str_id) {
    case MCB_ERROR:
        if (mcbComm.RX_Error(log_array, LOG_ARRAY_SIZE)) {
            String msg = String("MCBString: ") + String(log_array);
            SendMCBTM(CRIT, msg.c_str());
#if not DISABLE_DEVEL_ERROR_CHECKING
            inst_substate = MODE_ERROR;
            log_error("MCBString: Entering FL_ERROR HandleMCBString()");
#else
            log_error((String("DISABLE_DEVEL_ERROR_CHECKING is enabled, MCB error will be ignored: ")+log_array).c_str());  
#endif
        }
        break;
    default:
        log_error("Unknown MCB String message received");
        break;
    }
}

