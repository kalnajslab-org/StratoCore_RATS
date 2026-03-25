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
            SendMCBTM("MCBASCII", FINE, log_array);
        } else {
            SendMCBTM("MCBASCII", CRIT, "MCBASCII: Error receiving MCB voltages");
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

            // motion_fault[] fields, to decode, use the following maps:
            //   [0] rl_status_lo  — Reel SRL (Status Register Low),  use SRLmap()
            //   [1] rl_status_hi  — Reel SRH (Status Register High), use SRHmap()
            //   [2] rl_detailed_err — Reel detailed error (MCB-specific, no bit map)
            //   [3] rl_motion_err — Reel MER (Motion Error Register), use MERmap()
            //   [4] lw_status_lo  — Level Wind SRL,                  use SRLmap()
            //   [5] lw_status_hi  — Level Wind SRH,                  use SRHmap()
            //   [6] lw_detailed_err — Level Wind detailed error (MCB-specific, no bit map)
            //   [7] lw_motion_err — Level Wind MER,                  use MERmap()

            mcb_motion_ongoing = false;

            snprintf(log_array, LOG_ARRAY_SIZE, "MCBASCII: MCB Fault: %x,%x,%x,%x,%x,%x,%x,%x", motion_fault[0], motion_fault[1],
                     motion_fault[2], motion_fault[3], motion_fault[4], motion_fault[5], motion_fault[6], motion_fault[7]);
            SendMCBTM("MCBASCII", CRIT, log_array);
            if (motion_fault[3]) { 
                SendMCBTM("MCBASCII", CRIT, String("RL MER:" + MERmap(motion_fault[3])).c_str()); 
            }
            if (motion_fault[7]) { 
                SendMCBTM("MCBASCII", CRIT, String("LW MER:" + MERmap(motion_fault[7])).c_str()); 
            }

            inst_substate = MODE_ERROR;
            log_error(log_array);
            log_error("MCBASCII: Entering FL_ERROR MCB_MOTION_FAULT");
        } else {
            // The MCB MCB_MOTION_FAULT message was unsuccessfully decoded. 
            // However, there was still a motion fault.
            mcb_motion_ongoing = false;
            SendMCBTM("MCBASCII", CRIT, "MCBASCII: MCB fault, error receiving MCB parameters, motion terminated");
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
        SendMCBTM("MCBACK", FINE, "MCBACK: acked retract acc");
        break;
    case MCB_OUT_ACC:
        SendMCBTM("MCBACK", FINE, "MCBACK: acked deploy acc");
        break;
    case MCB_ZERO_REEL:
        SendMCBTM("MCBACK", FINE, "MCBACK: acked zero reel");
        break;
    case MCB_TEMP_LIMITS:
        SendMCBTM("MCBACK", FINE, "MCBACK: acked temp limits");
        break;
    case MCB_TORQUE_LIMITS:
        SendMCBTM("MCBACK", FINE, "MCBACK: acked torque limits");
        break;
    case MCB_CURR_LIMITS:
        SendMCBTM("MCBACK", FINE, "MCBACK: acked curr limits");
        break;
    case MCB_IGNORE_LIMITS:
        SendMCBTM("MCBACK", FINE, "MCBACK: acked ignore limits");
        break;
    case MCB_USE_LIMITS:
        SendMCBTM("MCBACK", FINE, "MCBACK: acked use limits");
        break;
    case MCB_GET_EEPROM:
        SendMCBTM("MCBACK", FINE, "MCBACK: acked get MCB eeprom");
        break;
    case MCB_GET_VOLTAGES:
        SendMCBTM("MCBACK", FINE, "MCBACK: acked get MCB voltages");
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
            SendMCBTM("MCBSTRING", CRIT, msg.c_str());
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

String StratoRATS::MERmap(uint16_t mer) {
    // MER - Motion Error Register: https://www.technosoftmotion.com/ESM-um-html/tml_mer.htm
    static const char* const bit_names[16] = {
        "CANBER",   // 0:  CAN bus error
        "SCER",     // 1:  Short-circuit protection
        "STPTBL",   // 2:  Invalid setup table
        "CTRER",    // 3:  Control error
        "SCIER",    // 4:  Serial/internal communication error
        "WRPSER",   // 5:  Hall/resolver/BiSS/wrap-around error
        "LSPST",    // 6:  Positive limit switch active
        "LSNST",    // 7:  Negative limit switch active
        "OCER",     // 8:  Over-current error
        "I2TER",    // 9:  I2T protection error
        "OTERM",    // 10: Motor over-temperature error
        "OTERD",    // 11: Drive over-temperature error
        "OVER",     // 12: Over-voltage error
        "UVER",     // 13: Under-voltage error
        "CMDER",    // 14: Command error
        "ENST",     // 15: Drive/motor disabled
    };
    String result = "";
    for (uint8_t i = 0; i < 16; i++) {
        if (mer & (1 << i)) {
            if (result.length() > 0) result += ",";
            result += bit_names[i];
        }
    }
    return result.length() > 0 ? result : "none";
}

String StratoRATS::SRLmap(uint16_t srl) {
    // SRL - Status Register Low: https://www.technosoftmotion.com/ESM-um-html/tml_srl.htm
    // Only defined bits listed; reserved bits skipped.
    struct { uint8_t bit; const char* name; } bits[] = {
        { 15, "AXISST" },    // Axis on
        { 14, "EVNS" },      // Last programmed event reached
        { 10, "MOTS" },      // Motion complete
        {  8, "CALLSST" },   // Function running via cancelable call
        {  7, "CALLWRG" },   // Cancelable call warning
    };
    String result = "";
    for (auto& b : bits) {
        if (srl & (1 << b.bit)) {
            if (result.length() > 0) result += ",";
            result += b.name;
        }
    }
    return result.length() > 0 ? result : "none";
}

String StratoRATS::SRHmap(uint16_t srh) {
    // SRH - Status Register High: https://www.technosoftmotion.com/ESM-um-html/tml_srh.htm
    static const char* const bit_names[16] = {
        "ENDINIT",   // 0:  Drive/motor initialization complete
        "PTRG1",     // 1:  Position trigger 1 active
        "PTRG2",     // 2:  Position trigger 2 active
        "PTRG3",     // 3:  Position trigger 3 active
        "PTRG4",     // 4:  Position trigger 4 active
        "AUTORUN",   // 5:  AUTORUN mode enabled
        "LSWPS",     // 6:  Positive limit switch event
        "LSWNS",     // 7:  Negative limit switch event
        "PCAPS",     // 8:  Capture event triggered
        "TRGR",      // 9:  Target command achieved
        "I2TWRGM",   // 10: Motor I2T warning
        "I2TWRGD",   // 11: Drive I2T warning
        "INGEAR",    // 12: Electronic gearing ratio achieved
        nullptr,     // 13: Reserved
        "INCAM",     // 14: Absolute electronic camming position reached
        "FAULT",     // 15: Drive/motor in fault
    };
    String result = "";
    for (uint8_t i = 0; i < 16; i++) {
        if (bit_names[i] && (srh & (1 << i))) {
            if (result.length() > 0) result += ",";
            result += bit_names[i];
        }
    }
    return result.length() > 0 ? result : "none";
}