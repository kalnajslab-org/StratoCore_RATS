#include "StratoRATS.h"

enum WarmupStates_t
{
    WARMUP_ENTRY,
    WARMUP_LORA_WAIT1,
    WARMUP_CONFIG_ECU,
    WARMUP_LORA_WAIT2
};

static WarmupStates_t warmup_state = WARMUP_ENTRY;

bool StratoRATS::Flight_Warmup(bool restart)
{
    if (restart)
    {
        warmup_state = WARMUP_ENTRY;

    }

#if EXTRA_LOGGING
    static uint old_warmup_state = 256;
    if (warmup_state != old_warmup_state) {
        log_nominal((String("warmup_state:" + String(warmup_state)).c_str()));
        old_warmup_state = warmup_state;
    }
#endif

    switch (warmup_state)
    {
    case WARMUP_ENTRY:
        // Start the LoRa message counter
        lora_count_check(true);
        LoRa_timer_start = now();
        scheduler.AddAction(ACTION_LORA_COUNT_MSGS, 1);
        warmup_state = WARMUP_LORA_WAIT1;
        log_nominal("Entering WARMUP_WAIT1");
        break;

    case WARMUP_LORA_WAIT1:
        if (LoRa_timer_start + LORA_WARMUP_MSG_TIMEOUT < now())
        {
            log_error("WARMUP_LORA_WAIT1 Expected LoRa messages not received");
            ZephyrLogWarn("LoRa messages not received during first wait");
            inst_substate = MODE_ERROR;
            log_error("Entering FL_ERROR");
        }
        else
        {
            if (CheckAction(ACTION_LORA_COUNT_MSGS))
            {
                // Wait for enough LoRa message to arrive.
                if (lora_count_check() >= LORA_MSG_COUNT)
                {
                    log_nominal("WARMUP_LORA_WAIT1 Required LoRa messages received");
                    warmup_state = WARMUP_CONFIG_ECU;
                    log_nominal("Entering WARMUP_CONFIG_ECU");
                }
                else
                {
                    scheduler.AddAction(ACTION_LORA_COUNT_MSGS, 1);
                }
            }
        }
        break;
    case WARMUP_CONFIG_ECU:
        // Configure the ECU here.
        log_nominal("WARMUP_CONFIG_ECU Configuring ECU");
        LoRa_timer_start = now();
        scheduler.AddAction(ACTION_LORA_COUNT_MSGS, 1);
        warmup_state = WARMUP_LORA_WAIT2;
        // Start the LoRa message counter
        lora_count_check(true);
        log_nominal("Entering WARMUP_LORA_WAIT2");
        log_nominal("WARMUP_LORA_WAIT2 waiting for LoRa message");
        break;
    case WARMUP_LORA_WAIT2:
        if (LoRa_timer_start + LORA_WARMUP_MSG_TIMEOUT < now())
        {
            log_error("WARMUP_LORA_WAIT2 Expected LoRa messages not received");
            ZephyrLogWarn("LoRa messages not received during second wait");
            inst_substate = MODE_ERROR;
            log_error("Entering FL_ERROR");
        }
        else
        {
            if (CheckAction(ACTION_LORA_COUNT_MSGS))
            {
                // Wait for enough LoRa messages to arrive.
                if (lora_count_check() >= LORA_MSG_COUNT)
                {
                    log_nominal("WARMUP_LORA_WAIT2 Required LoRa messages received");
                    return true;
                }
                else
                {
                    scheduler.AddAction(ACTION_LORA_COUNT_MSGS, 1);
                }
            }
        }
        break;

    default:
        // unknown state, move on!
        return true;
    }

    return false; //  // remain in this mode, unless we have changed inst_substate
}