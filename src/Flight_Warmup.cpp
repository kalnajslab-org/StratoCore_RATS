#include "StratoRATS.h"
#include <ArduinoJson.h>

enum WarmupStates_t
{
    WARMUP_ENTRY,
    WARMUP_LORA_WAIT1,
    WARMUP_CONFIG_ECU,
    WARMUP_LORA_WAIT2
};

static WarmupStates_t warmup_state = WARMUP_ENTRY;

static JsonDocument ecu_json;
static char ecu_json_str[ECU_LORA_DATA_BUFSIZE];

bool StratoRATS::Flight_Warmup(bool restart)
{
    if (restart)
    {
        warmup_state = WARMUP_ENTRY;
        warmup_status = WARMUP_INPROCESS;
        warmup_cycles = 0;
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
        // Power on ECU
        log_nominal("WARMUP_ENTRY Powering on ECU");
        // Start the LoRa message counter
        lora_count_check(true);
        LoRaMsg_timer_start = now();
        scheduler.AddAction(ACTION_LORA_COUNT_MSGS, 1);
        warmup_state = WARMUP_LORA_WAIT1;
        log_nominal("Entering WARMUP_WAIT1");
        break;

    case WARMUP_LORA_WAIT1:
        if (LoRaMsg_timer_start + LORA_WARMUP_MSG_TIMEOUT < now())
        {
            warmup_cycles++;
            if (warmup_cycles >= 2)
            {
                log_error("WARMUP_LORA_WAIT1 Too many LoRa message timeouts");
                ZephyrLogCrit("LoRa message timeouts during warmup wait1");
                warmup_status = WARMUP_FAILED;
                return(true);
            }
            else
            {
                warmup_state = WARMUP_ENTRY;
                log_nominal("Re-entering WARMUP_ENTRY");
                return(false);
            }
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
        ecu_json["tempC"] = ratsConfigs.ecu_tempC.Read();
        serializeJson(ecu_json, ecu_json_str);
        log_nominal(ecu_json_str);
        // Send the configuration message to the ECU
        ecu_lora_tx((uint8_t*)ecu_json_str, strlen(ecu_json_str));

        LoRaMsg_timer_start = now();
        scheduler.AddAction(ACTION_LORA_COUNT_MSGS, 1);
        warmup_state = WARMUP_LORA_WAIT2;
        // Start the LoRa message counter
        lora_count_check(true);
        log_nominal("Entering WARMUP_LORA_WAIT2");
        log_nominal("WARMUP_LORA_WAIT2 waiting for LoRa message");
        break;
    case WARMUP_LORA_WAIT2:
        if (LoRaMsg_timer_start + LORA_WARMUP_MSG_TIMEOUT < now())
        {
            warmup_cycles++;
            if (warmup_cycles >= 2)
            {
                log_error("WARMUP_LORA_WAIT2 Too many LoRa message timeouts");
                ZephyrLogCrit("LoRa message timeouts during warmup wait2");
                warmup_status = WARMUP_FAILED;
                return(true);
            }
            else
            {
                warmup_state = WARMUP_ENTRY;
                log_nominal("Re-entering WARMUP_ENTRY");
                return(false);
            }
        }
        else
        {
            if (CheckAction(ACTION_LORA_COUNT_MSGS))
            {
                // Wait for enough LoRa messages to arrive.
                if (lora_count_check() >= LORA_MSG_COUNT)
                {
                    log_nominal("WARMUP_LORA_WAIT2 Required LoRa messages received");
                    ZephyrLogFine("Warmup complete");
                    warmup_status = WARMUP_COMPLETE;
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