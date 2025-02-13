#ifndef _RATSHARDWARE_H_
#define _RATSHARDWARE_H_

// Serial connection to MCB
#define MCB_SERIAL      Serial3

// Zephyr Safe Pin
#define SAFE_PIN        31

// ECU Power Enable
#define ECU_PWR_EN      5

// LoRa Module
#define RATS_LORA_CS    38
#define RATS_LORA_RST   29
#define RATS_LORA_INT   37
#define RATS_LORA_SCK   27
#define RATS_LORA_MISO  39
#define RATS_LORA_MOSI  26

// LoRa Settings
#define FREQUENCY       868E6
#define BANDWIDTH       250E3
#define SF              9
#define RF_POWER        19

#endif /* _RATSHARDWARE_H_ */