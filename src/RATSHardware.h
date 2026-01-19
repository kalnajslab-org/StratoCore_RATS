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
#define FREQUENCY       868500000
#define BANDWIDTH       250E3
#define SF              9
#define TX_POWER        19

// A3_INST_IMON
#define A3_INST_IMON    A3

// ACS71240 current sensor
// Part number on chip: ACS71240-2315142K. Unfortunately this 
// does not provide the actual part number needed for specifications.
// Sensitivity in A/V. Note that the polarity is reversed, so the current 
// is negative when the voltage is above the zero current voltage.
#define ACS71240_A_PER_V (-1/0.132)
#define ACS71240_ZERO_CURRENT_V 1.65

// A17/A20_V56_VMON
#define V56_MON         A17

// V56_MON voltage divider resistors
#define R8              1000.0
#define R9              49900.0

#endif /* _RATSHARDWARE_H_ */