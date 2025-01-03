#ifndef _RATSHARDWARE_H_
#define _RATSHARDWARE_H_

// Serial connection to MCB
#define MCB_SERIAL      Serial3

#define SAFE_PIN        31

// LoRa Module
#define SS_PIN          38
#define RESET_PIN       29
#define INTERRUPT_PIN   37
#define LORA_SCK        27
#define LORA_MISO       39
#define LORA_MOSI       26

//LoRa Settings
#define FREQUENCY 868E6
#define BANDWIDTH 250E3
#define SF 9
#define RF_POWER 19


#endif /* _RATSHARDWARE_H_ */