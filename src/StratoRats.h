#ifndef STRATORATS_H
#define STRATORATS_H

#include <time.h>
#include "StratoCore.h"

// for testing purposes, use LPC
#define ZEPHYR_SERIAL   Serial8 // Teensy 4.1
#define INSTRUMENT      RATS
#define ZEPHYR_SERIAL_BUFFER_SIZE 4096

// number of loops before a flag becomes stale and is reset
#define FLAG_STALE      2

// todo: perhaps more creative/useful enum here by mode with separate arrays?
// WARNING: this construct assumes that NUM_ACTIONS will be equal to the number
// of actions. Never seen this coding style before; seems dangerous.
enum ScheduleAction_t : uint8_t {
    NO_ACTION = NO_SCHEDULED_ACTION,
    SEND_IMR,
    START_TELEMETRY,
    RESEND_SAFETY,
    GPS_WAIT_MSG,
    NUM_ACTIONS
};

class StratoRATS : public StratoCore {
public:
    StratoRATS();
    ~StratoRATS() { };

    // called before the loop begins
    void InstrumentSetup();

    // called at the end of each loop
    void InstrumentLoop();

private:
    // Mode functions (implemented in unique source files)
    void StandbyMode();
    void FlightMode();
    void LowPowerMode();
    void SafetyMode();
    void EndOfFlightMode();
    
    void RATS_Shutdown();

    // Telcommand handler - returns ack/nak
    bool TCHandler(Telecommand_t telecommand);

    // Action handler for scheduled actions
    void ActionHandler(uint8_t action);

    // Safely check and clear action flags
    bool CheckAction(uint8_t action);

    // Monitor the action flags and clear old ones
    void WatchFlags();
    
    // Actions
    ActionFlag_t action_flags[NUM_ACTIONS] = {{0}}; // initialize all flags to false
};
#endif /* STRATORATS_H */
