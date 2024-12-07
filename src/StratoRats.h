#ifndef STRATORATS_H
#define STRATORATS_H

#include <time.h>
#include "StratoCore.h"
#include "RS41.h"

// WARNING: DO NOT CHECK CODE INTO GIT WIH THIS OPTION ENABLED. 
//          MAKE SURE THIS OPTION IS DISABLED FOR FLIGHT DEPLOYED FIRMWARE.
#define ZEPHYR_COMMS_ON_DEBUG_PORT 1

#if not ZEPHYR_COMMS_ON_DEBUG_PORT
#define ZEPHYR_SERIAL   Serial8
#else
// This allows for use of the OBD_Simulator with just the Teensy programming port, 
// by sharing it for both Zephyr and StratoCore log messages.
#define ZEPHYR_SERIAL   Serial
#endif

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
