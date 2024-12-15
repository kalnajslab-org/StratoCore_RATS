#ifndef STRATORATS_H
#define STRATORATS_H

#include <time.h>
#include "StratoCore.h"
#include "RATSConfigs.h"
#include "MCBComm.h"

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

// Our instrument name
#define INSTRUMENT      RATS
// Buffers for msg reception and transmission to/from Zephyr. Should be large enough
// to hold a complete TM, some of which which will contain the measurement data.
#define ZEPHYR_SERIAL_BUFFER_SIZE 4096
// Serial connection to MCB
#define MCB_SERIAL      Serial3
// Reporting period for status message generation, including TM transmission.
#define STATUS_MSG_PERIOD_SECS 60
// Number of loops before a flag becomes stale and is reset
#define FLAG_STALE      2
// The size of a buffer used for binary transfers between RATS and MCB.
#define MCB_BINARY_BUFFER_SIZE MAX_MCB_BINARY
#define HEARTBEAT_LED_PIN	3

// todo: perhaps more creative/useful enum here by mode with separate arrays?
// WARNING: this construct assumes that NUM_ACTIONS will be equal to the number
// of actions. Never seen this coding style before; seems dangerous.
enum ScheduleAction_t : uint8_t {
    NO_ACTION = NO_SCHEDULED_ACTION,
    SEND_IMR,
    START_TELEMETRY,
    RESEND_SAFETY,
    GPS_WAIT_MSG,
    SEND_STATUS,

    ACTION_MOTION_TIMEOUT,

    NUM_ACTIONS
};

enum MCBMotion_t : uint8_t {
    NO_MOTION,
    MOTION_REEL_IN,
    MOTION_REEL_OUT,
    MOTION_DOCK,
    MOTION_IN_NO_LW
};

class StratoRATS : public StratoCore {
public:
    StratoRATS();
    ~StratoRATS() { };

    // called before the loop begins
    void InstrumentSetup();

    // called at the end of each loop
    void InstrumentLoop();

    // called in each main loop
    void RunMCBRouter();

private:
    // internal serial interface objects for the MCB and ECU
    MCBComm mcbComm;

    // EEPROM interface object
    RATSConfigs ratsConfigs;

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

    // Handle messages from the MCB (in MCBRouter.cpp)
    void HandleMCBASCII();
    void HandleMCBAck();
    void HandleMCBBin();
    void HandleMCBString();

    // Send a telemetry packet with MCB binary info
    void SendMCBTM(StateFlag_t state_flag, const char * message);

    uint8_t binary_mcb[MCB_BINARY_BUFFER_SIZE];

    // flags for MCB state tracking
    bool mcb_low_power = false;
    bool mcb_motion_ongoing = false;
    bool mcb_dock_ongoing = false;
    uint32_t max_profile_seconds = 0;
    bool mcb_reeling_in = false;
    uint16_t mcb_tm_counter = 0;

    // array of error values for MCB motion fault
    uint16_t motion_fault[8] = {0};
    uint8_t MCB_TM_buffer[8192] = {0};
    uint16_t MCB_TM_buffer_idx = 0;

    // tracks the current type of motion
    MCBMotion_t mcb_motion = NO_MOTION;

    // uint32_t start time of the current profile in millis
    uint32_t profile_start = 0;

    // Add an MCB motion TM packet to the binary TM buffer
    void AddMCBTM();

    // Set variables and TM buffer after a profile starts
    void NoteProfileStart();

    // Send a telemetry packet with EEPROM contents
    void SendMCBEEPROM();
    void SendRATSEEPROM();

    void statusMsgCheck(int repeat_secs);
    void sendTMstatusMsg();

    // Global variables used by RATS
    // Variables with initial values, that can be configured via telecommand
    uint16_t Set_sampleRateSecs = 60;
    uint8_t Set_dataProcMethod = 1;
    uint16_t Set_deployRevs = 10;
    uint16_t Set_deploySpeed = 1;
    uint16_t Set_retractRevs = 10;
    uint16_t Set_retractSpeed = 1;
    uint16_t Set_motorCurrentLimit = 1;
    uint16_t Set_motorTorqueLimit = 1;
    bool Set_motorHome = false;
    bool Set_motorReset = false;

    // Actions
    ActionFlag_t action_flags[NUM_ACTIONS] = {{0}}; // initialize all flags to false
};
#endif /* STRATORATS_H */
