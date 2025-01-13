#ifndef STRATORATS_H
#define STRATORATS_H

#include <time.h>
//#include <map>
#include "StratoCore.h"
#include "RATSHardware.h"
#include "RATSConfigs.h"
#include "MCBComm.h"
#include "ECULoRa.h"

// WARNING: DO NOT CHECK CODE INTO GIT WIH THIS OPTION ENABLED. 
//          MAKE SURE THIS OPTION IS FALSE FOR FLIGHT DEPLOYED FIRMWARE.
#define ZEPHYR_COMMS_ON_DEBUG_PORT true

// WARNING: DO NOT CHECK CODE INTO GIT WIH THIS OPTION ENABLED. 
//          MAKE SURE THIS OPTION IS FALSE FOR FLIGHT DEPLOYED FIRMWARE.
// Define this to disable some error checking and logging during development testing.
#define DISABLE_DEVEL_ERROR_CHECKING false

#define EXTRA_LOGGING false

// Reporting period for status message generation, including TM transmission.
#define RATS_REPORT_PERIOD_SECS 300

#if not ZEPHYR_COMMS_ON_DEBUG_PORT
#define ZEPHYR_SERIAL   Serial8
#else
// This allows for use of the OBD_Simulator with just the Teensy programming port, 
// by sharing it for both Zephyr and StratoCore log messages.
#define ZEPHYR_SERIAL   Serial
#endif

// Our instrument name
#define INSTRUMENT      RATS

// Number of LoRa messages to wait for before moving on
#define LORA_MSG_COUNT  3
// Seconds to wait for all LoRa messages to be received during warmup
#define LORA_WARMUP_MSG_TIMEOUT 60

#define ZEPHYR_SERIAL_BUFFER_SIZE 4096
#define MCB_SERIAL_BUFFER_SIZE    4096

// Buffers for msg reception and transmission to/from Zephyr. Should be large enough
// to hold a complete TM, some of which which will contain the measurement data.
#define ZEPHYR_SERIAL_BUFFER_SIZE 4096
// Number of loops before a flag becomes stale and is reset
#define FLAG_STALE      3
//
#define MCB_RESEND_TIMEOUT      10

// The size of a buffer used for binary transfers between RATS and MCB.
#define MCB_BINARY_BUFFER_SIZE MAX_MCB_BINARY
#define HEARTBEAT_LED_PIN	3

#define ZEPHYR_RESEND_TIMEOUT   60

    // Actions
enum ScheduleAction_t : uint8_t {
    NO_ACTION = NO_SCHEDULED_ACTION,
    SEND_IMR,

    RESEND_RA,
    RESEND_MOTION_COMMAND,
    RESEND_TM,
    RESEND_SAFETY,

    ACTION_START_TELEMETRY,
    ACTION_GPS_WAIT_MSG,
    ACTION_LORA_COUNT_MSGS,
    ACTION_SEND_STATUS,
    ACTION_REEL_OUT,
    ACTION_REEL_IN,
    ACTION_IN_NO_LW,

    ACTION_MOTION_STOP,
    ACTION_MOTION_TIMEOUT,

    NUM_ACTIONS
};

enum MCBMotion_t : uint8_t {
    NO_MOTION,
    MOTION_REEL_IN,
    MOTION_REEL_OUT,
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

    // Initialize the flight mode status values
    void FlightModeInit();

    // Mode functions (implemented in unique source files)
    void StandbyMode();
    void FlightMode();
    void LowPowerMode();
    void SafetyMode();
    void EndOfFlightMode();
    
    void RATS_Shutdown();

    // The FLIGHT mode substates.
    // Moved from Flight.cpp to here so that TCHandler() can use them to vet TCs.
    enum FLStates_t : uint8_t {
        FL_ENTRY = MODE_ENTRY,
        FL_GPS_WAIT,
        FL_WARMUP,
        FL_MEASURE,
        FL_SEND_TELEMETRY,
        FL_REEL,
        FL_ERROR = MODE_ERROR,
        FL_SHUTDOWN = MODE_SHUTDOWN,
        FL_EXIT = MODE_EXIT
    };

    // Flight states (each in own .cpp file)
    // When starting a state, initiate it by calling with state = true,
    // then call with restart==false until the function returns true,
    // a which indicates that the state has completed.
    // HOWEVER, THE STATE FUNCTION CAN CHANGE inst_substate (E.g. TO FL_ERROR)
    // AND RETURN FALSE, WHICH WILL FORCE THE FLIGHT MODE TO THE NEW STATE.
    // TODO: This seems hard to follow. Perhaps the flight state should
    // return a specific status code, and let Flight() determine what
    // to do. As it is, the complete state machine logic is split up
    // among multiple files (via inst_state), making it very hard to 
    // see the logic in one place.
    //
    // A sub-sub state machine to manage reel operations.
    bool Flight_Reel(bool restart);
    // A sub-sub state machine to manage the warmup operations.
    bool Flight_Warmup(bool restart);

    // Telcommand handler - returns ack/nak
    bool TCHandler(Telecommand_t telecommand);

    // *** Action processing ***
    // Action handler for scheduled actions
    void ActionHandler(uint8_t action);
    // Safely check and clear action flags
    bool CheckAction(uint8_t action);
    // Correctly set an action flag
    void SetAction(uint8_t action);
    // Monitor the action flags and clear old ones
    void WatchFlags();
    // Used by StratoRATS action logic (I wonder why this logic is not in the StratoCore class?)
    ActionFlag_t action_flags[NUM_ACTIONS] = {{0}}; // initialize all flags to false


    // Global variable to track flight_substate_map[inst_subst] during flight mode
    uint8_t flight_mode_substate = 0;

    // *** LoRa support ***
    // Call this during every loop to check for incoming LoRa messages.
    void LoRaRX();
    // The most recently received LoRa message.
    ECULoRaMsg_t lora_msg;
    // The total number of LoRa messages received since the application started.
    uint32_t total_lora_count = 0;
    // A temporary counter to track the number of LoRa messages received during warmup.
    uint32_t lora_count = 0;
    // Count incoming lora messages. If reset is set, the
    // count is set to 0. 
    // Return the current count.
    // Uses the global variable lora_count.
    uint32_t lora_count_check(bool reset=false);

    // *** Warmup state machine ***
    // LoRa message timeout counter
    uint32_t LoRaMsg_timer_start = 0;

    // *** Reel motion variables ***
    // Set in TCHandler(), used in Flight_Reel.
    float deploy_length = 0.0f;
    // Set in TCHandler(), used in Flight_Reel.
    float retract_length = 0.0f;
    // uint32_t start time of the current reel motion, in millis
    uint32_t reel_motion_start = 0;

    // *** MCB support ***
    // Handle ASCII messages from the MCB (in MCBRouter.cpp)
    void HandleMCBASCII();
    // Handle ACK messages from the MCB (in MCBRouter.cpp)
    void HandleMCBAck();
    // Handle binary messages from the MCB (in MCBRouter.cpp)
    void HandleMCBBin();
    // Handle string messages from the MCB (in MCBRouter.cpp)
    void HandleMCBString();
    // Start any type of MCB motion
    bool StartMCBMotion();
    // Set variables and initialize the TM for MCB binary data collection.
    // AddMCBTM() will add append MCB binary data to the TM buffer.
    void InitMCBMotionTracking();
    // Add the current MCB motion binary data to the TM buffer.
    // IF WE ARE IN REAL-TIME MODE, THE TM PACKET WILL BE SENT IMMEDIATELY.
    void AddMCBTM();
    // Send a TM with a StateMessage1 message, and the aggregated MCB binary info.
    // All of the aggregated MCB binary data are included in the TM packet.
    void SendMCBTM(StateFlag_t state_flag, const char * message);
    bool mcb_low_power = false;
    // Set when a reel motion is initiated, cleared when the motion is complete.
    bool mcb_motion_ongoing = false;
    // The maximum time allowed for a reel motion to complete.
    uint32_t max_reel_seconds = 0;
    // TODO: Will be used in the safety mode, which has been implemented yet
    bool mcb_reeling_in = false;
    // The buffer used to receive binary data from each incoming MCB message.
    // The data will be copied to the MCB_TM_buffer for TM transmission.
    uint8_t binary_mcb[MCB_BINARY_BUFFER_SIZE];
    // Counts the number of MCB binary messages received during a motion
    uint16_t mcb_tm_counter = 0;
    // The current reel position in revs, extracted from the MCB binary message.
    float reel_pos = 0.0;
    // array of error values for MCB motion fault
    uint16_t motion_fault[8] = {0};
    // A buffer to collect MCB binary data for the MCB TM.
    uint8_t MCB_TM_buffer[8192] = {0};
    // Next available index in the MCB TM buffer.
    uint16_t MCB_TM_buffer_idx = 0;
    // tracks the current type of motion
    MCBMotion_t mcb_motion = NO_MOTION;

    // *** Other TC handlers ***
    // Send a TM with MCB EEPROM contents
    void SendMCBEEPROM();
    // Send a TM with RATS EEPROM contents
    void SendRATSEEPROM();

    // *** ratsReport functions ***
    // Check if it's time for a ratsReport and send a TM if true
    void ratsReportCheck(int repeat_secs);
    // Send a TM with the ratsReport message
    void ratsReportTM();

};
#endif /* STRATORATS_H */
