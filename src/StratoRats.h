#ifndef STRATORATS_H
#define STRATORATS_H

#include <time.h>
//#include <map>
#include "StratoCore.h"
#include "RATSHardware.h"
#include "RATSConfigs.h"
#include "MCBComm.h"

// WARNING: DO NOT CHECK CODE INTO GIT WIH THIS OPTION ENABLED. 
//          MAKE SURE THIS OPTION IS FALSE FOR FLIGHT DEPLOYED FIRMWARE.
#define ZEPHYR_COMMS_ON_DEBUG_PORT true

// WARNING: DO NOT CHECK CODE INTO GIT WIH THIS OPTION ENABLED. 
//          MAKE SURE THIS OPTION IS FALSE FOR FLIGHT DEPLOYED FIRMWARE.
// Define this to disable some error checking and logging during development testing.
#define DISABLE_DEVEL_ERROR_CHECKING true

// Reporting period for status message generation, including TM transmission.
#define STATUS_MSG_PERIOD_SECS 300

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
#define LORA_MSG_COUNT  2

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

//LoRa Settings
#define FREQUENCY 868E6
#define BANDWIDTH 250E3
#define SF 9
#define RF_POWER 19
#define LORA_TM_TIMEOUT 600

// todo: perhaps more creative/useful enum here by mode with separate arrays?
enum ScheduleAction_t : uint8_t {
    NO_ACTION = NO_SCHEDULED_ACTION,
    SEND_IMR,

    RESEND_RA,
    RESEND_MOTION_COMMAND,
    RESEND_TM,
    RESEND_SAFETY,

    ACTION_START_TELEMETRY,
    ACTION_GPS_WAIT_MSG,
    ACTION_LORA_WAIT_MSG,
    ACTION_SEND_STATUS,
    ACTION_REEL_OUT,
    ACTION_REEL_IN,
    ACTION_IN_NO_LW,

    ACTION_MOTION_STOP,
    ACTION_MOTION_TIMEOUT,
    ACTION_SIM_LORA_MSG,

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
    
    void ManualFlight();

    void RATS_Shutdown();

    // LoRa
    void LoRaInit();
    void LoRaRX();

    // Flight states (each in own .cpp file)
    // when starting the state, call with restart_state = true
    // then call with restart_state = false until the function returns true meaning it's completed
    bool Flight_ManualMotion(bool restart_state);

    // Telcommand handler - returns ack/nak
    bool TCHandler(Telecommand_t telecommand);

    // Action handler for scheduled actions
    void ActionHandler(uint8_t action);

    // Safely check and clear action flags
    bool CheckAction(uint8_t action);

    // Correctly set an action flag
    void SetAction(uint8_t action);

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

    // Global variable to track flight_substate_map[inst_subst] during flight mode
    uint8_t flight_mode_substate = 0;

    // flags for MCB state tracking
    bool mcb_low_power = false;
    bool mcb_motion_ongoing = false;
    bool mcb_dock_ongoing = false;
    uint32_t max_profile_seconds = 0;
    bool mcb_reeling_in = false;
    uint16_t mcb_tm_counter = 0;
    float reel_pos = 0.0;


    // array of error values for MCB motion fault
    uint16_t motion_fault[8] = {0};
    uint8_t MCB_TM_buffer[8192] = {0};
    uint16_t MCB_TM_buffer_idx = 0;

    // tracks the current type of motion
    MCBMotion_t mcb_motion = NO_MOTION;
    float deploy_length = 0.0f;
    float retract_length = 0.0f;

    // uint32_t start time of the current profile in millis
    uint32_t profile_start = 0;


    // Count incoming lora messages. If reset is set, the
    // count is set to 0. 
    // Return the current count.
    uint32_t lora_count_check(bool reset=false);
    uint32_t lora_count = 0;

    // Start any type of MCB motion
    bool StartMCBMotion();

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

    // Actions
    ActionFlag_t action_flags[NUM_ACTIONS] = {{0}}; // initialize all flags to false

    //Variables for LoRa TMs and Status strings
    bool Send_LoRa_TM = true;
    bool Send_LoRa_status = true;
    char LoRa_RX_buffer[256] = {0};
    char LoRa_PU_status[256] = {0};
    
    uint8_t LoRa_TM_buffer[8192] = {0};
    uint16_t LoRa_TM_buffer_idx = 0;
    uint16_t pu_tm_counter = 0;
    long LoRa_rx_time = 0;

};
#endif /* STRATORATS_H */
