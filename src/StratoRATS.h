#ifndef STRATORATS_H
#define STRATORATS_H

#include <time.h>
#include "StratoCore.h"
#include "RATSHardware.h"
#include "RATSConfigs.h"
#include "MCBComm.h"
#include "ECULoRa.h"
#include "ECUReport.h"
#include "RATSReport.h"
#include "etl/bit_stream.h"
#include "etl/array.h"

// Set this true to disable some error checking and logging during development testing.
#define DISABLE_DEVEL_ERROR_CHECKING false

#define EXTRA_LOGGING false

// RATSReport reporting period, when scheduled by ACTION_RATS_REPORT.
#define RATS_REPORT_PERIOD_SECS 600

// Send a RATSReport when NUM_ECU_REPORTS have been received.
// But if RATS_REPORT_PERIOD_SECS has elapsed, the report will be sent regardless.
#define NUM_ECU_REPORTS 175

// RATS_REPORT_MAX_BYTES is the maximum size of a RATS report in bytes. 
#define RATS_REPORT_MAX_BYTES (RATS_HEADER_SIZE_BYTES+(NUM_ECU_REPORTS+1)*ECU_REPORT_SIZE_BYTES)

// Verify that a RATS report will fit in the TM message buffer.
#if RATS_REPORT_MAX_BYTES > 8192
// TODO: Verify that this is the true limit for the TM binary payload. It's
// likely that the limit is actually includes all of the XML header and other
// TM overhead, so this may be unsafe.
#error RATS_REPORT_MAX_BYTES exceeds TM message buffer size
#endif

#ifndef LOG_ZEPHYR_COMMS_SHARED
#define ZEPHYR_SERIAL   Serial1
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
#define LORA_WARMUP_MSG_TIMEOUT 15

#define MCB_SERIAL_BUFFER_SIZE    4096

// Buffers for msg reception and transmission to/from Zephyr. Should be large enough
// to hold a complete TM, some of which which will contain the measurement data.
#define ZEPHYR_SERIAL_BUFFER_SIZE (2*8192)

// Number of loops before a flag becomes stale and is reset
#define FLAG_STALE      3
//
#define MCB_RESEND_TIMEOUT      10

// The size of a buffer used for binary transfers between RATS and MCB.
#define MCB_BINARY_BUFFER_SIZE MAX_MCB_BINARY
#define HEARTBEAT_LED_PIN	3

#define ZEPHYR_RESEND_TIMEOUT   60

// The number of instrument current measurements to average.
// Empirically determined to calculate an average about every
// 15 seconds while in flight mode.
#define INST_IMON_AVERAGE_COUNT 50

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
    ACTION_RATS_REPORT,
    ACTION_REEL_OUT,
    ACTION_REEL_IN,
    ACTION_IN_NO_LW,
    ACTION_LORA_TX_TEST,
    ACTION_MOTION_STOP,
    ACTION_MOTION_TIMEOUT,
    ACTION_MCB_INIT_MOTION,

    NUM_ACTIONS
};

enum MCBMotion_t : uint8_t {
    NO_MOTION,
    MOTION_REEL_IN,
    MOTION_REEL_OUT,
    MOTION_IN_NO_LW
};

enum WarmupStatus_t : uint8_t {
    WARMUP_NOT_STARTED,
    WARMUP_INPROCESS,
    WARMUP_FAILED,
    WARMUP_COMPLETE
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

    // Save a copy of inst_mode here, since it is private in StratoCore.
    // It gets set whenever one of the mode handlers is entered. 
    // It's silly inst_mode is private, since shared functions sometime want 
    // to know the current mode.
    InstMode_t my_inst_mode = MODE_STANDBY;

    // The FLIGHT mode substates.
    // Moved from Flight.cpp to here so that TCHandler() can use them to vet TCs.
    enum FLStates_t : uint8_t {
        FL_ENTRY = MODE_ENTRY,
        FL_GPS_WAIT,
        FL_WARMUP,
        FL_MEASURE,
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
    // Set to true to enable LoRa TX test mode
    bool lora_tx_test = false;

    // ECU control
    void ECUPowerControl(bool enable);
    bool IsECUPowerEnabled();

    // *** Warmup state machine ***
    // LoRa message timeout counter
    uint32_t LoRaMsg_timer_start = 0;
    // The warmup status
    WarmupStatus_t warmup_status = WARMUP_INPROCESS;
    // Number of warmup cycles
    uint8_t warmup_cycles = 0;

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

    // MCB motion tracking and TM functions. 
    // These are used to track the progress of a reel motion, and to build and send TMs 
    // with the motion data. The motion data is received from the MCB in binary messages, 
    // which are processed in HandleMCBBin() and added to the MCB_TM_buffer via AddMCBTM().
    //
    // If in real-time MCB reporting mode, the MCB binary messages are sent immediately as 
    // MCBREPORT TMs.
    //
    // If not in real-time MCB reporting mode, the MCB binary messages are 
    // collected in MCB_TM_buffer, and sent as a single MCBREPORT TM when a certain
    // condition is met. The condition could be the completion of the motion, a timeout, 
    // reception of an MCB error message, or something else.
    //
    // It seems like there is an opportunity for a MCM_TM_buffer to overflow,
    // because the value if MCM_TM_buffer_idx is never checked against the size of MCB_TM_buffer. 
    // However, in practice this might not be an issue, because the MCB binary messages are 
    // expected to be small and infrequent, and the MCB_TM_buffer is large (8192 bytes). 
    // We could add a check in AddMCBTM() to ensure that we don't 
    // write past the end of the buffer. We could also consider implementing some logic to 
    // handle the case where the buffer does fill up, such as sending a TM with the data 
    // collected so far and then clearing the buffer to make room for more data.
    //
    // Set variables and initialize the TM for MCB binary data collection.
    void InitMCBMotionTracking();
    // **If we are not in real-time mcb reporting mode, data from mcbComm.binary_rx.bin_buffer 
    // will be appended to MCB_TM_buffer.**
    // **If we are in real-time mcb reporting mode, data from mcbComm.binary_rx.bin_buffer 
    // will be sent immediately as a MCBREPORT TM.**
    void AddMCBTM();
    // Send an MCBREPORT TM with a StateMessage1 message, and the aggregated 
    // data in MCB_TM_buffer.
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

    // *** Other TC/TM handlers ***
    // Send a TM with MCB EEPROM contents
    void SendMCBEEPROM();
    // Send a TM with RATS EEPROM contents
    void SendRATSEEPROM();
    // Send a TM
    void SendTM(String details1, StateFlag_t state_flag1, 
        String details2, StateFlag_t state_flag2, 
        String details3, StateFlag_t state_flag3);
    // Send a RATSTEXT TM with text data
    void SendRATSTextTM(String text_data, StateFlag_t state_flag = FINE);

    // A running sum of the voltage for inst_imon.
    float a3_v_sum = 0.0;
    // The last average of the inst_imon calculated from a3_v_sum.
    float inst_imon_mA = 0.0;

    // *** RatsReports ***
    // Accumulate RATS reports for transmission
    void ratsReportAccumulate(ECUReportBytes_t& ecu_report_bytes);
    // Check if it's time for a ratsReport and send a TM if true.
    // If immediate is true, the report will be sent immediately.
    void ratsReportCheck(bool immediate);
    // Send a TM with the ratsReport message
    void SendRATSReportTM();
    // Time of last RATS report
    time_t last_rats_report = 0;
    // Build and manage the RATS data report here. ECUReports are accumulated
    // into this report until it is sent.
    RATSReport<NUM_ECU_REPORTS> rats_report;

    // The Teensy MAC address set during InstrumentSetup().
    uint8_t mac_address[6];

    // The RATS ID number, set during InstrumentSetup().
    uint16_t rats_id;

    // The paired ECU ID. It is stored in ratsConfigs.paired_ecu, but cached here
    // so that we don't keep reading it from the program flash. Any time that
    // ratsConfigs.paired_ecu_id is updated, this variable should also be updated.
    uint8_t paired_ecu;

    // Prepend the RATS message header to a string and send to ECU via LoRa.
    void LoRaTx(char* ecu_cmd, bool immediate=false);

    // Get the name of the current mode
    String getModeName(const uint8_t mode);

    // Send a tiny command to the MCB so that we will get an MCB message containing
    // the reel position. This will cause reel_pos to be initialized.
    void InitializeReelPosition();

};
#endif /* STRATORATS_H */
