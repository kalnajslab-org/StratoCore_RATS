# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

This is a PlatformIO project targeting the **Teensy 4.1** (Arduino framework). The MCU runs at 150 MHz (reduced from default to save power).

**Build:**

```bash
pio run -e rats
```

**Upload:**

```bash
pio run -e rats --target upload
```

**Build environments:**

- `rats` — standard build
- `rats_serial_shared` — shares USB serial port for both Zephyr comms and log output (useful for bench testing without full hardware)

**PlatformIO + Arduino IDE compatibility:** The repo supports both. For PlatformIO, a symlink is needed: `cd src; ln -s ../StratoCore_RATS.ino StratoCore_RATS.cpp`

**Pre/post build scripts (Python):**

- `exclude_files.py` — excludes ECUComm's `pro-rf-duplex.cpp` from the build
- `version_header.py` — generates `src/rats_version.h` from `git describe`
- `hex_save.py` — copies the compiled `.hex` to `src/`

## Architecture

### System Overview

RATS (Radiosonde Atmospheric Temperature Sensor) is a balloon instrument that:

1. Communicates with **Zephyr** (gondola OBC) via serial (`Serial1`) for telemetry/telecommands
2. Controls the **MCB** (Motor Control Board) via serial (`Serial3`) for reel deploy/retract
3. Communicates with the **ECU** (End-of-Cable Unit) via **LoRa** radio for sensor data

### Class Hierarchy

`StratoRATS` inherits from `StratoCore` (external library). `StratoCore` provides the mode state machine, scheduler, Zephyr serial framing (XML-based TM/TC), GPS time, and logging macros (`log_nominal`, `log_error`, `log_debug`).

The main class `StratoRATS` is declared in [src/StratoRATS.h](src/StratoRATS.h) and the mode implementations are split across files:

| File | Purpose |
| ---- | ------- |
| `StratoRATS.cpp` | Constructor, `InstrumentSetup()`, `InstrumentLoop()` |
| `Flight.cpp` | Flight mode state machine |
| `Flight_Warmup.cpp` | Warmup sub-state (LoRa handshake with ECU) |
| `Flight_Reel.cpp` | Reel motion sub-state |
| `Standby.cpp` | Standby mode |
| `LowPower.cpp` | Low power mode |
| `Safety.cpp` | Safety mode |
| `EndOfFlight.cpp` | End-of-flight mode |
| `TCHandler.cpp` | Telecommand dispatcher |
| `MCBRouter.cpp` | MCB message router (ASCII/ACK/binary/string) |

### Mode / State Machine

Flight mode substates (in order): `FL_ENTRY` → `FL_GPS_WAIT` → `FL_WARMUP` → `FL_MEASURE` ↔ `FL_REEL`

- `FL_WARMUP` waits for `LORA_MSG_COUNT` (3) LoRa messages from the ECU within `LORA_WARMUP_MSG_TIMEOUT` (15 s)
- `FL_MEASURE` accumulates ECU data; reel motion is triggered by `DEPLOYx`/`RETRACTx` telecommands
- `FL_REEL` manages MCB motor motion; returns to `FL_WARMUP` on completion

### Scheduling

`scheduler.AddAction(ACTION_XXX, seconds)` queues timed actions. `CheckAction()` and `SetAction()` manage the `action_flags[]` array. `WatchFlags()` clears stale flags after `FLAG_STALE` (3) loops.

**Caveats:**

- There is no way to cancel a scheduled action once it has been added.
- If an action is scheduled twice, the first scheduled instance remains pending alongside the new one, likely firing at an unexpected time with unintended consequences. Avoid calling `AddAction` for an action that may already be scheduled.

### Telemetry (TM) Format

TMs are XML-framed messages sent to Zephyr. Each TM has three `(message, StateFlag)` tuples where flags are `FINE`, `WARN`, or `CRIT`. Flag1 on Msg1 is the only flag monitored for alerts by the ground station.

Key TM types: `RATSREPORT` (primary data), `RATSTCACK` (TC acknowledgement), `MCBREPORT` (reel motion data), `RATSTEXT`, `RATSEEPROM`, `MCBEEPROM`.

### RATSReport Binary Payload

`RATSReport<N>` (in [src/RATSReport.h](src/RATSReport.h)) is a bit-packed template class that accumulates up to `NUM_ECU_REPORTS` (175) ECU observations. Reports are sent every `RATS_REPORT_PERIOD_SECS` (600 s) or after 175 ECU records, whichever comes first. The header is bit-packed using the ETL `bit_stream_writer`.

**When modifying `RATSReportHeader_t`:** increment `RATS_REPORT_REV`, update `RATS_REPORT_HEADER_SIZE_BITS`, update `serializeHeader()`, `fillReportHeader()`, and `print()` — all in lockstep.

### EEPROM Configuration

`RATSConfigs` (extends `TeensyEEPROM`) stores persistent config. To add a config value: add a public `EEPROMData<T>` member, set its backup value in the constructor, and register it in `RegisterAll()` — maintaining order in all three locations. Increment `CONFIG_VERSION` to force an EEPROM update on next boot.

### ECU LoRa Communication

RATS operates as a LoRa **follower** (`ECUCOMMFOLLOWER` build flag); the ECU is the leader. Commands to the ECU are sent as JSON payloads via `sendEcuJson()` in `TCHandler.cpp`. Incoming ECU reports are accumulated into `rats_report` via `ratsReportAccumulate()`.

### MCB Communication

The MCB (Motor Control Board) controls two motors: **RL** (Reel) and **LW** (Level Wind). Communication is via `MCBComm` on `Serial3`. Message types are ASCII, ACK, binary (`MCB_MOTION_TM`), and string (errors).

When a `MCB_MOTION_FAULT` is received, `RX_Motion_Fault()` populates `motion_fault[8]` with the following fields (logged as hex):

| Index | Field | Description |
| ----- | ----- | ----------- |
| `[0]` | `rl_status_lo` | Reel controller status register (low word) |
| `[1]` | `rl_status_hi` | Reel controller status register (high word) |
| `[2]` | `rl_detailed_err` | Reel controller detailed error code |
| `[3]` | `rl_motion_err` | Reel motion error code |
| `[4]` | `lw_status_lo` | Level Wind controller status register (low word) |
| `[5]` | `lw_status_hi` | Level Wind controller status register (high word) |
| `[6]` | `lw_detailed_err` | Level Wind controller detailed error code |
| `[7]` | `lw_motion_err` | Level Wind motion error code |

The bit-level meanings of the status/error registers are defined by the MCB motor controller hardware, not in this repo.

### Hardware Pin Definitions

All hardware pin assignments are in [src/RATSHardware.h](src/RATSHardware.h). LoRa uses SPI1 with a dedicated CS/RST/INT set of pins.

## Code Style

- K&R brace style: opening brace on the same line, closing brace on its own line.
- Always use braces for conditional/loop bodies, even single-line ones.
