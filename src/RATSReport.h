#ifndef RATS_REPORT_H
#define RATS_REPORT_H

#include <stddef.h>
#include "StratoGroundPort.h"
#include "ECUReport.h"
#include "RATSHardware.h"
#define ETL_NO_STL
#define ETL_NO_INITIALIZER_LIST
#include "etl/array.h"
#include "etl/bit_stream.h"

#ifndef DIV_ROUND_UP
// Rounding up #define for CPP math
#define DIV_ROUND_UP(Numerator, Denominator) (((Numerator) + (Denominator) - 1) / (Denominator))
#endif

// Build the binary payload for a RATSReport Telemetry message.
//
// The ECUReports are collected. When getReportBytes() is called, the RATSReportHeader
// is serialized and the ECUReports are added to the payload. The payload is then ready
// to be sent in a TM.
//
// Usage:
// 1. Create an instance of RATSReport with the max number of ECU reports.
// 2. Iterate as needed:
//    Call addECUReport() to add ECU reports.
// 3. Call fillReportHeader() to set the header values.
// 4. Call getReportBytes() to fetch the TM binary payload.
// 5. Call initReport() to reset the report for the next collection.
//
// The maximum number of ECU reports is specified as the template parameter N_ECU_REPORTS.
template <size_t N_ECU_REPORTS>
class RATSReport
{

protected:
    // When modifying the RATSReportHeader_t structure, there are five things you must do:
    // 1. Increment the RATS_REPORT_REV.
    // 2. Modify or add fields to the RATSReportHeader_t structure as needed. 
    // 3. Update the RATS_REPORT_HEADER_SIZE_BITS to the sum of the bitfield sizes in the structure.
    // 4. Update serializeHeader() to serialize the new fields in the correct order.
    // 5. Update fillReportHeader() to set new fields with the scaled values.
    // 6. Update RATSReportPrint() to print the new fields (scaled).

    // The RATS report header revision. Increment this whenever the header structure is modified.
#define RATS_REPORT_REV 4

    // The RATS report header structure.
    // The type of each field will be the next larger unsigned type that can hold the required number of bits.
    // For example, if you need 13 bits, use uint16_t. If you need 10 bits, use uint16_t as well.
    // If you need 1 bit, use uint8_t, etc.
    struct RATSReportHeader_t
    {
        uint8_t version :           4;       // The version of the RATS report header.
        uint16_t rats_id :         16;       // RATS unique identifier
        uint32_t epoch :           32;       // Epoch time in seconds when the report is generated.
        uint8_t paired_ecu :        8;       // Paired ECU ID
        uint8_t header_size_bytes : 8;       // The size of the RATS report header in bytes.
        uint16_t num_ecu_records : 10;       // The number of ECU records in the report.
        uint16_t ecu_size_bytes :   9;       // The size of each ECU record in bytes.
        uint8_t ecu_pwr_on :        1;       // If the ECU is powered on.
        uint16_t v56 :             13;       // (56V voltage)*100 (0-8191 : 0 to -81.9V)
        uint16_t cpu_temp :        11;       // (CPU temperature + 100)*10 (0-2047 : -100.0C to +104.7C)
        uint16_t lora_rssi :       10;       // (Lora RSSI + 100)*10 (0-1023 : -100.0dBm to +10.2dBm)
        uint16_t lora_snr :        10;       // (Lora SNR + 70)*10 (0-1023 : -70.0dB to +32.3dB)
        uint16_t inst_imon :       11;       // (Inst IMon)*10 (0-2047 : 0.0mA to 204.7mA)
        int32_t  gps_lat :         32;       // GPS Latitude*1e6 (degrees*1e6)
        int32_t  gps_lon :         32;       // GPS Longitude*1e6 (degrees*1e6)
        uint16_t gps_alt:          16;       // GPS Altitude (meters)
        uint16_t reel_revs:        16;       // (-Reel revolutions+100)*100 (0-65535 : -100.00 to 555.35 revolutions)     
    };

//    You can use the copilot to create this sum by prompting: "sum of bitfield sizes in RATSReportHeader_t".
#define RATS_REPORT_HEADER_SIZE_BITS (4 + 16 + 32 + 8 + 8 + 10 + 9 + 1 + 13 + 11 + 10 + 10 + 11 + 32 + 32 + 16 + 16)
#define RATS_REPORT_HEADER_SIZE_BYTES DIV_ROUND_UP(RATS_REPORT_HEADER_SIZE_BITS, 8)

    // Serialize the RATS report into the header bytes.
    // This should be called before sending the report.
    void serializeHeader()
    {
        // *** Modify this function whenever the RATSReportHeader_t struct is modified ***

        // The report header is serialized into the beginning of report bytes.
        etl::span<uint8_t> data_span(_report_bytes.data(), _report_bytes.size());
        etl::bit_stream_writer writer(data_span, etl::endian::big);

        // Serialize the header
        writer.write_unchecked(_header.version, 4);
        writer.write_unchecked(_header.rats_id, 16);
        writer.write_unchecked(_header.epoch, 32);
        writer.write_unchecked(_header.paired_ecu, 8);
        writer.write_unchecked(_header.header_size_bytes, 8);
        writer.write_unchecked(_header.num_ecu_records, 10);
        writer.write_unchecked(_header.ecu_size_bytes, 9);
        writer.write_unchecked(_header.ecu_pwr_on, 1);
        writer.write_unchecked(_header.v56, 13);
        writer.write_unchecked(_header.cpu_temp, 11);
        writer.write_unchecked(_header.lora_rssi, 10);
        writer.write_unchecked(_header.lora_snr, 10);
        writer.write_unchecked(_header.inst_imon, 11);
        writer.write_unchecked(_header.gps_lat, 32);
        writer.write_unchecked(_header.gps_lon, 32);
        writer.write_unchecked(_header.gps_alt, 16);
        writer.write_unchecked(_header.reel_revs, 16);
    };

public:
    void fillReportHeader(double lora_rssi, double lora_snr, double inst_imon_mA, uint16_t rats_id, uint8_t paired_ecu, float zephyr_lat, float zephyr_lon, float zephyr_alt, float reel_revs)
    {
        // *** Modify this function whenever the RATSReportHeader_t struct is modified ***

        _header.rats_id = rats_id;
        _header.epoch = (uint32_t)time(nullptr);
        _header.paired_ecu = paired_ecu;
        _header.ecu_pwr_on = digitalRead(ECU_PWR_EN);
        _header.v56 = 1000 * analogRead(V56_MON) * (3.3 / 1024.0) * (R8 + R9) / R8;
        _header.cpu_temp = (tempmonGetTemp() + 100.0) * 10;
        _header.lora_rssi = (lora_rssi + 100.0) * 10;
        _header.lora_snr = (lora_snr + 70) * 10;
        _header.inst_imon = (inst_imon_mA) * 10;
        _header.gps_lat = (int32_t)(zephyr_lat * 1e6);
        _header.gps_lon = (int32_t)(zephyr_lon * 1e6);
        _header.gps_alt = (uint16_t)(zephyr_alt);
        _header.reel_revs = (uint16_t)((-reel_revs + 100.0) * 100);
    };

    void print(bool print_bin)
    {
        // *** Modify this function whenever the RATSReportHeader_t struct is modified ***

        // For debugging use, print the RATS report header to SerialUSB.
        // If print_bin is true, print the binary representation of the header fields.

        SerialUSB.println("RATS Report:");

        // Header version
        SerialUSB.print("version: ");
        if (print_bin)
            binPrint(_header.version, 4);
        SerialUSB.print(String(_header.version));
        SerialUSB.println();

            // RATS ID
        SerialUSB.print("rats_id: ");
        if (print_bin)
            binPrint(_header.rats_id, 16);
        SerialUSB.print(String(_header.rats_id));
        SerialUSB.println();

        // Epoch time
        SerialUSB.print("epoch: ");
        if (print_bin) 
            binPrint(_header.epoch, 32);
        SerialUSB.print(String(_header.epoch) + "s");
        SerialUSB.println();

        // Paired ECU ID
        SerialUSB.print("paired_ecu: ");
        if (print_bin)
            binPrint(_header.paired_ecu, 8);
        SerialUSB.print(String(_header.paired_ecu));
        SerialUSB.println();

        // Header size in bytes
        SerialUSB.print("header_size_bytes: ");
        if (print_bin)
            binPrint(_header.header_size_bytes, 8);
        SerialUSB.print(String(_header.header_size_bytes));
        SerialUSB.println();

        // Number of ECU records
        SerialUSB.print("num_ecu_records: ");
        if (print_bin)
            binPrint(_header.num_ecu_records, 10);
        SerialUSB.print(String(_header.num_ecu_records));
        SerialUSB.println();

        // ECU record size in bytes
        SerialUSB.print("ecu_size_bytes: ");
        if (print_bin)
            binPrint(_header.ecu_size_bytes, 9);
        SerialUSB.print(String(_header.ecu_size_bytes));
        SerialUSB.println();

        // ECU power on
        SerialUSB.print("ecu_pwr_on: ");
        if (print_bin)
            binPrint(_header.ecu_pwr_on, 1);
        SerialUSB.print(String(_header.ecu_pwr_on));
        SerialUSB.println();

        // 56V voltage
        SerialUSB.print("v56: ");
        if (print_bin)
            binPrint(_header.v56, 13);
        SerialUSB.print((String(_header.v56 * 0.01) + "V"));
        SerialUSB.println();

        // CPU temperature
        SerialUSB.print("cpu_temp: ");
        if (print_bin)
            binPrint(_header.cpu_temp, 11);
        SerialUSB.print(String(_header.cpu_temp / 10.0 - 100.0) + "C");
        SerialUSB.println();

        // LoRa RSSI
        SerialUSB.print("lora_rssi: ");
        if (print_bin)
            binPrint(_header.lora_rssi, 10);
        SerialUSB.print(String(_header.lora_rssi / 10.0 - 100.0) + "dBm");
        SerialUSB.println();

        // LoRa SNR
        SerialUSB.print("lora_snr: ");
        if (print_bin)
            binPrint(_header.lora_snr, 10);
        SerialUSB.print(String(_header.lora_snr / 10.0 - 70.0) + "dB");
        SerialUSB.println();

        // Inst IMon
        SerialUSB.print("inst_imon: ");
        if (print_bin)
            binPrint(_header.inst_imon, 11);
        SerialUSB.print(String(_header.inst_imon / 10.0) + "mA");
        SerialUSB.println();

        // GPS Latitude
        SerialUSB.print("gps_lat: ");
        if (print_bin)            binPrint(_header.gps_lat, 32);
        SerialUSB.print(String(_header.gps_lat / 1e6) + "°");
        SerialUSB.println();

        // GPS Longitude
        SerialUSB.print("gps_lon: ");
        if (print_bin)            binPrint(_header.gps_lon, 32);
        SerialUSB.print(String(_header.gps_lon / 1e6) + "°");
        SerialUSB.println();

        // GPS Altitude
        SerialUSB.print("gps_alt: ");
        if (print_bin)            binPrint(_header.gps_alt, 16);
        SerialUSB.print(String(_header.gps_alt) + "m");
        SerialUSB.println();

        // Reel revolutions
        SerialUSB.print("reel_revs: ");
        if (print_bin)            binPrint(_header.reel_revs, 16);
        SerialUSB.print(String((_header.reel_revs / 100.0) - 100.0) + " revs");
        SerialUSB.println();
    };

    // Constructor to initialize the RATS report header with the number of ECU reports.
    explicit RATSReport()
    {
        initReport(0, 0);
    };

    void addECUReport(const ECUReportBytes_t &ecu_report_bytes)
    {
        if (_header.num_ecu_records < N_ECU_REPORTS)
        {
            _ecu_reports[_header.num_ecu_records] = ecu_report_bytes;
            _header.num_ecu_records++;
        }
        else
        {
            log_error("RATS report buffer full");
        }
    };

    auto& getReportBytes(uint& used_size)
    {
        // The report bytes include the serialized header and the ECU reports.
        this->serializeHeader();

        // Append the ECU reports to the binary payload after the RATSReport header.
        for (size_t i = 0; i < _header.num_ecu_records; i++)
        {
            for (size_t j = 0; j < ECU_DATA_REPORT_SIZE_BYTES; j++)
            {
                _report_bytes[RATS_REPORT_HEADER_SIZE_BYTES + (i * ECU_DATA_REPORT_SIZE_BYTES) + j] = _ecu_reports[i][j];
            }
        }
        used_size = RATS_REPORT_HEADER_SIZE_BYTES + (_header.num_ecu_records * ECU_DATA_REPORT_SIZE_BYTES);
        return _report_bytes;
    };

    // Get the number of ECU records in the report.
    int numECUrecords() const
    {
        return _header.num_ecu_records;
    }

    // Reset the report for the next collection.
    void initReport(uint16_t rats_id, uint8_t paired_ecu)
    {
        _header.rats_id = rats_id;
        _header.paired_ecu = paired_ecu;
        _header.version = RATS_REPORT_REV;
        _header.header_size_bytes = RATS_REPORT_HEADER_SIZE_BYTES;
        _header.num_ecu_records = 0;
        _header.ecu_size_bytes = ECU_DATA_REPORT_SIZE_BYTES;
        _report_bytes.fill(0);
    }

protected:
    void binPrint(uint32_t binValue, uint8_t nbits)
    {
        for (int i = nbits - 1; i >= 0; i--)
        {
            SerialUSB.print((binValue & (1 << i)) ? "1" : "0");
        }
        SerialUSB.print(" ");
    };

    // The RATSReport header. It is the non-serialized header; will be serialized
    // when getReportBytes() is called.
    RATSReportHeader_t _header;

    // The ECU report data is collected here. There may be zero records if the ECU was not powered on.
    // This will always be sized to hold the max number of ECU reports, but only the first num_ecu_records will be valid.
    ECUReportBytes_t _ecu_reports[N_ECU_REPORTS];

    // The storage for the complete RATS report TM binary payload.
    // The first bytes are the serialized RATS report header, followed by the ECU reports.
    etl::array<uint8_t, RATS_REPORT_HEADER_SIZE_BYTES + (N_ECU_REPORTS * ECU_DATA_REPORT_SIZE_BYTES)> _report_bytes;
};

#endif // RATS_REPORT_H
