#ifndef RATS_REPORT_H
#define RATS_REPORT_H
#include <stddef.h>
#include "ECUReport.h"
#include "RATSHardware.h"
#define ETL_NO_STL
#define ETL_NO_INITIALIZER_LIST
#include "etl/array.h"
#include "etl/bit_stream.h"

// Rounding up #define for CPP math
#define DIV_ROUND_UP(Numerator, Denominator) (((Numerator) + (Denominator) - 1) / (Denominator))

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
// 3. Call setHeader() to set the header values.
// 4. Call getReportBytes() to fetch the TM binary payload.
// 5. Call getReportLength() to get the TM binary payload length in bytes.
// 6. Call initReport() to reset the report for the next collection.
//
// The maximum number of ECU reports is specified as the template parameter N_ECU_REPORTS.
template <size_t N_ECU_REPORTS>
class RATSReport
{

protected:
// When modifying the RATSReportHeader_t structure, there are five things you must do:
// 1. Increment the RATS_REPORT_REV.
// 2. Modify or add fields to the RATSReportHeader_t structure as needed. The type of 
//    each field will be the next larger unsigned type that can hold the required number of bits.
//    For example, if you need 13 bits, use uint16_t. If you need 10 bits, use uint16_t as well.
//    If you need 1 bit, use uint8_t, etc.
// 3. Update the RATS_REPORT_HEADER_SIZE_BITS to the sum of the bitfield sizes in the structure.
//    You can use the copilot to create this sum by prompting: "sum of bitfield sizes in RATSReportHeader_t".
// 4. Update serializeHeader() to serialize the new fields in the correct order.
// 5. Update fillReportHeader() to set new fields with the scaled values.

#define RATS_REPORT_REV 1
    struct RATSReportHeader_t
    {
        uint8_t version :                 4; // The version of the RATS report header.
        uint8_t header_size_bytes :       8; // The size of the RATS report header in bytes.
        uint16_t num_ecu_records :       16; // The number of ECU records in the report.
        uint16_t ecu_record_size_bytes : 16;
        uint8_t ecu_pwr_on :              1; // If the ECU is powered on.
        uint16_t v56 :                   13; // The 56V voltage in 0.01V units
        uint16_t cpu_temp :              11; // (CPU temperature+100)*10
        uint16_t lora_rssi :             11; // (Lora RSSI in dBm-2047)*10
        uint16_t lora_snr :              10; // (Lora SNR in dB-512)*10
    };

#define RATS_REPORT_HEADER_SIZE_BITS (4 + 8 + 16 + 16 + 1 + 13 + 11 + 11 + 10)
#define RATS_REPORT_HEADER_SIZE_BYTES DIV_ROUND_UP(RATS_REPORT_HEADER_SIZE_BITS, 8)

    // Serialize the RATS report into the header bytes.
    // This should be called before sending the report.
    void serializHeader()
    {
        // The report header is serialized into the beginning of report bytes.
        etl::span<uint8_t> data_span(_report_bytes.data(), _report_bytes.size());
        etl::bit_stream_writer writer(data_span, etl::endian::big);

        // Serialize the header
        writer.write_unchecked(_report_header.version,                4);
        writer.write_unchecked(_report_header.header_size_bytes,      8);
        writer.write_unchecked(_report_header.num_ecu_records,       16);
        writer.write_unchecked(_report_header.ecu_record_size_bytes, 16);
        writer.write_unchecked(_report_header.ecu_pwr_on,             1);
        writer.write_unchecked(_report_header.v56,                   13);
        writer.write_unchecked(_report_header.cpu_temp,              11);
        writer.write_unchecked(_report_header.lora_rssi,             11);
        writer.write_unchecked(_report_header.lora_snr,              10);
    };

public:
    void fillReportHeader(uint8_t lora_rssi, uint8_t lora_snr)
    {
        _report_header.version = RATS_REPORT_REV;
        _report_header.header_size_bytes = RATS_REPORT_HEADER_SIZE_BYTES;
        // _report_header.num_ecu_records is set by addECUReport();
        _report_header.ecu_record_size_bytes = ECU_REPORT_SIZE_BYTES;
        _report_header.ecu_pwr_on = digitalRead(ECU_PWR_EN);
        _report_header.v56 = 1000*analogRead(V56_MON) * (3.3 / 1024.0) * (R8 + R9) / R8;
        _report_header.cpu_temp = (tempmonGetTemp() + 100) * 10;
        _report_header.lora_rssi = (lora_rssi - 2047) * 10;
        _report_header.lora_snr = (lora_snr - 512) * 10;
    };

    // Constructor to initialize the RATS report header with the number of ECU reports.
    explicit RATSReport()
    {
        initReport();
    };

    void addECUReport(const ECUReportBytes_t &ecu_report_bytes)
    {
        if (_report_header.num_ecu_records < N_ECU_REPORTS)
        {
            _ecu_reports[_report_header.num_ecu_records] = ecu_report_bytes;
            _report_header.num_ecu_records++;
        } else {
            // Log an error or handle the case where the maximum number of ECU reports is exceeded.
            // For now, we will just ignore the additional report.
        }
    };

    etl::array<uint8_t, RATS_REPORT_HEADER_SIZE_BYTES> &getReportBytes()
    {
        // The report bytes include the serialized header and the ECU reports.
        this->serializeHeader();

        // Append the ECU reports to the binary payload after the RATSReport header.
        for (size_t i = 0; i < _report_header.num_ecu_records; i++)
        {
            for (size_t j = 0; j < ECU_REPORT_SIZE_BYTES; j++)
            {
                _report_bytes[RATS_REPORT_HEADER_SIZE_BYTES + (i * ECU_REPORT_SIZE_BYTES) + j] = _ecu_reports[i][j];
            }
        }
        return _report_bytes;
    };

    // Get the length of the report TM binary payload in bytes.
    size_t getReportLength() const
    {
        return RATS_REPORT_HEADER_SIZE_BYTES + (_report_header.num_ecu_records * ECU_REPORT_SIZE_BYTES);
    }

    // Reset the report for the next collection.
    void initReport()
    {
        _report_header.num_ecu_records = 0;
        _report_bytes.fill(0);
    }

protected:
    // The RATSReport header. It is the non-serialized header; will be serialized
    // when getReportBytes() is called.
    RATSReportHeader_t _report_header;

    // The ECU report data is collected here. There may be zero records if the ECU was not powered on.
    ECUReportBytes_t _ecu_reports[N_ECU_REPORTS];

    // The storage for the complete RATS report TM binary payload.
    // The first bytes are the serialized RATS report header, followed by the ECU reports.
    etl::array<uint8_t, RATS_REPORT_HEADER_SIZE_BYTES + (N_ECU_REPORTS * ECU_REPORT_SIZE_BYTES)> _report_bytes;
};

#endif // RATS_REPORT_H
