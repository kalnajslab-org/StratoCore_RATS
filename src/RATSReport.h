#ifndef RATS_REPORT_H
#define RATS_REPORT_H
#include <stddef.h>
#include "ECUReport.h"
#define ETL_NO_STL
#define ETL_NO_INITIALIZER_LIST
#include "etl/array.h"
#include "etl/bit_stream.h"


#define RATS_REPORT_REV 1

#define RATS_REPORT_HEADER_SIZE_BITS (8 + 16 + 16 + 1 + 13)
#define RATS_REPORT_HEADER_SIZE_BYTES 7

// The RATS report serialized bytes are collected here. But note that each item
// will need to be added to the TM buffer individually.
template <size_t N_ECU_REPORTS>
class RATSReportTM {
public:
    // A byte array to hold the serialized RATS report header.
    typedef etl::array<uint8_t, RATS_REPORT_HEADER_SIZE_BYTES> RATSReportHeaderBytes_t;

    struct RATSReportHeader_t {
        uint8_t version :                 4; // The version of the RATS report header.
        uint8_t header_size_bytes :       8; // The size of the RATS report header in bytes.
        uint16_t num_ecu_records :       16; // The number of ECU records in the report.
        uint16_t ecu_record_size_bytes : 16;
        uint8_t ecu_pwr_on :              1; // If the ECU is powered on.
        uint16_t v56 :                   13; // The 56V voltage in 0.01V units
        uint16_t cpu_temp :              11; // (CPU temperature+100)*10
        uint8_t lora_rssi :              11; // (Lora RSSI in dBm - 2047)*10
        uint8_t lora_snr :               10; // (Lora SNR in dB - 512)*10
    };

    // Constructor to initialize the RATS report header with the number of ECU reports.
    explicit RATSReportTM() {
        setReportHeader(0, ECU_REPORT_SIZE_BYTES, false, 0, 0, 0, 0);
    };

    void setReportHeader(uint16_t num_ecu_records, uint16_t ecu_record_size_bytes, 
                            bool ecu_pwr_on, uint16_t v56, uint16_t cpu_temp, uint8_t lora_rssi, uint8_t lora_snr) {
        _header.version = RATS_REPORT_REV;
        _header.header_size_bytes = RATS_REPORT_HEADER_SIZE_BYTES;
        _header.num_ecu_records = num_ecu_records;
        _header.ecu_record_size_bytes = ecu_record_size_bytes;
        _header.ecu_pwr_on = ecu_pwr_on;
        _header.v56 = v56;
        _header.cpu_temp = cpu_temp;
        _header.lora_rssi = lora_rssi;
        _header.lora_snr = lora_snr;
    };

    void addECUReport(const ECUReportBytes_t& ecu_report_bytes) {
        if (_header.num_ecu_records < N_ECU_REPORTS) {
            records[_header.num_ecu_records] = ecu_report_bytes;
            _header.num_ecu_records++;
        }
    };

    // Serialize the RATS report into the byte array.
    RATSReportHeaderBytes_t& serialize_header() {

        etl::span<uint8_t> data_span(_header_bytes.data(), _header_bytes.size());
        etl::bit_stream_writer writer(data_span, etl::endian::big);

        // Serialize the header
        writer.write_unchecked(_header.version, 4);
        writer.write_unchecked(_header.header_size_bytes, 8);
        writer.write_unchecked(_header.num_ecu_records, 16);
        writer.write_unchecked(_header.ecu_record_size_bytes, 16);
        writer.write_unchecked(_header.ecu_pwr_on, 1);
        writer.write_unchecked(_header.v56, 13);
        writer.write_unchecked(_header.cpu_temp, 11);
        writer.write_unchecked(_header.lora_rssi, 11);
        writer.write_unchecked(_header.lora_snr, 10);

        return _header_bytes;
    };

    // The RATSReportHeader header
    RATSReportHeader_t _header;

    // The storage for the serialized RATS report header
    RATSReportHeaderBytes_t _header_bytes;

    // The ECU report data. There may be zero records if the ECU was not powered on.
    ECUReportBytes_t records[1 + N_ECU_REPORTS];
};

#endif // RATS_REPORT_H
