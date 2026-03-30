#ifndef SPARKFUN_UBLOX_GNSS_ARDUINO_LIBRARY_H
#define SPARKFUN_UBLOX_GNSS_ARDUINO_LIBRARY_H

// =============================================================================
// SparkFun_u-blox_GNSS_Arduino_Library.h — Sim stub
//
// Mirrors the full public API of SparkFun u-blox GNSS Arduino Library v2/v3
// so firmware code compiles unchanged under SIM. No UART/I2C bus exists in
// sim; getPVT() and the individual getters pull lat/lon from
// ArduinoCompat::g_gpsLat / g_gpsLon (degrees, WGS-84, set by the
// simulator). All configuration, reset, and protocol methods are no-ops.
//
// Units returned match the real library:
//   getLatitude() / getLongitude() → degrees × 10⁷  (int32_t)
//   getAltitude() / getAltitudeMSL() → mm            (int32_t)
//   getGroundSpeed()                 → mm/s           (int32_t)
//   getHeading()                     → degrees × 10⁵ (int32_t)
//   getHorizontalAccEst()            → mm             (uint32_t)
//   getPDOP()                        → × 0.01         (uint16_t)
// =============================================================================

#include <cstdint>
#include "inocompat.h"

using TwoWire = _WireClass;
using Stream  = HardwareSerial;

// ---------------------------------------------------------------------------
// defaultMaxWait matches the real library default (ms)
// ---------------------------------------------------------------------------
#define defaultMaxWait 1100

// ---------------------------------------------------------------------------
// Communication port IDs
// ---------------------------------------------------------------------------
const uint8_t COM_PORT_I2C   = 0;
const uint8_t COM_PORT_UART1 = 1;
const uint8_t COM_PORT_UART2 = 2;
const uint8_t COM_PORT_USB   = 3;
const uint8_t COM_PORT_SPI   = 4;

// ---------------------------------------------------------------------------
// Port protocol type bitmask
// ---------------------------------------------------------------------------
const uint8_t COM_TYPE_UBX   = (1 << 0);
const uint8_t COM_TYPE_NMEA  = (1 << 1);
const uint8_t COM_TYPE_RTCM3 = (1 << 5);
const uint8_t COM_TYPE_SPARTN= (1 << 6);

// ---------------------------------------------------------------------------
// UBX sync chars
// ---------------------------------------------------------------------------
const uint8_t UBX_SYNCH_1 = 0xB5;
const uint8_t UBX_SYNCH_2 = 0x62;

// ---------------------------------------------------------------------------
// UBX class IDs
// ---------------------------------------------------------------------------
const uint8_t UBX_CLASS_NAV  = 0x01;
const uint8_t UBX_CLASS_RXM  = 0x02;
const uint8_t UBX_CLASS_INF  = 0x04;
const uint8_t UBX_CLASS_ACK  = 0x05;
const uint8_t UBX_CLASS_CFG  = 0x06;
const uint8_t UBX_CLASS_UPD  = 0x09;
const uint8_t UBX_CLASS_MON  = 0x0A;
const uint8_t UBX_CLASS_TIM  = 0x0D;
const uint8_t UBX_CLASS_NMEA = 0xF0;
const uint8_t UBX_CLASS_PUBX = 0xF1;
const uint8_t UBX_RTCM_MSB   = 0xF5;

// ---------------------------------------------------------------------------
// NAV message IDs
// ---------------------------------------------------------------------------
const uint8_t UBX_NAV_POSECEF   = 0x01;
const uint8_t UBX_NAV_POSLLH    = 0x02;
const uint8_t UBX_NAV_STATUS    = 0x03;
const uint8_t UBX_NAV_DOP       = 0x04;
const uint8_t UBX_NAV_ATT       = 0x05;
const uint8_t UBX_NAV_PVT       = 0x07;
const uint8_t UBX_NAV_ODO       = 0x09;
const uint8_t UBX_NAV_VELECEF   = 0x11;
const uint8_t UBX_NAV_VELNED    = 0x12;
const uint8_t UBX_NAV_HPPOSECEF = 0x13;
const uint8_t UBX_NAV_HPPOSLLH  = 0x14;
const uint8_t UBX_NAV_TIMEGPS   = 0x20;
const uint8_t UBX_NAV_TIMEUTC   = 0x21;
const uint8_t UBX_NAV_CLOCK     = 0x22;
const uint8_t UBX_NAV_SVIN      = 0x3B;
const uint8_t UBX_NAV_GEOFENCE  = 0x39;
const uint8_t UBX_NAV_SAT       = 0x35;
const uint8_t UBX_NAV_SIG       = 0x43;

// ---------------------------------------------------------------------------
// CFG message IDs
// ---------------------------------------------------------------------------
const uint8_t UBX_CFG_MSG     = 0x01;
const uint8_t UBX_CFG_INF     = 0x02;
const uint8_t UBX_CFG_RST     = 0x04;
const uint8_t UBX_CFG_RATE    = 0x08;
const uint8_t UBX_CFG_CFG     = 0x09;
const uint8_t UBX_CFG_RXM     = 0x11;
const uint8_t UBX_CFG_ANT     = 0x13;
const uint8_t UBX_CFG_NAV5    = 0x24;
const uint8_t UBX_CFG_TP5     = 0x31;
const uint8_t UBX_CFG_GNSS    = 0x3E;
const uint8_t UBX_CFG_TMODE3  = 0x71;
const uint8_t UBX_CFG_VALGET  = 0x8B;
const uint8_t UBX_CFG_VALSET  = 0x8A;
const uint8_t UBX_CFG_VALDEL  = 0x8C;

// ---------------------------------------------------------------------------
// NMEA message IDs
// ---------------------------------------------------------------------------
const uint8_t UBX_NMEA_MSB = 0xF0;
const uint8_t UBX_NMEA_GGA = 0x00;
const uint8_t UBX_NMEA_GLL = 0x01;
const uint8_t UBX_NMEA_GSA = 0x02;
const uint8_t UBX_NMEA_GSV = 0x03;
const uint8_t UBX_NMEA_RMC = 0x04;
const uint8_t UBX_NMEA_VTG = 0x05;
const uint8_t UBX_NMEA_GRS = 0x06;
const uint8_t UBX_NMEA_GST = 0x07;
const uint8_t UBX_NMEA_ZDA = 0x08;
const uint8_t UBX_NMEA_GBS = 0x09;
const uint8_t UBX_NMEA_DTM = 0x0A;
const uint8_t UBX_NMEA_GNS = 0x0D;
const uint8_t UBX_NMEA_VLW = 0x0F;

// ---------------------------------------------------------------------------
// Buffer / size constants
// ---------------------------------------------------------------------------
#define MAX_PAYLOAD_SIZE              276
#define SFE_UBLOX_SPI_BUFFER_SIZE     128
#define SFE_UBLOX_MAX_NMEA_BYTE_COUNT  88

// ---------------------------------------------------------------------------
// Configuration sub-section masks (saveConfigSelective)
// ---------------------------------------------------------------------------
const uint32_t VAL_CFG_SUBSEC_IOPORT  = 0x00000001;
const uint32_t VAL_CFG_SUBSEC_MSGCONF = 0x00000002;
const uint32_t VAL_CFG_SUBSEC_INFMSG  = 0x00000004;
const uint32_t VAL_CFG_SUBSEC_NAVCONF = 0x00000008;
const uint32_t VAL_CFG_SUBSEC_RXMCONF = 0x00000010;

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------
typedef enum {
    SFE_UBLOX_STATUS_SUCCESS = 0,
    SFE_UBLOX_STATUS_FAIL,
    SFE_UBLOX_STATUS_CRC_FAIL,
    SFE_UBLOX_STATUS_TIMEOUT,
    SFE_UBLOX_STATUS_COMMAND_NACK,
    SFE_UBLOX_STATUS_OUT_OF_RANGE,
    SFE_UBLOX_STATUS_INVALID_ARG,
    SFE_UBLOX_STATUS_INVALID_OPERATION,
    SFE_UBLOX_STATUS_MEM_ERR,
    SFE_UBLOX_STATUS_HW_ERR,
    SFE_UBLOX_STATUS_DATA_SENT,
    SFE_UBLOX_STATUS_DATA_RECEIVED,
    SFE_UBLOX_STATUS_I2C_COMM_FAILURE,
    SFE_UBLOX_STATUS_DATA_OVERWRITTEN
} sfe_ublox_status_e;

typedef enum {
    SFE_UBLOX_PACKET_VALIDITY_NOT_VALID,
    SFE_UBLOX_PACKET_VALIDITY_VALID,
    SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
    SFE_UBLOX_PACKET_NOTACKNOWLEDGED
} sfe_ublox_packet_validity_e;

enum dynModel {
    DYN_MODEL_PORTABLE    = 0,
    DYN_MODEL_STATIONARY  = 2,
    DYN_MODEL_PEDESTRIAN  = 3,
    DYN_MODEL_AUTOMOTIVE  = 4,
    DYN_MODEL_SEA         = 5,
    DYN_MODEL_AIRBORNE1g  = 6,
    DYN_MODEL_AIRBORNE2g  = 7,
    DYN_MODEL_AIRBORNE4g  = 8,
    DYN_MODEL_WRIST       = 9,
    DYN_MODEL_BIKE        = 10,
    DYN_MODEL_MOWER       = 11,
    DYN_MODEL_ESCOOTER    = 12,
    DYN_MODEL_UNKNOWN     = 255
};

enum sfe_ublox_gnss_ids_e {
    SFE_UBLOX_GNSS_ID_GPS     = 0,
    SFE_UBLOX_GNSS_ID_SBAS    = 1,
    SFE_UBLOX_GNSS_ID_GALILEO = 2,
    SFE_UBLOX_GNSS_ID_BEIDOU  = 3,
    SFE_UBLOX_GNSS_ID_IMES    = 4,
    SFE_UBLOX_GNSS_ID_QZSS    = 5,
    SFE_UBLOX_GNSS_ID_GLONASS = 6
};

enum sfe_ublox_dgnss_mode_e {
    SFE_UBLOX_DGNSS_MODE_FLOAT = 2,
    SFE_UBLOX_DGNSS_MODE_FIXED = 3
};

enum sfe_ublox_pms_mode_e {
    SFE_UBLOX_PMS_MODE_FULLPOWER      = 0,
    SFE_UBLOX_PMS_MODE_BALANCED       = 1,
    SFE_UBLOX_PMS_MODE_INTERVAL       = 2,
    SFE_UBLOX_PMS_MODE_AGGRESSIVE_1HZ = 3,
    SFE_UBLOX_PMS_MODE_AGGRESSIVE_2HZ = 4,
    SFE_UBLOX_PMS_MODE_AGGRESSIVE_4HZ = 5,
    SFE_UBLOX_PMS_MODE_INVALID        = 0xff
};

enum sfe_ublox_rxm_mode_e {
    SFE_UBLOX_CFG_RXM_CONTINUOUS = 0,
    SFE_UBLOX_CFG_RXM_POWERSAVE  = 1
};

enum sfe_ublox_mga_assist_ack_e {
    SFE_UBLOX_MGA_ASSIST_ACK_NO      = 0,
    SFE_UBLOX_MGA_ASSIST_ACK_YES     = 1,
    SFE_UBLOX_MGA_ASSIST_ACK_ENQUIRE = 2
};

typedef enum {
    SFE_UBLOX_MGA_ACK_INFOCODE_ACCEPTED      = 0,
    SFE_UBLOX_MGA_ACK_INFOCODE_NO_TIME       = 1,
    SFE_UBLOX_MGA_ACK_INFOCODE_NOT_SUPPORTED = 2,
    SFE_UBLOX_MGA_ACK_INFOCODE_SIZE_MISMATCH = 3,
    SFE_UBLOX_MGA_ACK_INFOCODE_NOT_STORED    = 4,
    SFE_UBLOX_MGA_ACK_INFOCODE_NOT_READY     = 5,
    SFE_UBLOX_MGA_ACK_INFOCODE_TYPE_UNKNOWN  = 6
} sfe_ublox_mga_ack_infocode_e;

enum sfe_ublox_ls_src_e {
    SFE_UBLOX_LS_SRC_DEFAULT   = 0,
    SFE_UBLOX_LS_SRC_GLONASS   = 1,
    SFE_UBLOX_LS_SRC_GPS       = 2,
    SFE_UBLOX_LS_SRC_SBAS      = 3,
    SFE_UBLOX_LS_SRC_BEIDOU    = 4,
    SFE_UBLOX_LS_SRC_GALILEO   = 5,
    SFE_UBLOX_LS_SRC_AIDED     = 6,
    SFE_UBLOX_LS_SRC_CONFIGURED= 7,
    SFE_UBLOX_LS_SRC_UNKNOWN   = 255
};

enum sfe_ublox_talker_id_e {
    SFE_UBLOX_MAIN_TALKER_ID_DEFAULT = 0,
    SFE_UBLOX_MAIN_TALKER_ID_GP      = 1,
    SFE_UBLOX_MAIN_TALKER_ID_GL      = 2,
    SFE_UBLOX_MAIN_TALKER_ID_GN      = 3,
    SFE_UBLOX_MAIN_TALKER_ID_GA      = 4,
    SFE_UBLOX_MAIN_TALKER_ID_GB      = 5,
    SFE_UBLOX_MAIN_TALKER_ID_GQ      = 6
};

// ---------------------------------------------------------------------------
// Minimal forward declarations for data struct pointer overloads.
// Only fields actually used by firmware are populated in sim; the rest are
// zeroed. Full struct definitions are omitted — firmware code that references
// struct fields should access them through the getter methods below.
// ---------------------------------------------------------------------------
struct UBX_NAV_PVT_data_t   { uint8_t _pad[100] = {}; };
struct UBX_NAV_DOP_data_t   { uint8_t _pad[20]  = {}; };
struct UBX_NAV_ATT_data_t   { uint8_t _pad[20]  = {}; };
struct UBX_NAV_ODO_data_t   { uint8_t _pad[24]  = {}; };
struct UBX_CFG_TMODE3_data_t{ uint8_t _pad[40]  = {}; };
struct UBX_CFG_TP5_data_t   { uint8_t _pad[32]  = {}; };
struct UBX_CFG_ITFM_data_t  { uint8_t _pad[8]   = {}; };
struct UBX_MON_RF_data_t    { uint8_t _pad[24]  = {}; };
struct UBX_MON_HW_data_t    { uint8_t _pad[60]  = {}; };
struct UBX_MON_HW2_data_t   { uint8_t _pad[28]  = {}; };
struct ubxPacket            { uint8_t _pad[MAX_PAYLOAD_SIZE + 8] = {}; };

struct geofenceState {
    uint8_t status    = 0;
    uint8_t numFences = 0;
    uint8_t combState = 0;
    uint8_t states[4] = {};
};

// ---------------------------------------------------------------------------
// Main GNSS class
// ---------------------------------------------------------------------------
class SFE_UBLOX_GNSS {
public:

    SFE_UBLOX_GNSS() = default;

    // =========================================================================
    // Initialisation
    // =========================================================================
    bool begin(TwoWire& /*wire*/,
               uint8_t  /*addr*/       = 0x42,
               uint16_t /*maxWait*/    = defaultMaxWait,
               bool     /*assumeSuccess*/ = false) { return true; }

    bool begin(Stream&  /*serial*/,
               uint16_t /*maxWait*/    = defaultMaxWait,
               bool     /*assumeSuccess*/ = false) { return true; }

    void end() {}

    bool isConnected(uint16_t /*maxWait*/ = defaultMaxWait) { return true; }

    // =========================================================================
    // Port / protocol configuration — all no-ops
    // =========================================================================
    bool setI2CAddress  (uint8_t /*addr*/,  uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    void setSerialRate  (uint32_t /*baud*/, uint8_t  /*port*/    = COM_PORT_UART1,
                         uint16_t /*maxWait*/ = defaultMaxWait) {}

    bool setI2COutput   (uint8_t /*s*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool setUART1Output (uint8_t /*s*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool setUART2Output (uint8_t /*s*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool setUSBOutput   (uint8_t /*s*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool setSPIOutput   (uint8_t /*s*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool setPortOutput  (uint8_t /*port*/, uint8_t /*s*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool setPortInput   (uint8_t /*port*/, uint8_t /*s*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }

    // =========================================================================
    // Message configuration — all no-ops
    // =========================================================================
    bool configureMessage (uint8_t /*cls*/, uint8_t /*id*/, uint8_t /*port*/,
                           uint8_t /*rate*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool enableMessage    (uint8_t /*cls*/, uint8_t /*id*/, uint8_t /*port*/,
                           uint8_t /*rate*/ = 1, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool disableMessage   (uint8_t /*cls*/, uint8_t /*id*/, uint8_t /*port*/,
                           uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool enableNMEAMessage(uint8_t /*id*/, uint8_t /*port*/,
                           uint8_t /*rate*/ = 1, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool disableNMEAMessage(uint8_t /*id*/, uint8_t /*port*/,
                            uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool enableRTCMmessage (uint8_t /*num*/, uint8_t /*port*/,
                            uint8_t /*rate*/ = 1, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool disableRTCMmessage(uint8_t /*num*/, uint8_t /*port*/,
                            uint16_t /*maxWait*/ = defaultMaxWait) { return true; }

    // =========================================================================
    // Navigation rate
    // =========================================================================
    bool    setNavigationFrequency(uint8_t  /*freq*/,  uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    uint8_t getNavigationFrequency(uint16_t /*maxWait*/ = defaultMaxWait)                     { return 1; }
    bool    setMeasurementRate    (uint16_t /*ms*/,    uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool    setNavigationRate     (uint16_t /*cycles*/, uint16_t /*maxWait*/ = defaultMaxWait){ return true; }

    // =========================================================================
    // Configuration storage & reset — all no-ops
    // =========================================================================
    bool saveConfiguration   (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool saveConfigSelective (uint32_t /*mask*/,   uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool factoryDefault      (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    void factoryReset        () {}
    void hardReset           () {}
    void softwareResetGNSSOnly() {}

    // =========================================================================
    // Navigation data polling
    // All return true immediately; data is available via the getters below.
    // =========================================================================
    bool getPVT          (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVPVT       (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVPOSECEF   (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVPOSLLH    (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVSTATUS    (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getDOP          (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVATT       (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVODO       (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVVELECEF   (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVVELNED    (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVHPPOSECEF (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVHPPOSLLH  (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVTIMEUTC   (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVCLOCK     (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVSVIN      (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVSAT       (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVSIG       (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVGEOFENCE  (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getNAVPVAT      (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }

    // =========================================================================
    // Auto-message configuration — all no-ops
    // =========================================================================
    bool setAutoPVT     (bool /*en*/, uint16_t /*maxWait*/ = defaultMaxWait)  { return true; }
    bool setAutoPVT     (bool /*en*/, bool /*implicit*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool setAutoPVTrate (uint8_t /*rate*/, bool /*implicit*/ = true, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool assumeAutoPVT  (bool /*en*/, bool /*implicit*/)                      { return true; }
    void flushPVT       () {}
    bool logPVT         (bool /*en*/)                                         { return true; }

    bool setAutoDOP     (bool /*en*/, uint16_t /*maxWait*/ = defaultMaxWait)  { return true; }
    bool setAutoDOP     (bool /*en*/, bool /*implicit*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool assumeAutoDOP  (bool /*en*/, bool /*implicit*/)                      { return true; }
    void flushDOP       () {}

    bool setAutoNAVATT  (bool /*en*/, uint16_t /*maxWait*/ = defaultMaxWait)  { return true; }
    bool setAutoNAVATT  (bool /*en*/, bool /*implicit*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool assumeAutoNAVATT(bool /*en*/, bool /*implicit*/)                     { return true; }
    void flushNAVATT    () {}

    bool setAutoNAVODO  (bool /*en*/, uint16_t /*maxWait*/ = defaultMaxWait)  { return true; }
    bool setAutoNAVODO  (bool /*en*/, bool /*implicit*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool assumeAutoNAVODO(bool /*en*/, bool /*implicit*/)                     { return true; }
    void flushNAVODO    () {}

    bool setAutoNAVVELNED(bool /*en*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool setAutoNAVVELNED(bool /*en*/, bool /*implicit*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool assumeAutoNAVVELNED(bool /*en*/, bool /*implicit*/)                  { return true; }
    void flushNAVVELNED () {}

    bool setAutoNAVHPPOSLLH(bool /*en*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool setAutoNAVHPPOSLLH(bool /*en*/, bool /*implicit*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool assumeAutoNAVHPPOSLLH(bool /*en*/, bool /*implicit*/)                { return true; }
    void flushNAVHPPOSLLH() {}

    bool setAutoNAVTIMEUTC(bool /*en*/, uint16_t /*maxWait*/ = defaultMaxWait){ return true; }
    bool setAutoNAVTIMEUTC(bool /*en*/, bool /*implicit*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool assumeAutoNAVTIMEUTC(bool /*en*/, bool /*implicit*/)                 { return true; }
    void flushNAVTIMEUTC () {}

    bool setAutoNAVSAT  (bool /*en*/, uint16_t /*maxWait*/ = defaultMaxWait)  { return true; }
    bool setAutoNAVSAT  (bool /*en*/, bool /*implicit*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool assumeAutoNAVSAT(bool /*en*/, bool /*implicit*/)                     { return true; }
    void flushNAVSAT    () {}

    bool setAutoNAVSIG  (bool /*en*/, uint16_t /*maxWait*/ = defaultMaxWait)  { return true; }
    void flushNAVSIG    () {}

    // =========================================================================
    // Position / velocity / time getters
    // Lat and lon come from the simulator via ArduinoCompat::g_gpsLat/Lon.
    // Everything else returns plausible defaults for an outdoor 3D-fix.
    // =========================================================================

    // Latitude — degrees × 10⁷
    int32_t getLatitude (uint16_t /*maxWait*/ = defaultMaxWait) {
        float deg = ArduinoCompat::g_gpsLat ? *ArduinoCompat::g_gpsLat : 0.0f;
        return (int32_t)(deg * 1e7f);
    }

    // Longitude — degrees × 10⁷
    int32_t getLongitude(uint16_t /*maxWait*/ = defaultMaxWait) {
        float deg = ArduinoCompat::g_gpsLon ? *ArduinoCompat::g_gpsLon : 0.0f;
        return (int32_t)(deg * 1e7f);
    }

    // Altitude above ellipsoid — mm (fixed at 0 m in sim)
    int32_t getAltitude   (uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }
    int32_t getAltitudeMSL(uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }

    // Satellites in view — return a realistic locked count
    uint8_t getSIV(uint16_t /*maxWait*/ = defaultMaxWait) { return 12; }

    // Fix type: 3 = 3D fix
    uint8_t getFixType(uint16_t /*maxWait*/ = defaultMaxWait) { return 3; }

    // Carrier solution type: 0 = none
    uint8_t getCarrierSolutionType(uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }

    // Ground speed — mm/s (stationary in sim)
    int32_t getGroundSpeed(uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }

    // Heading of motion — degrees × 10⁵
    int32_t getHeading(uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }

    // Position dilution of precision × 100 (e.g. 120 = PDOP 1.20)
    uint16_t getPDOP(uint16_t /*maxWait*/ = defaultMaxWait) { return 120; }

    // Horizontal / vertical accuracy estimate — mm
    uint32_t getHorizontalAccEst(uint16_t /*maxWait*/ = defaultMaxWait) { return 1500; }
    uint32_t getVerticalAccEst  (uint16_t /*maxWait*/ = defaultMaxWait) { return 2000; }
    uint32_t getSpeedAccEst     (uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }
    uint32_t getHeadingAccEst   (uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }

    // NED velocities — mm/s
    int32_t getNedNorthVel(uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }
    int32_t getNedEastVel (uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }
    int32_t getNedDownVel (uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }

    // Date & time validity
    bool getDateValid(uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getTimeValid(uint16_t /*maxWait*/ = defaultMaxWait) { return true; }

    // Date & time fields (fixed epoch — not used by navigation firmware)
    uint16_t getYear      (uint16_t /*maxWait*/ = defaultMaxWait) { return 2026; }
    uint8_t  getMonth     (uint16_t /*maxWait*/ = defaultMaxWait) { return 1; }
    uint8_t  getDay       (uint16_t /*maxWait*/ = defaultMaxWait) { return 1; }
    uint8_t  getHour      (uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }
    uint8_t  getMinute    (uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }
    uint8_t  getSecond    (uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }
    int32_t  getNanosecond(uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }
    uint32_t getTimeOfWeek(uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }

    // =========================================================================
    // GNSS system control — no-ops
    // =========================================================================
    bool    setDynamicModel       (dynModel /*m*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    uint8_t getDynamicModel       (uint16_t /*maxWait*/ = defaultMaxWait)                 { return DYN_MODEL_MOWER; }
    bool    enableGNSS            (bool /*en*/, sfe_ublox_gnss_ids_e /*id*/,
                                   uint16_t /*maxWait*/ = defaultMaxWait)                 { return true; }
    bool    isGNSSenabled         (sfe_ublox_gnss_ids_e /*id*/,
                                   uint16_t /*maxWait*/ = defaultMaxWait)                 { return true; }
    bool    setNAV5PositionAccuracy(uint16_t /*m*/, uint16_t /*maxWait*/ = defaultMaxWait){ return true; }
    uint16_t getNAV5PositionAccuracy(uint16_t /*maxWait*/ = defaultMaxWait)               { return 0; }

    // =========================================================================
    // RTK / survey — no-ops
    // =========================================================================
    bool getSurveyMode (uint16_t /*maxWait*/ = defaultMaxWait)               { return true; }
    bool getSurveyMode (UBX_CFG_TMODE3_data_t* /*d*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool setSurveyMode (uint8_t /*mode*/, uint16_t /*obs*/, float /*acc*/,
                        uint16_t /*maxWait*/ = defaultMaxWait)               { return true; }
    bool enableSurveyMode (uint16_t /*obs*/, float /*acc*/,
                           uint16_t /*maxWait*/ = defaultMaxWait)            { return true; }
    bool disableSurveyMode(uint16_t /*maxWait*/ = defaultMaxWait)            { return true; }
    bool setStaticPosition(int32_t, int8_t, int32_t, int8_t,
                           int32_t, int8_t, bool,
                           uint16_t /*maxWait*/ = defaultMaxWait)            { return true; }
    bool setDGNSSConfiguration(sfe_ublox_dgnss_mode_e /*m*/,
                               uint16_t /*maxWait*/ = defaultMaxWait)        { return true; }

    // =========================================================================
    // Power management — no-ops
    // =========================================================================
    bool    powerSaveMode   (bool /*en*/, uint16_t /*maxWait*/ = defaultMaxWait)          { return true; }
    uint8_t getPowerSaveMode(uint16_t /*maxWait*/ = defaultMaxWait)                       { return 0; }
    bool    powerOff        (uint32_t /*ms*/, uint16_t /*maxWait*/ = defaultMaxWait)      { return true; }
    bool    powerOffWithInterrupt(uint32_t /*ms*/, uint32_t /*src*/,
                                  bool /*usb*/, uint16_t /*maxWait*/ = defaultMaxWait)    { return true; }
    bool    setPowerManagement(sfe_ublox_pms_mode_e /*mode*/, uint16_t /*period*/,
                               uint16_t /*onTime*/, uint16_t /*maxWait*/ = defaultMaxWait){ return true; }
    bool    setupPowerMode  (sfe_ublox_rxm_mode_e /*mode*/,
                             uint16_t /*maxWait*/ = defaultMaxWait)                       { return true; }

    // =========================================================================
    // Geofencing — no-ops
    // =========================================================================
    bool addGeofence   (int32_t, int32_t, uint32_t, uint8_t, uint8_t, uint8_t,
                        uint16_t /*maxWait*/ = defaultMaxWait)               { return true; }
    bool clearGeofences(uint16_t /*maxWait*/ = defaultMaxWait)               { return true; }
    bool getGeofenceState(geofenceState& /*s*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }

    // =========================================================================
    // Odometer — no-ops
    // =========================================================================
    bool resetOdometer  (uint16_t /*maxWait*/ = defaultMaxWait)              { return true; }
    bool enableOdometer (bool /*en*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool getOdometerConfig(uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*,
                           uint16_t /*maxWait*/ = defaultMaxWait)            { return true; }
    bool setOdometerConfig(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t,
                           uint16_t /*maxWait*/ = defaultMaxWait)            { return true; }

    // =========================================================================
    // Time pulse — no-ops
    // =========================================================================
    bool getTimePulseParameters(UBX_CFG_TP5_data_t* /*d*/,
                                uint16_t /*maxWait*/ = defaultMaxWait)       { return true; }
    bool setTimePulseParameters(UBX_CFG_TP5_data_t* /*d*/,
                                uint16_t /*maxWait*/ = defaultMaxWait)       { return true; }

    // =========================================================================
    // Jamming / RF monitoring — no-ops
    // =========================================================================
    bool getJammingConfiguration(UBX_CFG_ITFM_data_t* /*d*/,
                                 uint16_t /*maxWait*/ = defaultMaxWait)      { return true; }
    bool setJammingConfiguration(UBX_CFG_ITFM_data_t* /*d*/,
                                 uint16_t /*maxWait*/ = defaultMaxWait)      { return true; }
    bool getRFinformation (UBX_MON_RF_data_t*  /*d*/,
                           uint16_t /*maxWait*/ = defaultMaxWait)            { return true; }
    bool getHWstatus      (UBX_MON_HW_data_t*  /*d*/,
                           uint16_t /*maxWait*/ = defaultMaxWait)            { return true; }
    bool getHW2status     (UBX_MON_HW2_data_t* /*d*/,
                           uint16_t /*maxWait*/ = defaultMaxWait)            { return true; }

    // =========================================================================
    // Protocol / version — no-ops
    // =========================================================================
    uint8_t getProtocolVersionHigh(uint16_t /*maxWait*/ = defaultMaxWait) { return 27; }
    uint8_t getProtocolVersionLow (uint16_t /*maxWait*/ = defaultMaxWait) { return 0;  }
    bool    getProtocolVersion    (uint16_t /*maxWait*/ = defaultMaxWait) { return true; }

    // =========================================================================
    // Configuration key-value system (protocol v27+) — all no-ops
    // =========================================================================
    uint32_t createKey(uint16_t /*group*/, uint16_t /*id*/, uint8_t /*size*/) { return 0; }

    sfe_ublox_status_e getVal  (uint32_t, uint8_t, uint16_t = defaultMaxWait) { return SFE_UBLOX_STATUS_SUCCESS; }
    uint8_t  getVal8  (uint32_t, uint8_t, uint16_t = defaultMaxWait) { return 0; }
    uint16_t getVal16 (uint32_t, uint8_t, uint16_t = defaultMaxWait) { return 0; }
    uint32_t getVal32 (uint32_t, uint8_t, uint16_t = defaultMaxWait) { return 0; }
    uint64_t getVal64 (uint32_t, uint8_t, uint16_t = defaultMaxWait) { return 0; }

    uint8_t setVal  (uint32_t, uint16_t, uint8_t, uint16_t = defaultMaxWait) { return 0; }
    uint8_t setVal8 (uint32_t, uint8_t,  uint8_t, uint16_t = defaultMaxWait) { return 0; }
    uint8_t setVal16(uint32_t, uint16_t, uint8_t, uint16_t = defaultMaxWait) { return 0; }
    uint8_t setVal32(uint32_t, uint32_t, uint8_t, uint16_t = defaultMaxWait) { return 0; }
    uint8_t setVal64(uint32_t, uint64_t, uint8_t, uint16_t = defaultMaxWait) { return 0; }

    uint8_t newCfgValset  (uint8_t)           { return 0; }
    uint8_t newCfgValset8 (uint32_t, uint8_t,  uint8_t) { return 0; }
    uint8_t newCfgValset16(uint32_t, uint16_t, uint8_t) { return 0; }
    uint8_t newCfgValset32(uint32_t, uint32_t, uint8_t) { return 0; }
    uint8_t newCfgValset64(uint32_t, uint64_t, uint8_t) { return 0; }

    uint8_t addCfgValset8 (uint32_t, uint8_t)  { return 0; }
    uint8_t addCfgValset16(uint32_t, uint16_t) { return 0; }
    uint8_t addCfgValset32(uint32_t, uint32_t) { return 0; }
    uint8_t addCfgValset64(uint32_t, uint64_t) { return 0; }

    uint8_t sendCfgValset(uint16_t /*maxWait*/ = defaultMaxWait) { return 0; }

    // =========================================================================
    // AssistNow / SPARTN — no-ops
    // =========================================================================
    size_t pushAssistNowData(const uint8_t* /*data*/, size_t /*n*/,
                             sfe_ublox_mga_assist_ack_e /*ack*/ = SFE_UBLOX_MGA_ASSIST_ACK_NO,
                             uint16_t /*maxWait*/ = defaultMaxWait)          { return 0; }

    bool setUTCTimeAssistance(uint16_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t,
                              uint32_t, uint16_t, uint32_t, uint8_t,
                              sfe_ublox_mga_assist_ack_e = SFE_UBLOX_MGA_ASSIST_ACK_NO,
                              uint16_t = defaultMaxWait)                     { return true; }
    bool setPositionAssistanceXYZ(int32_t, int32_t, int32_t, uint32_t,
                                  sfe_ublox_mga_assist_ack_e = SFE_UBLOX_MGA_ASSIST_ACK_NO,
                                  uint16_t = defaultMaxWait)                 { return true; }
    bool setPositionAssistanceLLH(int32_t, int32_t, int32_t, uint32_t,
                                  sfe_ublox_mga_assist_ack_e = SFE_UBLOX_MGA_ASSIST_ACK_NO,
                                  uint16_t = defaultMaxWait)                 { return true; }
    bool setDynamicSPARTNKey (uint8_t, uint16_t, uint32_t, const char*)      { return true; }
    bool setDynamicSPARTNKeys(uint8_t, uint16_t, uint32_t, const char*,
                              uint8_t, uint16_t, uint32_t, const char*)      { return true; }

    // =========================================================================
    // High-precision mode / talker ID — no-ops
    // =========================================================================
    bool setHighPrecisionMode(bool /*en*/, uint16_t /*maxWait*/ = defaultMaxWait) { return true; }
    bool setMainTalkerID(sfe_ublox_talker_id_e /*id*/,
                         uint16_t /*maxWait*/ = defaultMaxWait)              { return true; }

    // =========================================================================
    // File buffer — no-ops
    // =========================================================================
    void     setFileBufferSize     (uint16_t /*n*/)                { }
    uint16_t getFileBufferSize     ()                              { return 0; }
    uint16_t extractFileBufferData (uint8_t* /*dst*/, uint16_t /*n*/) { return 0; }
    uint16_t fileBufferAvailable   ()                              { return 0; }
    void     clearFileBuffer       ()                              { }

    // =========================================================================
    // Low-level / debug — no-ops
    // =========================================================================
    void enableDebugging (Stream& /*port*/, bool /*limited*/ = false) {}
    void disableDebugging() {}
    void debugPrint  (char* /*msg*/) {}
    void debugPrintln(char* /*msg*/) {}

    bool checkUblox(uint8_t /*cls*/ = 0, uint8_t /*id*/ = 0) { return true; }
    void checkCallbacks() {}

    bool pushRawData(uint8_t* /*data*/, size_t /*n*/, bool /*stop*/ = false) { return true; }

    const char* statusString(sfe_ublox_status_e /*s*/) { return "SIM"; }
    void printPacket(ubxPacket* /*pkt*/, bool /*always*/ = false) {}
};

#endif
