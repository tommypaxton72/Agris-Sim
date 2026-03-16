/*
 * RPLidarC1 Implementation
 * 
 * On real hardware this communicates with the lidar over HardwareSerial.
 * In sim, get_measurement() detects that a DataLayer is registered and
 * pulls points from there instead — no serial port needed.
 * Every other function is unchanged from the original.
 */

#include "RPLidarC1sim.h"
#include "inocompat.h"      // gives us ArduinoCompat::g_dataLayer

RPLidarC1::RPLidarC1(HardwareSerial* serial) {
    _serial = serial;
    _scanning = false;
    _timeout_ms = 1000;
    _simIndex = 0;          // tracks which lidar point we hand out next in sim mode
}

bool RPLidarC1::begin(uint32_t baudrate, uint32_t timeout_ms) {
    _timeout_ms = timeout_ms;

    // In sim there is no serial port, so skip hardware init entirely
    if (ArduinoCompat::g_dataLayer) {
        _scanning = true;
        return true;
    }

    if (!_serial) return false;
    _serial->begin(baudrate);
    delay(50);
    _flush_serial();

    reset();
    RPLidarHealth health;
    return get_health(&health);
}

void RPLidarC1::end() {
    stop_scan();
    if (_serial) {
        _serial->end();
    }
}

bool RPLidarC1::get_health(RPLidarHealth* health) {
    // In sim the lidar is always healthy — no hardware to query
    if (ArduinoCompat::g_dataLayer) {
        health->status     = RPLIDAR_STATUS_OK;
        health->error_code = 0;
        return true;
    }

    if (!health || !_send_command(RPLIDAR_CMD_GET_HEALTH)) {
        return false;
    }
    
    uint32_t response_size;
    if (!_wait_response_header(RPLIDAR_ANS_TYPE_DEVHEALTH, &response_size)) {
        Serial.println("in get_health(): _wait_response_header failed!");
        return false;
    }
    
    if (response_size != 3) {
        Serial.print("in get_health(): calculated response size !=3 but instead= "); Serial.println(response_size);
        return false;
    }
    
    uint8_t buffer[response_size];
    if (!_read_response_data(buffer, response_size)) {
        Serial.println("in get_health(): didn't get expected number of bytes.");
        return false;
    }
    
    health->status     = buffer[0];
    health->error_code = (buffer[2] << 8) | buffer[1];
    
    return true;
}

bool RPLidarC1::start_scan(bool force) {
    // In sim there is nothing to start — data is already flowing from the sim lidar
    if (ArduinoCompat::g_dataLayer) {
        _scanning = true;
        return true;
    }

    uint8_t cmd = force ? RPLIDAR_CMD_FORCE_SCAN : RPLIDAR_CMD_SCAN;
    if (!_send_command(cmd)) return false;
    
    uint32_t response_size;
    if (!_wait_response_header(RPLIDAR_ANS_TYPE_MEASUREMENT, &response_size)) return false;
    
    _scanning = true;
    return true;
}

bool RPLidarC1::stop_scan() {
    // In sim there is nothing to stop
    if (ArduinoCompat::g_dataLayer) {
        _scanning = false;
        return true;
    }

    if (!_send_command(RPLIDAR_CMD_STOP)) return false;
    _scanning = false;
    delay(10);
    _flush_serial();
    return true;
}

bool RPLidarC1::is_scanning() {
    return _scanning;
}

bool RPLidarC1::get_measurement(RPLidarMeasurement* measurement) {
    if (!measurement) return false;

    // --- SIM PATH ---
    // A DataLayer pointer means we are inside the simulator.
    // Walk through lidarData point by point, wrapping back to zero when
    // we reach the end — this mimics the continuous rotation of a real lidar.
    if (ArduinoCompat::g_dataLayer) {
        const LidarData& data = ArduinoCompat::g_dataLayer->lidarData;

        // No scan data available yet this frame
        if (data.count == 0) return false;

        // Wrap the index so the sketch sees a continuous stream of points
        
		if (_simIndex >= data.count) {
			_simIndex = 0;
			if (ArduinoCompat::g_dataLayer) {
				ArduinoCompat::g_dataLayer->lidarData.scanComplete = true;
			}
		}

        const LidarPoint& pt = data.points[_simIndex];

        // Convert radians to degrees — the real lidar outputs degrees
		
        measurement->angle    = 360 - (pt.angle * (180.0f / M_PI));

        // Distance passes through unchanged — both are in mm
        measurement->distance = pt.distance;

        // start_flag marks the first point of a full rotation
        // the sketch uses this to know when a complete scan is ready
        measurement->start_flag = (_simIndex == 0);

        // Sim points are always clean so quality is fixed at max
        measurement->quality      = 15;
        measurement->quality_flag = true;
        measurement->timestamp    = millis();

        _simIndex++;
        return true;
    }

    // --- HARDWARE PATH (unchanged from original) ---
    if (!_scanning) return false;

    const uint32_t t0 = millis();
    uint8_t buf[5];

    while ((millis() - t0) < _timeout_ms) {
        if (_serial->available() >= 5) {
            _serial->readBytes(buf, 5);
            if (_parse_measurement_node(buf, measurement)) {
                measurement->timestamp = millis();
                return true;
            } else {
                // Discard one byte and try to re-sync with the packet boundary
                if (_serial->available() > 0) {
                    uint8_t dummy;
                    _serial->readBytes(&dummy, 1);
                }
            }
        } else {
            delay(1);
        }
    }
    return false;
}

int RPLidarC1::get_measurements(RPLidarMeasurement* measurements, int max_count, uint32_t timeout_ms) {
    if (!measurements || max_count <= 0 || !_scanning) return 0;
    
    int count = 0;
    uint32_t start_time = millis();
    
    while (count < max_count && (millis() - start_time) < timeout_ms) {
        if (get_measurement(&measurements[count])) {
            count++;
        }
    }
    
    return count;
}

bool RPLidarC1::reset() {
    // Nothing to reset in sim
    if (ArduinoCompat::g_dataLayer) return true;

    Serial.print("Resetting Lidar... ");
    if (!_send_command(RPLIDAR_CMD_RESET)) {
        Serial.println("Send failed!");
        return false;
    }
    delay(1000);
    _scanning = false;
    _flush_serial();
    Serial.println("Sent!");
    return true;
}

bool RPLidarC1::disconnect() {
    stop_scan();
    return true;
}

void RPLidarC1::set_timeout(uint32_t timeout_ms) {
    _timeout_ms = timeout_ms;
}

bool RPLidarC1::is_connected() {
    // Always connected in sim
    if (ArduinoCompat::g_dataLayer) return true;

    RPLidarHealth health;
    return get_health(&health);
}

// ---- Private methods (hardware only, unchanged) ----

bool RPLidarC1::_send_command(uint8_t cmd, const uint8_t* payload, uint8_t payload_size) {
    if (!_serial) return false;
    
    uint8_t header[2] = {RPLIDAR_SYNC_BYTE1, cmd};
    _serial->write(header, 2);
    _serial->flush();

    Serial.print("TX: ");
    Serial.print(header[0], HEX);
    Serial.print(":");
    Serial.println(header[1], HEX);
    return true;
}

bool RPLidarC1::_wait_response_header(uint8_t expected_type, uint32_t* response_size) {
    if (!_serial || !response_size) return false;
    
    uint32_t start_time = millis();
    uint8_t header[7] = {0,0,0,0,0,0,0};
    uint8_t pointer = 0;

    while (millis() - start_time < _timeout_ms) {
        if (_serial->available()) {
            header[pointer] = _serial->read();
            pointer++;
        }
        if (pointer == 7) {
            if (header[0] == RPLIDAR_SYNC_BYTE1 && header[1] == RPLIDAR_SYNC_BYTE2) {
                if (expected_type == RPLIDAR_ANS_TYPE_DEVHEALTH) {
                    if (header[2] == RPLIDAR_ANS_TYPE_DEVHEALTH) {
                        *response_size = 3;
                        return true;
                    }
                    return false;
                } else if (expected_type == RPLIDAR_ANS_TYPE_MEASUREMENT) {
                    return (header[6] == RPLIDAR_ANS_TYPE_MEASUREMENT);
                }
            }
        }
        delay(1);
    }
    return false;
}

bool RPLidarC1::_read_response_data(uint8_t* buffer, uint32_t size) {
    if (!_serial || !buffer || size == 0) return false;
    
    uint32_t start_time = millis();
    while (millis() - start_time < _timeout_ms) {
        if (_serial->available() >= (int)size) {
            _serial->readBytes(buffer, size);
            return true;
        }
        delay(1);
    }
    return false;
}

void RPLidarC1::_flush_serial() {
    if (_serial) {
        while (_serial->available()) {
            _serial->read();
        }
    }
}

bool RPLidarC1::_parse_measurement_node(const uint8_t* buffer, RPLidarMeasurement* measurement) {
    if (!buffer || !measurement) return false;

    uint8_t b0 = buffer[0];

    bool start_flag = (b0 & 0x01) != 0;
    bool inv_start  = (b0 & 0x02) != 0;
    uint8_t quality = (b0 >> 2) & 0x3F;

    // These two bits must be opposite — if they match the packet is corrupt
    if (start_flag == inv_start) return false;

    uint16_t raw_dist  = (uint16_t)(buffer[3] | ((uint16_t)buffer[4] << 8));
    uint16_t raw_angle = (uint16_t)((buffer[1] >> 1) | ((uint16_t)buffer[2] << 7));

    raw_dist &= 0x7FFF;

    measurement->distance   = raw_dist  * 0.25f;
    measurement->angle      = raw_angle / 64.0f;

    if (measurement->angle >= 360.0f) measurement->angle = fmodf(measurement->angle, 360.0f);
    if (measurement->angle <    0.0f) measurement->angle += 360.0f;

    measurement->start_flag   = start_flag;
    measurement->quality_flag = true;
    measurement->quality      = quality;

    return true;
}

void RPLidarC1::print_health(RPLidarHealth* health) {
    Serial.println("=== RPLidar Health ===");
    Serial.print("Status: ");
    Serial.println(get_health_status_string(health->status));
    Serial.print("Error Code: 0x");
    Serial.println(health->error_code, HEX);
}

const char* RPLidarC1::get_health_status_string(uint8_t status) {
    switch (status) {
        case RPLIDAR_STATUS_OK:      return "OK";
        case RPLIDAR_STATUS_WARNING: return "WARNING";
        case RPLIDAR_STATUS_ERROR:   return "ERROR";
        default:                     return "UNKNOWN";
    }
}
