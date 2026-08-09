#pragma once

#include "motor/motor.h"
#include "uv/uv.h"
#include <Arduino.h>
#include <vector>

/// @brief A task that runs on core 0 and performs communication functions
/// @param pvParameters
void communicationTask(void *pvParameters);

/// Intercore communication queue
extern QueueHandle_t intercoreQueue;

/// @brief Bind communication task to a core 0 and init internal vars
void startCommunication();

/// @brief Pointed packet reader
class PacketReader {
  private:
    std::vector<uint8_t> data;
    size_t pos = 0;
    size_t size = 0;

    uint32_t packetStartTime = 0;
    bool waitingPacket = false;

    /// CRC8 gen
    uint8_t crc8(uint8_t *data, size_t len) {
        uint8_t crc = 0x00;
        uint8_t poly = 0x07;

        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];

            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ poly;
                } else {
                    crc <<= 1;
                }
            }
        }

        return crc;
    }

  public:
    /// @brief Parse buffer
    bool parse();

    /// @brief read int32 from buf
    int32_t readInt32();

    /// @brief read double from buf
    double readDouble();

    /// @brief read u8 from buf
    uint8_t readUInt8();

    /// @brief read u16 from buf
    uint16_t readUInt16();
};

extern const uint8_t SYNC_BYTES[];

class PacketWriter {
  private:
    std::vector<uint8_t> data;
    size_t pos = 0;
    size_t size = 0;

    void resize(size_t add) {
        size_t oldSize = data.size();
        data.resize(oldSize + add);
    }

  public:
    void writeBytes(const void *ptr, size_t size) {
        size_t oldSize = data.size();
        data.resize(oldSize + size);

        memcpy(data.data() + oldSize, ptr, size);
    }

    /// @brief write int32 to buf
    void write(int32_t val) { writeBytes(&val, sizeof(val)); }

    /// @brief write double to buf
    void write(double val) { writeBytes(&val, sizeof(val)); }

    /// @brief write u8 to buf
    void write(uint8_t val) { writeBytes(&val, sizeof(val)); }

    /// @brief write u16 to buf
    void write(uint16_t val) { writeBytes(&val, sizeof(val)); }

    /// @brief Convert PacketWriter to a vector
    std::vector<uint8_t> getPacket() { return data; }
};

union CommunicationUnion {
    UVTask ut;          // 40
    MotorTargetTask mt; // 50

    uint16_t current;    // 52
    uint16_t microsteps; // 54

    // <HOME>            // 56
};

struct CommunicationStruct {
    size_t field_id;
    CommunicationUnion u;
};

bool receiveQueue(CommunicationStruct *cs);