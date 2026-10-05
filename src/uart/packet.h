#pragma once

#include <Arduino.h>
#include <vector>

/// @brief Check if uart buffer is starts with sync bytes
/// @return bool
bool startsWithSync();

/// @brief Pointed packet reader
class PacketReader {
private:
    std::vector<uint8_t> data;
    size_t pos = 0;
    size_t size = 0;

    uint32_t packetStartTime = 0;
    bool waitingPacket = false;

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
    void writeBytes(const void* ptr, size_t size) {
        size_t oldSize = data.size();
        data.resize(oldSize + size);

        memcpy(data.data() + oldSize, ptr, size);
    }

    /// @brief write int32 to buf
    void write(int32_t val) {
        writeBytes(&val, sizeof(val));
    }

    /// @brief write double to buf
    void write(double val) {
        writeBytes(&val, sizeof(val));
    }

    /// @brief write u8 to buf
    void write(uint8_t val) {
        writeBytes(&val, sizeof(val));
    }

    /// @brief write u16 to buf
    void write(uint16_t val) {
        writeBytes(&val, sizeof(val));
    }

    /// @brief Convert PacketWriter to a vector
    std::vector<uint8_t> getPacket() {
        return data;
    }

    void erase() {
        data.clear();
        pos = 0;
        size = 0;
    }
};
