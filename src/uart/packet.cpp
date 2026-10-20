#include "packet.h"

#include "uart.h"

#include <Arduino.h>
#include <config.h>

bool startsWithSync() {
    if (UART_BUFFER.size() < SYNC_LEN)
        return false;

    return UART_BUFFER[0] == SYNC_BYTES[0] && UART_BUFFER[1] == SYNC_BYTES[1];
}

bool PacketReader::parse() {
    if (millis() - packetStartTime > COMMUNICATION_TIMEOUT && UART_BUFFER.size() >= SYNC_LEN &&
        waitingPacket) {
        UART_BUFFER.erase(UART_BUFFER.begin()); // We corrupt the buffer at the beginning so
                                                // that the parser skips damaged packets

        packetStartTime = millis();
        waitingPacket = false;
    }

    // Looking for packet start
    while (UART_BUFFER.size() >= SYNC_LEN && !startsWithSync()) {
        UART_BUFFER.erase(UART_BUFFER.begin());
    }

    if (UART_BUFFER.size() < SYNC_LEN)
        return false;

    if (packetStartTime == 0) {
        packetStartTime = millis();
        waitingPacket = true;
    }

    short len_len = sizeof(uint16_t);
    if (UART_BUFFER.size() < SYNC_LEN + len_len)
        return false;

    uint16_t len;
    memcpy(&len, &UART_BUFFER[SYNC_LEN], len_len);

    //                2          2      len   crc8
    size_t total = SYNC_LEN + len_len + len + 1;

    if (UART_BUFFER.size() < total)
        return false;

    this->data.clear();
    this->pos = 0;
    size_t pos = SYNC_LEN + len_len;
    while (pos < total - 1) {
        this->data.push_back(UART_BUFFER[pos++]);
    }

    uint8_t crc;
    memcpy(&crc, &UART_BUFFER[pos], 1);

    if (crc8(data.data(), data.size()) != crc) {
        this->data.clear();

        // throw out only the first byte and look for the next SYNC
        UART_BUFFER.erase(UART_BUFFER.begin());

        packetStartTime = 0;
        waitingPacket = false;

        return false;
    }

    UART_BUFFER.erase(UART_BUFFER.begin(), UART_BUFFER.begin() + total);

    packetStartTime = 0;
    waitingPacket = false;
    return true;
}

int32_t PacketReader::readInt32() {
    int32_t value;

    memcpy(&value, &data[pos], sizeof(int32_t));
    pos += sizeof(int32_t);

    return value;
}

double PacketReader::readDouble() {
    double value;

    memcpy(&value, &data[pos], 8);
    pos += 8;

    return value;
}

uint8_t PacketReader::readUInt8() {
    return this->data[this->pos++];
}

uint16_t PacketReader::readUInt16() {
    uint16_t value;

    memcpy(&value, &data[pos], sizeof(uint16_t));
    pos += sizeof(uint16_t);

    return value;
}