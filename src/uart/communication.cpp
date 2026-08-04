#include "communication.h"
#include "config.h"
#include "motor/motor.h"
#include "uart.h"
#include <Arduino.h>

QueueHandle_t intercoreQueue = nullptr;

bool receiveQueue(CommunicationStruct *cs) {
    return xQueueReceive(intercoreQueue, cs, portMAX_DELAY);
}

void startCommunication() {
    UARTBegin();
    intercoreQueue = xQueueCreate(10, sizeof(CommunicationStruct));

    xTaskCreatePinnedToCore(communicationTask, // Tasks function
        "CommTask",                            // Debug name
        2048,                                  // Stack size in bytes
        NULL,                                  // Argument
        1,                                     // Priority (low)
        NULL,                                  // Handle
        0                                      // Core number: 0
    );
}

void communicationTask(void *pvParameters) {
    while (1) {
        xSemaphoreTake(uartMutex, portMAX_DELAY);
        while (Serial.available()) {
            UART_BUFFER.push_back(Serial.read());
        }
        xSemaphoreGive(uartMutex);

        PacketReader pr;
        while (pr.parse()) {
            u_int8_t cmd = pr.readUInt8();

            CommunicationStruct commStruct;
            switch (cmd) {
            case 40:
                commStruct.u.ut = parseUVTask(&pr);
                break;
            case 50:
                commStruct.u.mt = parseMotorTargetTask(&pr);
                break;

            case 52:
                commStruct.u.current = pr.readUInt16();
                break;

            case 54:
                commStruct.u.microsteps = pr.readUInt16();
                break;

            default:
                continue;
            }

            commStruct.field_id = cmd;
            xQueueSend(intercoreQueue, &commStruct, portMAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/// @brief Check if uart buffer is starts with sync bytes
/// @return bool
bool startsWithSync() {
    if (UART_BUFFER.size() < SYNC_LEN)
        return false;

    return UART_BUFFER[0] == SYNC_BYTES[0] && UART_BUFFER[1] == SYNC_BYTES[1];
}

bool PacketReader::parse() {
    if (millis() - packetStartTime > COMMUNICATION_TIMEOUT &&
        UART_BUFFER.size() >= SYNC_LEN && waitingPacket) {
        UART_BUFFER.erase(
            UART_BUFFER.begin()); // We corrupt the buffer at the beginning so
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

    if (UART_BUFFER.size() < SYNC_LEN + 1)
        return false;

    uint8_t len = UART_BUFFER[SYNC_LEN];
    size_t total = SYNC_LEN + 1 + len;

    if (UART_BUFFER.size() < total)
        return false;

    this->data.clear();
    size_t pos = SYNC_LEN + 1;
    while (pos < total) {
        this->data.push_back(UART_BUFFER[pos++]);
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

uint8_t PacketReader::readUInt8() { return this->data[this->pos++]; }

uint16_t PacketReader::readUInt16() {
    uint16_t value;

    memcpy(&value, &data[pos], sizeof(uint16_t));
    pos += sizeof(uint16_t);

    return value;
}