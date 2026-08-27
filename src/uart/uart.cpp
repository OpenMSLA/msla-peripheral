#include "uart.h"
#include "config.h"
#include <vector>

std::vector<uint8_t> UART_BUFFER;

const uint8_t SYNC_BYTES[] = {0xFF, 0x55};
const size_t SYNC_LEN = 2;

SemaphoreHandle_t uartMutex = nullptr;

#ifdef ORCHESTRATOR_HARDWARE_SERIAL
HardwareSerial OrchestratorSerial(PHYSICAL_SERIAL);
#else
HardwareSerial OrchestratorSerial = Serial;
#endif

void UARTBegin() {
#ifdef ORCHESTRATOR_HARDWARE_SERIAL
    OrchestratorSerial.begin(ORCHESTRATOR_SERIAL_BAUD_RATE, SERIAL_8N1,
        ORCHESTRATOR_RX_PIN, ORCHESTRATOR_TX_PIN);

    Serial.begin(115200);
#else
    OrchestratorSerial.begin(ORCHESTRATOR_SERIAL_BAUD_RATE);
#endif
    uartMutex = xSemaphoreCreateMutex();
}

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

void UARTSend(uint8_t *data, size_t len) {
    xSemaphoreTake(uartMutex, portMAX_DELAY);

    OrchestratorSerial.write(SYNC_BYTES, SYNC_LEN);
    OrchestratorSerial.write((uint8_t *)&len, sizeof(uint16_t));
    OrchestratorSerial.write(data, len);

    uint8_t crc = crc8(data, len);
    OrchestratorSerial.write(crc);

    xSemaphoreGive(uartMutex);
}

void UARTSend(uint8_t byte) { UARTSend(&byte, 1); }

void UARTSend(PacketWriter packet) {
    std::vector<uint8_t> pack = packet.getPacket();
    UARTSend(pack.data(), pack.size());
}