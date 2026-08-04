#include "uart.h"
#include "config.h"
#include <vector>

std::vector<uint8_t> UART_BUFFER;

const uint8_t SYNC_BYTES[] = {0xFF, 0x55};
const size_t SYNC_LEN = 2;

SemaphoreHandle_t uartMutex = nullptr;

void UARTBegin() { uartMutex = xSemaphoreCreateMutex(); }

void UARTSend(uint8_t *data, size_t len) {
    xSemaphoreTake(uartMutex, portMAX_DELAY);

    Serial.write(SYNC_BYTES, SYNC_LEN);
    Serial.write(len);
    Serial.write(data, len);

    xSemaphoreGive(uartMutex);
}

void UARTSend(uint8_t byte) { UARTSend(&byte, 1); }

void UARTSend(PacketWriter packet) {
    std::vector<uint8_t> pack = packet.getPacket();
    UARTSend(pack.data(), pack.size());
}