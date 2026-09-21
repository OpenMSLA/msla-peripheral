#pragma once

#include "communication.h"
#include "packet.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <vector>

extern HardwareSerial OrchestratorSerial;

/// @brief A sequence of sync bytes
extern const uint8_t SYNC_BYTES[];
extern const size_t SYNC_LEN;

/// @brief A UART buffer for holding a upcoming packets
extern std::vector<uint8_t> UART_BUFFER;

/// @brief UART mutex
extern SemaphoreHandle_t uartMutex;

/// @brief Gen crc8
/// @return
uint8_t crc8(uint8_t* data, size_t len);

/// @brief Config UART
void UARTBegin();

/// @brief Send data to a serial
/// @param data payload
/// @param len payload len
void UARTSend(uint8_t* data, size_t len);

/// @brief Send one byte to a serial
void UARTSend(uint8_t byte);

/// @brief Send entire packet to a serial
void UARTSend(PacketWriter byte);