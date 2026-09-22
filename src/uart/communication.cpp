#include "communication.h"

#include "config.h"
#include "motor/motor.h"
#include "packet.h"
#include "uart.h"

#include <Arduino.h>

QueueHandle_t intercoreQueue = nullptr;

bool receiveQueue(CommunicationStruct* cs) {
    return xQueueReceive(intercoreQueue, cs, portMAX_DELAY);
}

void startCommunication() {
    UARTBegin();
    intercoreQueue = xQueueCreate(10, sizeof(CommunicationStruct));

    xTaskCreatePinnedToCore(communicationTask, // Tasks function
                            "CommTask",        // Debug name
                            2048,              // Stack size in bytes
                            NULL,              // Argument
                            1,                 // Priority (low)
                            NULL,              // Handle
                            0                  // Core number: 0
    );
}

void communicationTask(void* pvParameters) {
    while (1) {
        xSemaphoreTake(uartMutex, portMAX_DELAY);
        while (OrchestratorSerial.available()) {
            UART_BUFFER.push_back(OrchestratorSerial.read());
        }
        xSemaphoreGive(uartMutex);

        /// Notify the orchestrator when the motor has reached the target point
        /// or failed
        if (motor.must_be_notified != CallbackNotificationType::NONE) {
            PacketWriter pw;

            switch (motor.must_be_notified) {
                case SUCCESS:
                    pw.write((uint8_t)51);
                    pw.write((uint8_t)0);
                    break;

                case HOMED:
                    pw.write((uint8_t)57);
                    pw.write((uint8_t)0);
                    break;
            }

            motor.must_be_notified = CallbackNotificationType::NONE;
            UARTSend(pw);
        }

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

                case 56:
                    // <HOME>
                    break;

                case 58:
                    // <DISABLE STEPPER>
                    break;

                case 60:
                    // <ENABLE STEPPER>
                    break;

                case 62:
                    // <GET STEPPER POS>
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