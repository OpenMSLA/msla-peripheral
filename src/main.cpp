#include "config.h"
#include "motor/endstop.h"
#include "motor/motor.h"
#include "uart/communication.h"
#include "uart/uart.h"

#include <Arduino.h>

void setup() {
    startCommunication();
    initEndstop();
    initMotor();
    initUV();
}

void loop() {
    CommunicationStruct cs;
    if (receiveQueue(&cs)) {
        PacketWriter pw;
        switch (cs.field_id) {
            // UV command
            case 40: {
                executeUVTask(cs.u.ut);
                UARTSend(41);
                break;
            }

            // motor command
            case 50: {
                executeMotorTargetTask(cs.u.mt);
                break;
            }

            case 52: {
                motor.setCurrent(cs.u.current);

                pw.write((uint8_t)53);
                pw.write(motor.getCurrent());

                UARTSend(pw);
                break;
            }

            case 56: {
                motor.home();
                break;
            }

            case 58:
                motor.disable();
                UARTSend(59);
                break;

            case 60:
                motor.enable();
                UARTSend(61);
                break;

            case 62:
                // Current motor pos in mm
                pw.write((uint8_t)63);
                pw.write(motor.get_motor_pos());
                UARTSend(pw);
                break;

            case 64:
                // Current UV state
                pw.write((uint8_t)65);
                pw.write(getUVState());
                UARTSend(pw);
                break;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}