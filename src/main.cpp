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

    ledcSetup(0, SYSTEM_FAN_FREQ, 8);
    ledcAttachPin(SYSTEM_FAN_PIN, 0);
}

void loop() {
    CommunicationStruct cs;
    if (receiveQueue(&cs)) {
        PacketWriter pw;
        pw.erase();

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
                // Current physical state
                pw.write((uint8_t)63);
                pw.write((uint8_t)getUVState());
                pw.write(motor.get_motor_pos());
                UARTSend(pw);
                break;

            case 70:
                // Set sys fan speed
                ledcWrite(0, cs.u.sys_fan_power);
                // Dont send reply, cuz we don't need :)
                // UARTSend((uint8_t)71);
                break;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}