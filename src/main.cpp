#include "config.h"
#include "motor/motor.h"
#include "uart/communication.h"
#include "uart/uart.h"
#include <Arduino.h>

void setup() {
    Serial.begin(115200);

    startCommunication();
    initMotor();
    initUV();
}

void loop() {
    CommunicationStruct cs;
    if (receiveQueue(&cs)) {
        switch (cs.field_id) {
        // UV command
        case 40: {
            executeUVTask(cs.u.ut);
            UARTSend(0x29); // 41 dec
            break;
        }

        // motor command
        case 50: {
            executeMotorTargetTask(cs.u.mt);
            setMotorFinishTask([]() {
                UARTSend(0x33); // 51 dec
            });
            break;
        }

        case 52: {
            motor.setCurrent(cs.u.current);

            PacketWriter p;
            p.write((uint8_t)0x35); // 53 dec
            p.write(motor.getCurrent());

            UARTSend(p);
            break;
        }

        case 54: {
            motor.setMicrosteps(cs.u.microsteps);
            UARTSend(0x37); // 55 dec
            break;
        }
        }

        delay(5);
    }
}