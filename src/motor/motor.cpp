#include "motor.h"
#include "config.h"
#include "uart/communication.h"

ZMotor motor;

void motorTask(void *pvParameters) {
    (void)pvParameters;

    while (true) {
        bool running = motor.tick();

        if (running) {
            taskYIELD();
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

void initMotor() {
    xTaskCreatePinnedToCore(
        motorTask, "MotorTask", 4096, nullptr, 3, nullptr, 1);
}

MotorTargetTask parseMotorTargetTask(PacketReader *pr) {
    MotorTargetTask mt;

    mt.pos = pr->readInt32();
    mt.speed = pr->readDouble();
    mt.accel = pr->readUInt16();

    return mt;
}

void executeMotorTargetTask(MotorTargetTask mt) {
    motor.setAcceleration(mt.accel);
    motor.setTarget(mt.pos);
    motor.setMaxSpeed(mt.speed);
}