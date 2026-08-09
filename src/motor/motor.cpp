#include "motor.h"
#include "config.h"
#include "uart/communication.h"

ZMotor motor;

/// @brief Infinity motor task
void motorTask(void *pvParameters) {
    (void)pvParameters;

    while (true) {
        motor.tick();
        taskYIELD();
    }
}

void initMotor() {
    xTaskCreatePinnedToCore(
        motorTask, "MotorTask", 4096, nullptr, 1, nullptr, 1);
}

MotorTargetTask parseMotorTargetTask(PacketReader *pr) {
    MotorTargetTask mt;

    mt.pos = pr->readInt32();
    mt.speed = pr->readDouble();

    return mt;
}

void executeMotorTargetTask(MotorTargetTask mt) {
    motor.setTarget(mt.pos);
    motor.setMaxSpeed(mt.speed);
}