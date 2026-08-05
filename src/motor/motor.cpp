#include "motor.h"
#include "config.h"
#include "uart/communication.h"

hw_timer_t *timer = nullptr;
ZMotor motor;

void stepperTimer() { motor.tick(); }

void initMotor() {
    timer = timerBegin(0, 80, true); // 80 МГц / 80 = 1 МГц
    timerAttachInterrupt(timer, &stepperTimer, true);
    timerAlarmWrite(timer, 1000000.0 / STEPPER_MAX_SPEED, true);
    timerAlarmEnable(timer);
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

std::function<void()> motor_finish_callback = nullptr;

void setMotorFinishTask(std::function<void()> callback) {
    motor_finish_callback = callback;
}