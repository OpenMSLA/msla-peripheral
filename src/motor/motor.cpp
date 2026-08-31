#include "motor.h"
#include "config.h"
#include "uart/communication.h"

ZMotor::ZMotor() {
    Serial2.begin(115200, SERIAL_8N1, STEPPER_UART_RX_PIN, STEPPER_UART_TX_PIN);

    driver.begin();
    setCurrent(DEFAULT_STEPPER_DRIVER_CURRENT);
    setMicrosteps(DEFAULT_STEPPER_MICROSTEP);
    stepper.setRunMode(FOLLOW_POS);
    stepper.setAcceleration(MM_TO_STEPS(Z_MOTOR_ACCEL));

    driver.TPWMTHRS(10);
}

bool ZMotor::tick() {
    bool st = stepper.tick();

    // Ensure that endstop will be updated after release
    if (end.triggered) {
        end.manualUpdate();
    }

    bool is_down = stepper.getTarget() - stepper.getCurrent() < 0;

    /// If motor is homing
    if (is_homing) {
        tickHome(st);
    } else if (!st && last_tick_run_state) {
        if (must_be_notified == CallbackNotificationType::NONE) {
            must_be_notified = CallbackNotificationType::SUCCESS;
        }
    }

    last_tick_run_state = st;
    return st;
}

void ZMotor::setAcceleration(uint16_t acc) { stepper.setAcceleration(acc); }

void ZMotor::setTarget(double target) {
    stepper.setTarget(MM_TO_STEPS(fmin(target, Z_HEIGHT)));
}

void ZMotor::setTarget(double target, GS_posType pos) {
    switch (pos) {
    case GS_posType::ABSOLUTE:
        target = fmin(target, Z_HEIGHT);
        break;

    case GS_posType::RELATIVE:
        double max_target = Z_HEIGHT - STEPS_TO_MM(this->stepper.pos);

        if (target > max_target) {
            target = max_target;
        }
        break;
    }

    stepper.setTarget(MM_TO_STEPS(target));
}

void ZMotor::setMaxSpeed(double speed) {
    stepper.setMaxSpeed(MM_TO_STEPS(speed) / 60);
}

void ZMotor::setSpeed(double speed) {
    stepper.setSpeed(MM_TO_STEPS(speed) / 60);
}

void ZMotor::setCurrent(uint16_t curr) { driver.rms_current(curr); }

uint16_t ZMotor::getCurrent() { return driver.rms_current(); }

void ZMotor::setMicrosteps(uint16_t ms) {
    driver.microsteps(ms);
    microsteps = ms;
}

bool ZMotor::getState() { return stepper.getState(); }

void ZMotor::brake() {
    stepper.reset();
    end.manualUpdate();
    last_tick_run_state = false;
    must_be_notified = CallbackNotificationType::NONE;
}

/// @brief Calling if motor is currently homing
void ZMotor::tickHome(bool is_running) {
    switch (homing_state) {
    case HomingState::HOMING_NONE:
        stepper.setRunMode(FOLLOW_POS);
        motor.setMaxSpeed(Z_MOTOR_PRIMARY_HOMING_SPEED);
        motor.setTarget(Z_MOTOR_RISING_HEIGHT, RELATIVE);

        homing_state = HomingState::INIT_RAISING;
        break;

    case HomingState::INIT_RAISING:
        if (!is_running) {
            vTaskDelay(pdMS_TO_TICKS(300));
            stepper.setRunMode(KEEP_SPEED);
            motor.setSpeed(abs(Z_MOTOR_PRIMARY_HOMING_SPEED) * -1);

            homing_state = HomingState::FAST_DOWN;
        }
        break;

    case HomingState::FAST_DOWN:
        if (end.triggered) {
            stepper.brake();
            stepper.setRunMode(FOLLOW_POS);
            motor.setMaxSpeed(Z_MOTOR_PRIMARY_HOMING_SPEED);
            motor.setTarget(Z_MOTOR_RISING_HEIGHT, RELATIVE);

            homing_state = HomingState::RAISING;
        }
        break;

    case HomingState::RAISING:
        if (!is_running) {
            vTaskDelay(pdMS_TO_TICKS(300));
            stepper.setRunMode(KEEP_SPEED);
            motor.setSpeed(abs(Z_MOTOR_SECONDARY_HOMING_SPEED) * -1);

            homing_state = HomingState::SLOW_DOWN;
        }
        break;

    case HomingState::SLOW_DOWN:
        if (end.triggered) {
            stepper.brake();
            stepper.setRunMode(FOLLOW_POS);
            stepper.reset();

            homing_state = HomingState::HOMING_NONE;
            must_be_notified = HOMED;
            is_homing = false;
        }
        break;
    }
}

/// @brief Start homing Z axis
void ZMotor::home() { is_homing = true; }

void ZMotor::disable() { stepper.disable(); }

void ZMotor::enable() { stepper.enable(); }

////////////////////////////////////////////////////////////////////////////////////////////

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

    mt.pos = pr->readDouble();
    mt.speed = pr->readDouble();
    switch (pr->readUInt8()) {
    case 0:
        mt.positioning = GS_posType::ABSOLUTE;
        break;

    case 1:
        mt.positioning = GS_posType::RELATIVE;
        break;
    }

    return mt;
}

void executeMotorTargetTask(MotorTargetTask mt) {
    motor.enable();
    motor.is_homing = false;
    motor.setTarget(mt.pos, mt.positioning);
    motor.setMaxSpeed(mt.speed);
}