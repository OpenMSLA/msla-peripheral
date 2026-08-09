#pragma once

#include "config.h"
#include "endstop.h"
#include <Arduino.h>
#include <GyverStepper.h>
#include <TMCStepper.h>

/// @brief Initialize and begin z motor
void initMotor();

/// @brief Struct, representing a next motor target
struct MotorTargetTask {
    int32_t pos;
    double speed;
};

class PacketReader;

MotorTargetTask parseMotorTargetTask(PacketReader *pr);
void executeMotorTargetTask(MotorTargetTask mt);

enum CallbackNotificationType {
    SUCCESS,
    Z_ENDSTOP_FAILED,
    HOMED,
    HOMING_FAILED,
    NONE
};
enum HomingState { INIT_RAISING, RAISING, FAST_DOWN, SLOW_DOWN, HOMING_NONE };

class ZMotor {
  public:
    bool is_homed = false;
    bool is_homing = false;

    /// @brief Last motor state
    bool last_tick_run_state = false;

    /// @brief When should the communication flow notify the orchestrator about
    /// completion of movement
    CallbackNotificationType must_be_notified = CallbackNotificationType::NONE;

    HomingState homing_state = HomingState::HOMING_NONE;

    ZMotor() {
        Serial2.begin(
            115200, SERIAL_8N1, STEPPER_UART_RX_PIN, STEPPER_UART_TX_PIN);

        driver.begin();
        setCurrent(500);
        setMicrosteps(DEFAULT_STEPPER_MICROSTEP);
        stepper.setRunMode(FOLLOW_POS);
        stepper.setAcceleration(Z_MOTOR_ACCEL);
    }

    bool tick() {
        bool st = stepper.tick();

        // Ensure that endstop will be updated after release
        if (end.triggered) {
            end.manualUpdate();
        }

        /// If motor is homing
        if (is_homing) {
            tickHome(st);
        } else {
            if (st && end.triggered && stepper.dir == -1) {
                brake();
                must_be_notified = CallbackNotificationType::Z_ENDSTOP_FAILED;
            } else if (!st && last_tick_run_state) {
                if (must_be_notified == CallbackNotificationType::NONE) {
                    must_be_notified = CallbackNotificationType::SUCCESS;
                }
            }
        }

        last_tick_run_state = st;
        return st;
    }

    void setAcceleration(uint16_t acc) { stepper.setAcceleration(acc); }

    void setTarget(int32_t target) {
        if (target < 0)
            target = 0;
        stepper.setTarget(MM_TO_STEPS(target));
    }

    void setMaxSpeed(double speed) { stepper.setMaxSpeed(speed); }

    void setCurrent(uint16_t curr) { driver.rms_current(curr); }

    uint16_t getCurrent() { return driver.rms_current(); }

    void setMicrosteps(uint16_t ms) {
        driver.microsteps(ms);
        microsteps = ms;
    }

    bool getState() { return stepper.getState(); }

    void brake() {
        stepper.reset();
        end.manualUpdate();
        last_tick_run_state = false;
        must_be_notified = CallbackNotificationType::NONE;
        is_homed = false;
    }

    /// @brief Calling if motor is currently homing
    void tickHome(bool is_running) {
        switch (homing_state) {
        case HomingState::HOMING_NONE:
            stepper.setRunMode(FOLLOW_POS);
            stepper.setMaxSpeed(MM_TO_STEPS(Z_MOTOR_PRIMARY_HOMING_SPEED));
            stepper.setTarget(MM_TO_STEPS(Z_MOTOR_RISING_HEIGHT), RELATIVE);

            homing_state = HomingState::INIT_RAISING;
            break;

        case HomingState::INIT_RAISING:
            if (!is_running) {
                vTaskDelay(pdMS_TO_TICKS(300));
                stepper.setRunMode(KEEP_SPEED);
                stepper.setSpeed(
                    abs(MM_TO_STEPS(Z_MOTOR_PRIMARY_HOMING_SPEED)) * -1);

                homing_state = HomingState::FAST_DOWN;
            }
            break;

        case HomingState::FAST_DOWN:
            if (end.triggered) {
                stepper.brake();
                stepper.setRunMode(FOLLOW_POS);
                stepper.setMaxSpeed(MM_TO_STEPS(Z_MOTOR_PRIMARY_HOMING_SPEED));
                stepper.setTarget(MM_TO_STEPS(Z_MOTOR_RISING_HEIGHT), RELATIVE);

                homing_state = HomingState::RAISING;
            }
            break;

        case HomingState::RAISING:
            if (!is_running) {
                vTaskDelay(pdMS_TO_TICKS(300));
                stepper.setRunMode(KEEP_SPEED);
                stepper.setSpeed(
                    abs(MM_TO_STEPS(Z_MOTOR_SECONDARY_HOMING_SPEED)) * -1);

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
                is_homed = true;
            }
            break;
        }
    }

    /// @brief Start homing Z axis
    void home() { is_homing = true; }

  private:
    /// @brief Z-motor driver preferences
    TMC2208Stepper driver{&Serial2, TMC2208_R_SENSE};

    /// @brief Z-motor
    GStepper<STEPPER2WIRE> stepper{
        TOTAL_STEPPER_STEPS, STEPPER_STEP_PIN, STEPPER_DIR_PIN, STEPPER_EN_PIN};

    uint16_t microsteps = DEFAULT_STEPPER_MICROSTEP;
};

extern ZMotor motor;