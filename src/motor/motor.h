#pragma once

#include "config.h"
#include <Arduino.h>
#include <GyverStepper.h>
#include <TMCStepper.h>

/// @brief Initialize and begin z motor
void initMotor();

/// @brief Struct, representing a next motor target
struct MotorTargetTask {
    int32_t pos;
    double speed;
    u_int16_t accel;
};

class PacketReader;

MotorTargetTask parseMotorTargetTask(PacketReader *pr);
void executeMotorTargetTask(MotorTargetTask mt);

extern std::function<void()> motor_finish_callback;

/// @brief Set callback, that will be called once, at motor stops
void setMotorFinishTask(std::function<void()> callback);

class ZMotor {
  public:
    bool is_homed = false;

    ZMotor() {
        Serial2.begin(
            115200, SERIAL_8N1, STEPPER_UART_RX_PIN, STEPPER_UART_TX_PIN);

        driver.begin();
        setCurrent(500);
        setMicrosteps(DEFAULT_STEPPER_MICROSTEP);
        stepper.setRunMode(FOLLOW_POS);
    }

    bool tick() { return stepper.tick(); }

    void setAcceleration(uint16_t acc) { stepper.setAcceleration(acc); }

    void setTarget(int32_t target) { stepper.setTarget(target); }

    void setMaxSpeed(double speed) { stepper.setMaxSpeed(speed); }

    void setCurrent(uint16_t curr) { driver.rms_current(curr); }

    uint16_t getCurrent() { return driver.rms_current(); }

    void setMicrosteps(uint16_t ms) {
        driver.microsteps(ms);
        microsteps = ms;
    }

  private:
    /// @brief Z-motor driver preferences
    TMC2208Stepper driver{&Serial2, TMC2208_R_SENSE};

    /// @brief Z-motor
    GStepper<STEPPER2WIRE> stepper{
        TOTAL_STEPPER_STEPS, STEPPER_STEP_PIN, STEPPER_DIR_PIN, STEPPER_EN_PIN};

    uint16_t microsteps = DEFAULT_STEPPER_MICROSTEP;
};

extern ZMotor motor;