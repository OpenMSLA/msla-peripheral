#pragma once

#include "config.h"
#include "endstop.h"
#include "uart/packet.h"

#include <Arduino.h>
#include <GyverStepper.h>
#include <TMCStepper.h>

/// @brief Initialize and begin z motor
void initMotor();

/// @brief Struct, representing a next motor target
struct MotorTargetTask {
    double pos;
    double speed;
    GS_posType positioning;
};

MotorTargetTask parseMotorTargetTask(PacketReader* pr);
void executeMotorTargetTask(MotorTargetTask mt);

enum CallbackNotificationType { SUCCESS, HOMED, HOMING_FAILED, NONE };
enum HomingState { INIT_RAISING, RAISING, FAST_DOWN, SLOW_DOWN, HOMING_NONE };

class ZMotor {
public:
    bool is_homing = false;

    /// @brief Last motor state
    bool last_tick_run_state = false;

    /// @brief When should the communication flow notify the orchestrator about
    /// completion of movement
    CallbackNotificationType must_be_notified = CallbackNotificationType::NONE;

    HomingState homing_state = HomingState::HOMING_NONE;

    ZMotor();

    /// @brief Tick motor run
    bool tick();

    /// @brief Set motor accel
    void setAcceleration(uint16_t acc);

    /// @brief Set motor target (mm)
    void setTarget(double target);

    /// @brief Set target with positioning type
    void setTarget(double target, GS_posType pos);

    /// @brief Set motor max speed
    void setMaxSpeed(double speed);

    /// @brief Set motor speed
    void setSpeed(double speed);

    /// @brief Set driver current
    void setCurrent(uint16_t curr);

    /// @brief Get driver current
    uint16_t getCurrent();

    /// @brief Set micro stepping
    void setMicrosteps(uint16_t ms);

    /// @brief Get stepper state
    bool getState();

    /// @brief Brake stepper
    void brake();

    /// @brief Start homing Z axis
    void home();

    /// @brief Disable stepper
    void disable();

    /// @brief Enable stepper
    void enable();

private:
    /// @brief Z-motor driver preferences
    TMC2208Stepper driver{&Serial2, TMC2208_R_SENSE};

    /// @brief Z-motor
    GStepper<STEPPER2WIRE> stepper{TOTAL_STEPPER_STEPS, STEPPER_STEP_PIN, STEPPER_DIR_PIN,
                                   STEPPER_EN_PIN};

    uint16_t microsteps = DEFAULT_STEPPER_MICROSTEP;

    /// @brief Calling if motor is currently homing
    void tickHome(bool is_running);
};

extern ZMotor motor;