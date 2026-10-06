
/// Pin to which the UV lamp control relay is connected
#define UV_RELAY_PIN 14

/// Number of steps per one revolution
#define STEPPER_STEPS 200

/// Default stepper current configuration (mA)
#define DEFAULT_STEPPER_DRIVER_CURRENT 500

/// Default stepper pins
#define STEPPER_STEP_PIN 5
#define STEPPER_DIR_PIN 18
#define STEPPER_EN_PIN 19

/// TMC2208 UART pins
#define STEPPER_UART_RX_PIN 16
#define STEPPER_UART_TX_PIN 17

/// Max speed of stepper (feed rate)
/// Used for calculating minimum period of timer interrupts
#define STEPPER_MAX_SPEED 5000
/// Default stepper microstep multiplier
#define DEFAULT_STEPPER_MICROSTEP 256

/// Total steps per revolution
#define TOTAL_STEPPER_STEPS STEPPER_STEPS* DEFAULT_STEPPER_MICROSTEP

/// Drive screw thread pitch (mm/rev)
#define Z_MOTOR_LEADSCREW_STEP 8

#define MM_TO_STEPS(x)                                                                             \
    (((double)(x) * (double)(TOTAL_STEPPER_STEPS)) / (double)(Z_MOTOR_LEADSCREW_STEP))
#define STEPS_TO_MM(x)                                                                             \
    ((double)(x) * (double)(Z_MOTOR_LEADSCREW_STEP) / (double)(TOTAL_STEPPER_STEPS))

/// Default Z-motor accel (mm/s2)
#define Z_MOTOR_ACCEL 10
/// Primary speed at which the motor will reach the limit switch (mm/min)
#define Z_MOTOR_PRIMARY_HOMING_SPEED 150
/// Speed ​​of precise aiming to the limit switch (mm/min)
#define Z_MOTOR_SECONDARY_HOMING_SPEED 60

/// Speed of the motor rising (mm/min)
#define Z_MOTOR_RISING_SPEED 45
/// The height to which the motor rises when homing (mm)
#define Z_MOTOR_RISING_HEIGHT 3

/// Value of the current-setting resistor on TMC2208
/// Used to calculate current
#define TMC2208_R_SENSE 0.11f

/// Timeout (ms) after which a packet that was not completely received is
/// considered invalid
#define COMMUNICATION_TIMEOUT 100

/// Z-endstop configuration
#define Z_ENDSTOP_PIN 27
#define Z_ENDSTOP_POWER_PIN 21
#define Z_ENDSTOP_INVERSE true

/// z-dimension of printing space (mm)
#define Z_HEIGHT 165

/// Baud rate used for orchestrator connectivity
#define ORCHESTRATOR_SERIAL_BAUD_RATE 115200

/// Use hardware serial for orchestrator connectivity
// #define ORCHESTRATOR_HARDWARE_SERIAL

#ifdef ORCHESTRATOR_HARDWARE_SERIAL
/// Physical serial ID
#define PHYSICAL_SERIAL 1

#define ORCHESTRATOR_RX_PIN 26
#define ORCHESTRATOR_TX_PIN 25
#endif

// System fan connection pin (PWM)
#define SYSTEM_FAN_PIN 25

// System fan pwm freq
#define SYSTEM_FAN_FREQ 30