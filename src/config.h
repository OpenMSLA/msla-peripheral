
/// Pin to which the UV lamp control relay is connected
#define UV_RELAY_PIN 14

/// Number of steps per one revolution
#define STEPPER_STEPS 200

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
#define DEFAULT_STEPPER_MICROSTEP 16

/// Total steps per revolution
#define TOTAL_STEPPER_STEPS STEPPER_STEPS *DEFAULT_STEPPER_MICROSTEP

/// Value of the current-setting resistor on TMC2208
/// Used to calculate current
#define TMC2208_R_SENSE 0.11f

/// Timeout (ms) after which a packet that was not completely received is
/// considered invalid
#define COMMUNICATION_TIMEOUT 100

/// Z-endstop configuration
#define Z_ENDSTOP_PIN 27
#define Z_ENDSTOP_INVERSE true