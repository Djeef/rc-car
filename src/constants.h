#ifndef CONSTANTS_H
#define CONSTANTS_H

// Number of channel and index in tab DIRECTION, SPEED, RATIO
#define RC_NUM_CHANNELS 3
#define RC_CH1 0
#define RC_CH2 1
#define RC_CH3 2

// RC channel INPUT PIN
#define RC_CH1_PIN A0
#define RC_CH2_PIN A1
#define RC_CH3_PIN A2

// Servo INPUT PIN
#define SERVO_PIN 10

// Motors OUTPUT PIN direction (forward or backward) and speed
#define MOTOR_R_DIR_PIN 7
#define MOTOR_L_DIR_PIN 8
#define MOTOR_R_SPEED_PIN 3
#define MOTOR_L_SPEED_PIN 11

// RC min, neutral and max value (theorical values)
#define RC_MIN 1000
#define RC_NEUTRAL 1500
#define RC_MAX 2000

// Direction in degree min, neutral and max (depending of the car structure)
#define ANGLE_MIN 60
#define ANGLE_NEUTRAL 90
#define ANGLE_MAX 120

// Speed PWM min and max (0% to 100% of speed)
#define SPEED_PWM_MIN 0
#define SPEED_PWM_MAX 255

// => Speed ratio CH2 100% to -100%
// If positive go forward, else go backward
//
// => Motor selection ratio CH3 100% to -100%
// MIN left side is set to -100% and right side to 100%
// NEUTRAL side right and left are set to 100%
// MIN right side is set to -100% and left side to 100%
#define RATIO_MIN -100
#define RATIO_NEUTRAL 0
#define RATIO_MAX 100

// Serial speed, set if is verbose = true
#define SERIAL_PORT_SPEED 57600
#define IS_VERBOSE

// Number of values in mean window filter (of each channels)
#define NB_VALUES 20

// Number of iterations without command accepted
#define HEALTH_IT 5

// Number of iterations for convergence
#define CONVERGENCE_IT (2 * NB_VALUES)

// Main loop delay 50Hz
#define DELAY_MAIN_LOOP 10

// Max value of uint8_t
#define MAX_VALUE_UINT8 255

#endif // CONSTANTS_H
