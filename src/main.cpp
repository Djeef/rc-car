#include <Arduino.h>
#include <Servo.h>
#include <EnableInterrupt.h>
#include "constants.h"

// Position of direction (in degree)
uint8_t pos;
// Direction of right side (0 backward, 1 forward)
uint8_t dirR;
// Direction of left side (0 backward, 1 forward)
uint8_t dirL;
// Speed of right side (PWM : 0 to 255)
uint8_t speedR;
// Speed of left side (PWM : 0 to 255)
uint8_t speedL;

// Ratio of right side (-100; 100)
short ratioR;
// Ratio of left side (-100; 100)
short ratioL;
// Raw speed (-100; 100)
short ratioSpeed;

// If true, RC signal is etablished
uint8_t rcHealth;
// Number of loop without signal SHARED
volatile uint8_t itHealthShared;

// Number of iteractions after rcHealth
uint8_t itConvergence;
// If true, filter is operational
uint8_t converged;

// Raw values from RC of each channels SHARED
volatile uint16_t rcRawsShared[RC_NUM_CHANNELS];
// Time at the rising edge of each channels
volatile uint32_t rcStarts[RC_NUM_CHANNELS];
// Current index in the window of each channels SHARED
volatile uint8_t rcIndexesShared[RC_NUM_CHANNELS];

// Sum of the values for each channels (in window defined by NB_VALUES_FILTER)
uint32_t rcSums[RC_NUM_CHANNELS];
// Mean of the values for each channels (in window defined by NB_VALUES_FILTER)
uint16_t rcMeans[RC_NUM_CHANNELS];
// Window, save NB_VALUES_FILTER values for each channels
uint16_t rcWindows[RC_NUM_CHANNELS][NB_VALUES];

// Filtred values from RC of each channels
uint16_t rcValues[RC_NUM_CHANNELS];
// Current index in the window of each channels
uint8_t rcIndexes[RC_NUM_CHANNELS];

// Direction servo
Servo servoDir;

// Calc input channel
// @param channel the channel
// @param inputPin associated pin
void calcInput(uint8_t channel, uint8_t inputPin) {
	// If is rising edge
	if (digitalRead(inputPin) == HIGH) {
		// Save current time of this channel
		rcStarts[channel] = micros();
	} else {
		// Compare the rising edge time and now (falling edge)
		rcRawsShared[channel] = (uint16_t)(micros() - rcStarts[channel]);

		// Index++ (circular index [0; NB_VALUES_FILTER - 1])
		rcIndexesShared[channel] = ((rcIndexesShared[channel] + 1) % NB_VALUES);
	}

	// Number of iterations without signal equal to 0
	itHealthShared = 0;
}

// Calc input channel 1 (direction)
void calcCh1() {
	calcInput(RC_CH1, RC_CH1_PIN);
}

// Calc input channel 2 (speed)
void calcCh2() {
	calcInput(RC_CH2, RC_CH2_PIN);
}

// Calc input channel 3 (ratio)
void calcCh3() {
	calcInput(RC_CH3, RC_CH3_PIN);
}

// Compute speed of each side
void computeSpeed() {
	// Security overflow : ratio side right
	if (ratioR > RATIO_MAX) {
		ratioR = RATIO_MAX;
	} else if (ratioR < RATIO_MIN) {
		ratioR = RATIO_MIN;
	}

	// Security overflow : ratio side left
	if (ratioL > RATIO_MAX) {
		ratioL = RATIO_MAX;
	} else if (ratioL < RATIO_MIN) {
		ratioL = RATIO_MIN;
	}

	// Security overflow : ratio speed
	if (ratioSpeed > RATIO_MAX) {
		ratioSpeed = RATIO_MAX;
	} else if (ratioSpeed < RATIO_MIN) {
		ratioSpeed = RATIO_MIN;
	}

	// rawSpeed and ratio converted in range [-1.0; 1.0])
	float totalRatioR = ((float)ratioSpeed / RATIO_MAX) * ((float)ratioR / RATIO_MAX);

	// Compute motor direction right side : depending of rawSpeedR
	if (totalRatioR <= 0.0) {
		dirR = LOW;
	} else {
		dirR = HIGH;
	}

	// rawSpeed and ratio converted in range [-1.0; 1.0])
	float totalRatioL = ((float)ratioSpeed / RATIO_MAX) * ((float)ratioL / RATIO_MAX);

	// Compute motor direction left side : depending of rawSpeedL
	if (totalRatioL <= 0.0) {
		dirL = LOW;
	} else {
		dirL = HIGH;
	}

	// Final speed is equal to SPEED_MAX * rawSpeed * ratio
	speedR = abs(SPEED_PWM_MAX * totalRatioR);
	speedL = abs(SPEED_PWM_MAX * totalRatioL);
}

// Get data from shared variable and compute filter
void getData() {
	// Turn interrupts off quickly while we take
	// local copies of the shared variables
	noInterrupts();
	memcpy(rcValues, (const void *)rcRawsShared, sizeof(rcRawsShared));
	memcpy(rcIndexes, (const void *)rcIndexesShared, sizeof(rcIndexesShared));
	interrupts();

	for(int channel = 0; channel < RC_NUM_CHANNELS; ++channel) {
		// Get previous value at current windowIndex for this channel
		// and store the new value
		uint16_t prevValue = rcWindows[channel][rcIndexes[channel]];
		if(prevValue != rcValues[channel]) {
			rcWindows[channel][rcIndexes[channel]] = rcValues[channel];

			// Sum is equal to the last sum + the diference between
			// the new value and the last one
			rcSums[channel] += int(rcValues[channel] - prevValue);
			rcMeans[channel] =  rcSums[channel] / NB_VALUES;
		}
	}
}

// Convert input signals to command
void getCommands() {
	if(rcHealth) {
		// Copy data from shared variables
		getData();

		// If RC connection is etablished and filtre converged
		if (converged) {
			pos = map(rcValues[RC_CH1], RC_MIN, RC_MAX, ANGLE_MIN, ANGLE_MAX);
			ratioSpeed = map(rcMeans[RC_CH2], RC_MIN, RC_MAX, RATIO_MIN, RATIO_MAX);
			ratioR = map(rcMeans[RC_CH3], RC_MAX, RC_NEUTRAL, RATIO_MIN, RATIO_MAX);
			ratioL = map(rcMeans[RC_CH3], RC_MIN, RC_NEUTRAL, RATIO_MIN, RATIO_MAX);

			// Compute speed
			computeSpeed();
		}
	}
}

void writeOutputs() {
	servoDir.write(pos);
	analogWrite(MOTOR_R_SPEED_PIN, speedR);
	analogWrite(MOTOR_L_SPEED_PIN, speedL);
	digitalWrite(MOTOR_R_DIR_PIN, dirR);
	digitalWrite(MOTOR_L_DIR_PIN, dirL);
}

// Write commands
void setOutputs() {
	if(converged) {
		writeOutputs();
	}
}

// If is verbose
#ifdef IS_VERBOSE
// Send data through the serial port
void verboseMode() {
	Serial.print("RC:");
	Serial.print(rcHealth);
	Serial.print("\t");

	Serial.print("Pos:");
	Serial.print(pos);
	Serial.print("\t");

	Serial.print("Speed:");
	Serial.print(ratioSpeed);
	Serial.print("\t");

	Serial.print("DirR:");
	Serial.print(dirR);
	Serial.print("\t");

	Serial.print("DirL:");
	Serial.print(dirL);
	Serial.print("\t");

	Serial.print("SpeedR:");
	Serial.print(speedR);
	Serial.print("\t");

	Serial.print("SpeedL:");
	Serial.print(speedL);
	Serial.print("\n");
}
#endif // IS_VERBOSE

// Initialise all value
void initialise() {
	rcHealth = false;
	converged = false;
	itConvergence = 0;
	itHealthShared = MAX_VALUE_UINT8;

	// Set neutral values
	pos = ANGLE_NEUTRAL;
	ratioSpeed = RATIO_NEUTRAL;
	ratioR = RATIO_NEUTRAL;
	ratioL = RATIO_NEUTRAL;

	// Compute speed
	computeSpeed();

	// Write values
	writeOutputs();
}

// Check RC health status and convergence of the filter
void checkRcHealth() {
	// Turn interrupts off quickly
	// Get iterations
	noInterrupts();
	uint8_t iterations = itHealthShared;
	interrupts();

	// If RC active
	if(rcHealth) {
		// If the number of iterations without signal is reached
		if (iterations > HEALTH_IT) {
			initialise();

		} else {
			// Turn interrupts off quickly
			// Iterations +1 (is set to 0 when RC send data)
			noInterrupts();
			++itHealthShared;
			interrupts();

			// If not converged
			if(!converged) {
				// If number of iteration of convergence is reached : converged
				if(itConvergence > CONVERGENCE_IT) {
					converged = true;
				} else {
					++itConvergence;
				}
			}
		}
	} else if(iterations <= HEALTH_IT) {
		rcHealth = true;
	}
}

// Setup
void setup() {
// If is verbose
#ifdef IS_VERBOSE
	// If is verbose, then init serial port
	Serial.begin(SERIAL_PORT_SPEED);
#endif // IS_VERBOSE

	// Set RC channel pins to INPUT
	pinMode(RC_CH1_PIN, INPUT);
	pinMode(RC_CH2_PIN, INPUT);
	pinMode(RC_CH3_PIN, INPUT);

	// Set motors left and right pins to OUTPUT
	pinMode(MOTOR_R_DIR_PIN, OUTPUT);
	pinMode(MOTOR_L_DIR_PIN, OUTPUT);
	pinMode(MOTOR_R_SPEED_PIN, OUTPUT);
	pinMode(MOTOR_L_SPEED_PIN, OUTPUT);

	// Attach direction servo
	servoDir.attach(SERVO_PIN);

	// Set all default values
	initialise();

	// Enable interrupt for each channel, for each edges (rising and falling)
	enableInterrupt(RC_CH1_PIN, calcCh1, CHANGE);
	enableInterrupt(RC_CH2_PIN, calcCh2, CHANGE);
	enableInterrupt(RC_CH3_PIN, calcCh3, CHANGE);
}

// Main loop
void loop() {
	// Check RC health status
	checkRcHealth();
	// Get commands
	getCommands();
	// Set output
	setOutputs();

	// Wait delay
	delay(DELAY_MAIN_LOOP);

// If is verbose
#ifdef IS_VERBOSE
	// If is verbose, then send data through the serial port
	verboseMode();
#endif // IS_VERBOSE
}
