#include <Arduino.h> // Include the main Arduino library for basic functions and macros
#include <MeMCore.h> // Include the Makeblock library for controlling Makeblock components
#include <SoftwareSerial.h> // Include the library for serial communication on digital pins
#include <Wire.h> // Include the library for I2C communication
#include <stdlib.h> // Include standard library for general purpose functions
#include <math.h> // Include math library for mathematical operations

// Timer control variables for managing time-based actions
double currentTime = 0; // Stores the current time in seconds
double lastTime = 0; // Stores the last time an action was taken in seconds

// Initialize components attached to the robot
MeLineFollower linefollower_2(2); // Line follower module on port 2
MeDCMotor motor_9(9); // DC motor connected to port 9
MeDCMotor motor_10(10); // DC motor connected to port 10

// Variables to control the wander behavior
double turnDuration; // Duration of the turning action in seconds
double forwardDuration; // Duration of the forward movement in seconds
double lineTurnDuration; // Duration of the turn when a line is detected
double targetForwardSpeed; // Desired speed for forward movement
double targetTurnSpeed; // Desired speed for turning
int acceleration; // Factor to increase/decrease speed gradually
int wanderCycle; // Counter for the number of wander cycles completed

// Flag to determine if the inputs for wander behavior are valid
boolean doWander;

// General input parameters for the wander behavior
double duration = 10; // Total duration of the wander behavior in seconds
boolean stayInBounds = true; // Flag to stay within a bounded area

// Specific input parameters for configuring the wander behavior
double wanderSpeed = 100; // Base speed for wandering
double wanderSlope = 0; // Slope for changing speed dynamically
double wanderRoundness = 0.5; // Factor for adjusting the sharpness of turns
double wanderTurnToForwardRatio = 0.9; // Ratio of turn duration to forward duration within a cycle
double wanderCycleRate = 2; // Number of cycles per second
double wanderCycleStandardDeviation = 0.5; // Variability in the cycle rate
double wanderSpeedStandardDeviation = 0.5; // Variability in the speed
double wanderPhase = 0; // Initial phase delay before starting to wander

// Function to calculate the elapsed time since the last reset
double getLastTime() 
{ 
  // Update currentTime, calculate and return the elapsed time since last reset
  return currentTime = millis() / 1000.0 - lastTime; 
}

// Custom delay function to pause execution for a given number of seconds
void _delay(double seconds) 
{
  long endTime = millis() + seconds * 1000; // Calculate end time in milliseconds
  while (millis() < endTime) _loop(); // Loop until the end time is reached
}

// Function to control the robot's movement in a specified direction at a specified speed
void move(int direction, int speed) 
{
  int leftSpeed = 0; // Speed of the left motor
  int rightSpeed = 0; // Speed of the right motor

  // Determine the speed of each motor based on the desired direction of movement
  if (direction == 1) {
    // Forward
    leftSpeed = speed;
    rightSpeed = speed;
  } else if (direction == 2) {
    // Backward
    leftSpeed = -speed;
    rightSpeed = -speed;
  } else if (direction == 3) {
    // Left
    leftSpeed = -speed;
    rightSpeed = speed;
  } else if (direction == 4) {
    // Right
    leftSpeed = speed;
    rightSpeed = -speed;
  }

  // Apply the calculated speeds to the motors
  motor_9.run(9 == M1 ? -leftSpeed : leftSpeed); // Adjust direction based on motor configuration
  motor_10.run(10 == M1 ? -rightSpeed : rightSpeed);
}

// Generates a random number with a Gaussian distribution around a mean of 0
double GenerateGaussian(double standardDeviation) 
{
    double u1 = rand() / (RAND_MAX + 1.0);
    double u2 = rand() / (RAND_MAX + 1.0);
    double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    return z0 * standardDeviation;
}

// Generates a random number within a specified range [min, max]
double GetRandomNumber(double min, double max)
{
    double randomNumber = (double)rand() / RAND_MAX; // Generate a random value between 0 and 1
    return (randomNumber * (max - min)) + min; // Scale and shift to the specified range
}

// Caps a number within a specified range [lowerLimit, upperLimit]
void CapNumber(double* number, double lowerLimit, double upperLimit)
{
    if (*number < lowerLimit) {
        *number = lowerLimit; // Cap to lower limit
    }
    if (*number > upperLimit) {
        *number = upperLimit; // Cap to upper limit
    }
}

// Checks whether the input parameters for wander behavior are within valid ranges
void CheckValidWanderInput(double speed, double slope, double roundness, double turnToForwardRatio, double cycleRate, double cycleStandardDeviation, double speedStandardDeviation, double phase)
{
  // Validate each input parameter against its acceptable range
  boolean isValidSpeed = speed >= 25 && speed <= 100;
  boolean isValidSlope = slope >= -5 && slope <= 5;
  boolean isValidRoundness = roundness >= 0 && roundness <= 1;
  boolean isValidTurnToForwardRatio = turnToForwardRatio <= 1 && turnToForwardRatio > 0;
  boolean isCycleRatePositive = cycleRate > 0;
  boolean isValidStandardDeviation = cycleStandardDeviation >= 0 && speedStandardDeviation >= 0;
  boolean isValidPhase = phase >= 0;

  // Set doWander to true only if all parameters are valid
  doWander = isValidSpeed && isValidSlope && isValidRoundness && isValidTurnToForwardRatio && isCycleRatePositive && isValidStandardDeviation && isValidPhase;
}

// Sets the durations for forward movement, turning, and line avoidance turning
void SetWanderDurations(double speed, double turnToForwardRatio, double cycleRate, double cycleStandardDeviation)
{
  forwardDuration = (1 - turnToForwardRatio) / cycleRate; // Calculate base forward duration
  forwardDuration += GenerateGaussian(cycleStandardDeviation); // Add variability
  CapNumber(&forwardDuration, 0, 1 / cycleRate); // Ensure duration is within bounds

  turnDuration = 1 / cycleRate - forwardDuration; // Calculate turn duration based on remaining time in the cycle

  // Calculate line turn duration based on speed
  if (speed <= 100 && speed >= 50) {
    lineTurnDuration = 0.5;
  } else {
    lineTurnDuration = 0.0008 * pow(speed, 2) - 0.11 * speed + 4;
  }
}

// Sets the target speeds for forward movement and turning
void SetTargetSpeeds(double speed, double roundness, double speedStandardDeviation)
{
  targetForwardSpeed = speed + GenerateGaussian(speedStandardDeviation); // Add variability to forward speed
  CapNumber(&targetForwardSpeed, 25, 100); // Ensure speed is within bounds

  // Calculate target turn speed based on forward speed and roundness
  targetTurnSpeed = -targetForwardSpeed / 2 + 4 * targetForwardSpeed * roundness - 4 * targetForwardSpeed * pow(roundness, 2);
}

// Method to execute the wander base behavior
void Wander(double duration, boolean stayInBounds,
            double wanderSpeed, double wanderSlope, double wanderRoundness, double wanderTurnToForwardRatio, double wanderCycleRate, double wanderCycleStandardDeviation, double wanderSpeedStandardDeviation, double wanderPhase) 
{
  // Validates the input parameters for the wander behavior.
  CheckValidWanderInput(wanderSpeed, wanderSlope, wanderRoundness, wanderTurnToForwardRatio, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase);

  // Proceeds with the wander behavior if the inputs are validated successfully.
  if (doWander) {
    // Calculates and sets target speeds for both forward movement and turning based on inputs and variability.
    SetTargetSpeeds(wanderSpeed, wanderRoundness, wanderSpeedStandardDeviation);
    
    // Determines durations for forward movement, turning, and line-avoidance based on inputs and variability.
    SetWanderDurations(wanderSpeed, wanderTurnToForwardRatio, wanderCycleRate, wanderCycleStandardDeviation);

    // Initializes the counter to keep track of completed wander cycles.
    wanderCycle = 0;

    // Sets the initial acceleration for speed adjustment during forward movement, used when slope != 0.
    acceleration = 1; 
  }

  // Records the start time of the wander behavior.
  lastTime = millis() / 1000.0;

  // Continues executing the wander behavior for the specified duration.
  while (!(getLastTime() > duration)) {
    _loop();

    // Checks if wander behavior is still valid and should continue.
    if (doWander) {
      // Delay start based on the wanderPhase parameter to allow for timed initiation of wandering.
      if (getLastTime() - wanderPhase >= 0) {
        // Enforces boundary constraints if required.
        if (stayInBounds) {
          // Reads sensor values to detect boundaries (e.g., lines on the floor).
          int sensorReading = linefollower_2.readSensors();

          // Executes avoidance maneuver if a black line is detected by the right sensor.
          if ((0 ? (1 == 0 ? sensorReading == 0 : (sensorReading & 1) == 1)
                 : (1 == 0 ? sensorReading == 3 : (sensorReading & 1) == 0))) { // Condition checks if the first bit is set, indicating the right sensor detects a line.
              move(3, targetForwardSpeed / 100.0 * 255); // Turns left to avoid crossing the line.
              _delay(lineTurnDuration); // Waits for the duration of the turn.
              move(3, 0); // Stops the turn.
          }

          // Executes avoidance maneuver if a black line is detected by the left sensor.
          if ((0 ? (2 == 0 ? sensorReading == 0 : (sensorReading & 2) == 2)
                 : (2 == 0 ? sensorReading == 3 : (sensorReading & 2) == 0))) { // Condition checks if the second bit is set, indicating the left sensor detects a line.
              move(4, targetForwardSpeed / 100.0 * 255); // Turns right to avoid crossing the line.
              _delay(lineTurnDuration); // Waits for the duration of the turn.
              move(4, 0); // Stops the turn.
          }
        }
      }

      // Determines if it's time to move forward based on the current phase and cycle rate.
      if (wanderSlope == 0) {
        // Moves forward at a constant speed if there's no slope.
        if (getLastTime() - wanderPhase < wanderCycle / wanderCycleRate + forwardDuration) {
          move(1, targetForwardSpeed / 100.0 * 255);
        } 
      } else if (wanderSlope > 0) {
        // Gradually increases speed if the slope is positive.
        if (getLastTime() - wanderPhase < wanderCycle / wanderCycleRate + acceleration * forwardDuration / 100) {
          move(1, wanderSlope * acceleration * targetForwardSpeed / 100 / 100.0 * 255);
          if (acceleration < 100) {
            acceleration += 1; // Increment acceleration until it reaches 100.
          }
        }
      } else if (wanderSlope < 0) {
        // Gradually decreases speed if the slope is negative.
        if (getLastTime() - wanderPhase < wanderCycle / wanderCycleRate + acceleration * forwardDuration / 100) {
          move(1, wanderSlope * (acceleration - 100) * targetForwardSpeed / 100 / 100.0 * 255);
          if (acceleration < 100) {
            acceleration += 1; // Increment acceleration until it reaches 100, for decreasing speed.
          }
        }
      }

      // Initiates a turn after the forward movement phase is complete.
      if (getLastTime() - wanderPhase > wanderCycle / wanderCycleRate + forwardDuration && 
          getLastTime() - wanderPhase < (wanderCycle + 1) / wanderCycleRate) {
        // Alternates turning direction with each cycle.
        if (fmod(wanderCycle, 2) == 0) {
          // Executes a right turn on even cycles.
          motor_9.run(-targetForwardSpeed / 100.0 * 255);
          motor_10.run(targetTurnSpeed / 100.0 * 255);
        } else {
          // Executes a left turn on odd cycles.
          motor_9.run(-targetTurnSpeed / 100.0 * 255);
          motor_10.run(targetForwardSpeed / 100.0 * 255);
        }
      }

      // Prepares for the next cycle after completing both forward movement and turning.
      if (getLastTime() - wanderPhase > (wanderCycle + 1) / wanderCycleRate) {
        wanderCycle += 1; // Increments the cycle counter.
        acceleration = 1; // Resets acceleration for the next cycle.

        // Recalculates speeds and durations for the next cycle, incorporating variability.
        SetTargetSpeeds(wanderSpeed, wanderRoundness, wanderSpeedStandardDeviation);
        SetWanderDurations(wanderSpeed, wanderTurnToForwardRatio, wanderCycleRate, wanderCycleStandardDeviation);
      }
    }
  }

  // Stops the motors to halt the robot's movement at the end of the wander behavior.
  motor_9.run(0);
  motor_10.run(0);
}

// Setup function to initialize the robot and start the wander behavior
void setup() 
{
  randomSeed(0); // Initialize the random number generator seed.

  // Start the wandering behavior with the specified parameters.
  Wander(duration, stayInBounds, wanderSpeed, wanderSlope, wanderRoundness, wanderTurnToForwardRatio, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase); // Start wandering
}

// Placeholder loop function, required for Arduino structure but not used
void _loop() 
{
  
}

// Main loop function, continuously called by Arduino framework
void loop() 
{
  _loop(); // Call the placeholder loop function
}