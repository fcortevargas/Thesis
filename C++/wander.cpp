#include <Arduino.h>
#include <MeMCore.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <stdlib.h>
#include <math.h>

// Timer control variables
double currentTime = 0;
double lastTime = 0;

// Arduino components
MeLineFollower linefollower_2(2);
MeDCMotor motor_9(9);
MeDCMotor motor_10(10);

// Wander control variables
double turnDuration;
double forwardDuration;
double lineTurnDuration;
double targetForwardSpeed;
double targetTurnSpeed;
int acceleration;
int wanderCycle;

// Boolean to determine if input is valid
boolean doWander;

// Example general input parameters
double duration = 10;
boolean stayInBounds = true;

// Example wander input parameters
double wanderSpeed = 100;
double wanderSlope = 0;
double wanderRoundness = 1;
double wanderTurnToForwardRatio = 0.9;
double wanderCycleRate = 2;
double wanderCycleStandardDeviation = 0.5;
double wanderSpeedStandardDeviation = 0.5;
double wanderPhase = 0;

// Built-in method to get current time
double getLastTime() 
{ 
  return currentTime = millis() / 1000.0 - lastTime; 
}

void _delay(double seconds) 
{
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime) _loop();
}

// Built-in method to move robot in certain direction
void move(int direction, int speed) 
{
  int leftSpeed = 0;
  int rightSpeed = 0;

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

  // Run left motor at given speed
  motor_9.run(9 == M1 ? -leftSpeed : leftSpeed);

  // Run right motor at given speed
  motor_10.run(10 == M1 ? -rightSpeed : rightSpeed);
}

// Helper method to generate a random standard gaussian number
double GenerateGaussian(double standardDeviation) 
{
    double u1 = rand() / (RAND_MAX + 1.0);
    double u2 = rand() / (RAND_MAX + 1.0);
    double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    return z0 * standardDeviation;
}

double GetRandomNumber(double min, double max)
{
    double randomNumber = (double)rand() / RAND_MAX; // Generate a random value between 0 and 1
    return (randomNumber * (max - min)) + min; // Scale and shift to the specified range
}

void CapNumber(double* number, double lowerLimit, double upperLimit)
{
    // Cap number to lower limit
    if (*number < lowerLimit) {
        *number = lowerLimit;
    }

    // Cap number to upper limit
    if (*number > upperLimit) {
        *number = upperLimit;
    }
}

void CheckValidWanderInput(double speed, double slope, double roundness, double turnToForwardRatio, double cycleRate, double cycleStandardDeviation, double speedStandardDeviation, double phase)
{
  boolean isValidSpeed = speed >= 25 && speed <= 100;
  boolean isValidSlope = slope >= -5 && slope <= 5;
  boolean isValidRoundness = roundness >= 0 && roundness <= 1;
  boolean isValidTurnToForwardRatio = turnToForwardRatio <= 1 && turnToForwardRatio > 0;
  boolean isCycleRatePositive = cycleRate > 0;
  boolean isValidStandardDeviation = cycleStandardDeviation >= 0 && speedStandardDeviation >= 0;
  boolean isValidPhase = phase >= 0;

  doWander = isValidSpeed && isValidSlope && isValidRoundness && isValidTurnToForwardRatio && isCycleRatePositive && isValidStandardDeviation && isValidPhase;
}

// Helper method to set the forward, turn, and line turn durations for the wander base behavior
void SetWanderDurations(double speed, double turnToForwardRatio, double cycleRate, double cycleStandardDeviation)
{
  // Set base forward duration based on input cycle rate
  forwardDuration = (1 - turnToForwardRatio) / cycleRate;

  // Add random normal variation to the base turn duration
  forwardDuration += GenerateGaussian(cycleStandardDeviation);

  // Cap the forward duration to be between zero and the inverse of the cycle rate in seconds
  CapNumber(&forwardDuration, 0, 1 / cycleRate);

  // Set turn duration
  turnDuration = 1 / cycleRate - forwardDuration;

  // Set line turn duration
  if (speed <= 100 && speed >= 50) {
    lineTurnDuration = 0.5;
  } else {
    lineTurnDuration = 0.0008 * pow(speed, 2) - 0.11 * speed + 4;
  }
}

// Helper method to set the target forward and turn speed for the wander base behavior
void SetTargetSpeeds(double speed, double roundness, double speedStandardDeviation)
{
  // Add random normal variation to the input wander speed to get the target forward speed
  targetForwardSpeed = speed + GenerateGaussian(speedStandardDeviation);

  // Cap target forward speed to be between 25 and 100
  CapNumber(&targetForwardSpeed, 25, 100);

  // Set target turn speed based on input roundness and target speed
  targetTurnSpeed = -targetForwardSpeed / 2 + 4 * targetForwardSpeed * roundness - 4 * targetForwardSpeed * pow(roundness, 2);
}

// Method to execute the wander base behavior
void Wander(double duration, boolean stayInBounds,
            double wanderSpeed, double wanderSlope, double wanderRoundness, double wanderTurnToForwardRatio, double wanderCycleRate, double wanderCycleStandardDeviation, double wanderSpeedStandardDeviation, double wanderPhase) 
{
  // INPUT CHECKS AND VARIABLE INITIALIZATION

  // Check if wander input is valid
  CheckValidWanderInput(wanderSpeed, wanderSlope, wanderRoundness, wanderTurnToForwardRatio, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase);

  // If wander input is valid, initialize wander variables
  if (doWander) {
    // Set the target forward and turning speeds based on given input
    SetTargetSpeeds(wanderSpeed, wanderRoundness, wanderSpeedStandardDeviation);
    
    // Set turn, line turn and forward durations based on given input
    SetWanderDurations(wanderSpeed, wanderTurnToForwardRatio, wanderCycleRate, wanderCycleStandardDeviation);

    // Variable used to determine if robot should turn left or right
    wanderCycle = 0;

    // Variable used to compute the acceleration of the forward segment 
    acceleration = 1; 
  }

  // MAIN LOOP

  // Initialize timer
  lastTime = millis() / 1000.0;

  // Loop during given duration 
  while (!(getLastTime() > duration)) {
    _loop();

    // If valid wander input is given
    if (doWander) {
      // Start executing after the given phase
      if (getLastTime() - wanderPhase >= 0) {
        // Turn around if a black line is detected
        if (stayInBounds) {
          // Get sensor reading
          int sensorReading = linefollower_2.readSensors();

          // Turn left if right sensor detects a black line
          if ((0 ? (1 == 0 ? sensorReading == 0 : (sensorReading & 1) == 1)
                  : (1 == 0 ? sensorReading == 3 : (sensorReading & 1) == 0))) {
              move(3, targetForwardSpeed / 100.0 * 255);
              _delay(lineTurnDuration);
              move(3, 0);
          }

          // Turn right if left sensor detects a black line
          if ((0 ? (2 == 0 ? sensorReading == 0 : (sensorReading & 2) == 2)
                  : (2 == 0 ? sensorReading == 3 : (sensorReading & 2) == 0))) {
              move(4, targetForwardSpeed / 100.0 * 255);
              _delay(lineTurnDuration);
              move(4, 0);
          }
        }
      }

      // Move forward for given duration

      // Constant slope
      if (wanderSlope == 0) {
        if (getLastTime() - wanderPhase < wanderCycle / wanderCycleRate + forwardDuration) {
          move(1, targetForwardSpeed / 100.0 * 255);
        } 
      }

      // Rising slope
      if (wanderSlope > 0) {
        if (getLastTime() - wanderPhase < wanderCycle / wanderCycleRate + acceleration * forwardDuration / 100) {
          move(1, wanderSlope * acceleration * targetForwardSpeed / 100 / 100.0 * 255);
        } else {
          if (acceleration < 100) {
            acceleration += 1;
          }
        }
      }

      // Falling slope
      if (wanderSlope < 0) {
        if (getLastTime() - wanderPhase < wanderCycle / wanderCycleRate + acceleration * forwardDuration / 100) {
          move(1, wanderSlope * (acceleration - 100) * targetForwardSpeed / 100 / 100.0 * 255);
        } else {
          if (acceleration < 100) {
            acceleration += 1;
          }
        }
      }

      // Turn for given duration
      if (getLastTime() - wanderPhase > wanderCycle / wanderCycleRate + forwardDuration && 
          getLastTime() - wanderPhase < (wanderCycle + 1) / wanderCycleRate) {
        if (fmod(wanderCycle, 2) == 0) {
          // Turn right
          motor_9.run(-targetForwardSpeed / 100.0 * 255);
          motor_10.run(targetTurnSpeed / 100.0 * 255);
        } else {
          // Turn left
          motor_9.run(-targetTurnSpeed / 100.0 * 255);
          motor_10.run(targetForwardSpeed / 100.0 * 255);
        }
      }

      // After forward and turn cycle is finished
      if (getLastTime() - wanderPhase > (wanderCycle + 1) / wanderCycleRate) {
        // Increase cycle count by one
        wanderCycle += 1;

        // Reset to one
        acceleration = 1;

        // Change target forward and turning speeds
        SetTargetSpeeds(wanderSpeed, wanderRoundness, wanderSpeedStandardDeviation);

        // Change forward and turn durations
        SetWanderDurations(wanderSpeed, wanderTurnToForwardRatio, wanderCycleRate, wanderCycleStandardDeviation);
      }
    }
  }

  motor_9.run(0);
  motor_10.run(0);
}


void setup() 
{
  // Initialize random seed
  randomSeed(0);

  Wander(duration, stayInBounds, wanderSpeed, wanderSlope, wanderRoundness, wanderTurnToForwardRatio, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase);
}

void _loop() 
{
  
}

void loop() 
{
  _loop();
}