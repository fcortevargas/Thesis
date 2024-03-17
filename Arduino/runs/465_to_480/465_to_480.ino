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
MeRGBLed rgbled_7(7, 2);
MeBuzzer buzzer;
MeIR ir;

// Wander control variables
double turnDuration;
double forwardDuration;
double turnToForwardRatio;
double lineTurnDuration;
double targetForwardSpeed;
double targetTurnSpeed;
int acceleration;
int wanderCycle;

// Blink control variables
double lightsOnDuration;
double lightsOffDuration;
double targetRedIntensity;
double targetGreenIntensity;
double targetBlueIntensity;
int brightness;
int blinkCycle;

// Beep control variables
double soundDuration;
double silenceDuration;
double targetPitch;
double currentPitch;
double semitone;
int beepCycle;

// Booleans to determine if input is valid
boolean doWander;
boolean doBeep;
boolean doBlink;

// General input parameters
double duration;
boolean stayInBounds;

// Wander input parameters
double wanderSpeed;
double wanderSlope;
double wanderRoundness;
double wanderCycleRate;
double wanderCycleStandardDeviation;
double wanderSpeedStandardDeviation;
double wanderPhase;

// Blink input parameters
double blinkTemperature;
double blinkSlope;
double blinkLightsOnToOffRatio;
double blinkCycleRate;
double blinkCycleStandardDeviation;
double blinkTemperatureStandardDeviation;
double blinkPhase;

// Beep input parameters
double beepPitch;
double beepSlope;
double beepSoundToSilenceRatio;
double beepCycleRate;
double beepCycleStandardDeviation;
double beepPitchStandardDeviation;
double beepRandomSoundProbability;
double beepPhase;

// Variable to keep track of how many times the button is pressed
int timesButtonPressed = 0;

// Built-in method to get current time
double getLastTime() 
{ 
  return currentTime = millis() / 1000.0 - lastTime; 
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

void SetTurnToForwardRatio(double roundness, double cycleRate)
{
  // Calculate turn to forward ratio based on roundness and cycle rate
  if (roundness >= 0.5) {
    turnToForwardRatio = 0.9;
  } else {
    turnToForwardRatio = 0.245455 + 0.109091 * cycleRate - 0.6 * roundness;
  }
}

// Sets the durations for forward movement, turning, and line avoidance turning
void SetWanderDurations(double speed, double roundness, double cycleRate, double cycleStandardDeviation)
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

// Helper method to set the lights on and off durations for the blink base behavior
void SetBlinkDurations(double lightsOnToOffRatio, double cycleRate, double cycleStandardDeviation)
{
  // Set lights off duration based on input cycle rate
  lightsOffDuration = (1 - lightsOnToOffRatio) / cycleRate;

  // Add random normal variation to the base lights off duration
  lightsOffDuration += GenerateGaussian(cycleStandardDeviation);

  // Cap the lights off duration to be between zero and the inverse of the cycle rate in seconds
  CapNumber(&lightsOffDuration, 0, 1 / cycleRate);

  // Set lights on duration
  lightsOnDuration = 1 / cycleRate - lightsOffDuration;
}

// Helper method to set the sound and silence durations for the blink base behavior
void SetBeepDurations(double soundToSilenceRatio, double cycleRate, double cycleStandardDeviation)
{
  // Set silence duration based on input cycle rate
  silenceDuration = (1 - soundToSilenceRatio) / cycleRate;

  // Add random normal variation to the base lights off duration
  silenceDuration += GenerateGaussian(cycleStandardDeviation);

  // Cap the silence duration to be between zero and the inverse of the cycle rate in seconds
  CapNumber(&silenceDuration, 0, 1 / cycleRate);

  // Set sound duration
  soundDuration = 1 / cycleRate - silenceDuration;
}

// Sets the target speeds for forward movement and turning
void SetTargetSpeeds(double speed, double roundness, double speedStandardDeviation)
{
  targetForwardSpeed = speed + GenerateGaussian(speedStandardDeviation); // Add variability to forward speed
  CapNumber(&targetForwardSpeed, 20, 100); // Ensure speed is within bounds

  // Calculate target turn speed based on forward speed and roundness
  if (roundness >= 0.5) { 
    targetTurnSpeed = -1.6 * targetForwardSpeed * roundness + 1.8 * targetForwardSpeed;
  } else {
    targetTurnSpeed = 1.6 * targetForwardSpeed * roundness - targetForwardSpeed;
  }
}

// Helper method to set the target red, blue and green intensity for the blink base behavior
void SetTargetIntensities(double temperature, double temperatureStandardDeviation)
{
  // Add random normal variation to the input blink temperature to get the target temperature
  double targetTemperature = temperature + GenerateGaussian(temperatureStandardDeviation);

  // Cap target temperature to be between 0 and 1
  CapNumber(&targetTemperature, 0, 1);

  // If target temperature is neutral, choose a greenish yellow.
  if (targetTemperature == 0.5) {
    targetRedIntensity = 200;
    targetGreenIntensity = 255;
    targetBlueIntensity = 0;
  }

  // If target temperature is higher than 0.5, choose warm colors
  if (targetTemperature > 0.5) {
    targetRedIntensity = round(110 * targetTemperature + 145);
    targetGreenIntensity = round(-510 * targetTemperature + 510);
    targetBlueIntensity = 0;
  }

  // If target temperature is lower than 0.5, choose cool colors
  if (targetTemperature < 0.5) {
    targetRedIntensity = round(100 * targetTemperature);
    targetGreenIntensity = round(510 * targetTemperature);
    targetBlueIntensity = round(-510 * targetTemperature + 255);
  }
}

// Helper method to set the target pitch for the beep base behavior
void SetTargetPitch(double pitch, double pitchStandardDeviation)
{
  // Add random normal variation to the input beep pitch to get the target pitch
  targetPitch = pitch + GenerateGaussian(pitchStandardDeviation);

  // Cap target pitch between 80 Hz and 3000 Hz
  CapNumber(&targetPitch, 80, 3000);

  // Initialize current pitch to target pitch
  currentPitch = targetPitch;
}

// Helper function used to play a random sound based on a given probability
void PlayRandomSoundWithProbability(double slope, double randomSoundProbability, double pitchStandardDeviation)
{
  // Generate a random uniformly distributed number between 0 and 1
  double randomNumber = GetRandomNumber(0, 1);

  // Initialize variables for the random pitch and semitone
  double randomPitch;
  int randomSemitone;

  if (slope == 0) {
    // Add random normal variation to the target beep pitch
    randomPitch = targetPitch + GenerateGaussian(pitchStandardDeviation);
  } else {
    // Generate a random semitone between 1 and 12
    randomSemitone = (int)GetRandomNumber(1, 12);

    // Get pitch of random semitone of the target pitch
    randomPitch = exp(log(targetPitch) + slope * randomSemitone / 12 * log(2));
  }

  // Compare random number with noiseProbability
  if (randomNumber < randomSoundProbability) {
    buzzer.tone(randomPitch, silenceDuration * 1000); // Play note for given duration if within probability
  } else {
    _delay(double(silenceDuration)); // Else play silence
  }
}

// Method to execute robot's base behaviors wander, blink and beep
void WanderBlinkBeep(double duration, boolean stayInBounds,
                     double wanderSpeed, double wanderSlope, double wanderRoundness, double wanderCycleRate, double wanderCycleStandardDeviation, double wanderSpeedStandardDeviation, double wanderPhase, 
                     double blinkTemperature, double blinkSlope, double blinkLightsOnToOffRatio, double blinkCycleRate, double blinkCycleStandardDeviation, double blinkTemperatureStandardDeviation, double blinkPhase,
                     double beepPitch, double beepSlope, double beepSoundToSilenceRatio, double beepCycleRate, double beepCycleStandardDeviation, double beepPitchStandardDeviation, double beepRandomSoundProbability, double beepPhase) 
{
  // INPUT CHECKS AND VARIABLE INITIALIZATION

  // Check if wander input is valid
  CheckValidWanderInput(wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase);

  // If wander input is valid, initialize wander variables
  if (doWander) {
    // Determines the turn to forward ratio to make sure the round movements look round, and the sharp movement look sharp.
    SetTurnToForwardRatio(wanderRoundness, wanderCycleRate);

    // Calculates and sets target speeds for both forward movement and turning based on inputs and variability.
    SetTargetSpeeds(wanderSpeed, wanderRoundness, wanderSpeedStandardDeviation);
    
    // Determines durations for forward movement, turning, and line-avoidance based on inputs and variability.
    SetWanderDurations(wanderSpeed, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation);

    // Initializes the counter to keep track of completed wander cycles.
    wanderCycle = 0;

    // Sets the initial acceleration for speed adjustment during forward movement, used when slope != 0.
    acceleration = 1; 
  }

  // Check if blink input is valid
  CheckValidBlinkInput(blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase);

  // If blink input is valid, initialize blink variables
  if (doBlink) {
    // Set the target red, green and blue light intensities based on given input
    SetTargetIntensities(blinkTemperature, blinkTemperatureStandardDeviation);

    // Set lights on and off durations based on given input
    SetBlinkDurations(blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation);

    // Variable used to control the blink cycles
    blinkCycle = 0;

    // Variable used to compute the brightness of the light 
    brightness = 1;
  }

  // Check if beep input is valid
  CheckValidBeepInput(beepPitch, beepSlope, beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

  if (doBeep) {
    // Set the target pitch based on given input
    SetTargetPitch(beepPitch, beepPitchStandardDeviation);

    // Set sound and silence durations based on given input
    SetBeepDurations(beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation);

    // Variable used to control the beep cycles
    beepCycle = 0;

    // Variable used to compute the pitch of each note
    semitone = 1;
  }

  // MAIN LOOP

  MagentaTransition(2);

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

          // Turn left if right sensor detects a white line
          if ((0 ? (1 == 0 ? sensorReading == 0 : (sensorReading & 1) == 1)
                  : (1 == 0 ? sensorReading == 3 : (sensorReading & 1) == 0))) {
              move(3, targetForwardSpeed / 100.0 * 255);
              _delay(lineTurnDuration);
              move(3, 0);
          }

          // Turn right if left sensor detects a white line
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
        wanderCycle += 1; // Increments the cycle counter.
        acceleration = 1; // Resets acceleration for the next cycle.

        // Recalculates speeds and durations for the next cycle, incorporating variability.
        SetTargetSpeeds(wanderSpeed, wanderRoundness, wanderSpeedStandardDeviation);
        SetWanderDurations(wanderSpeed, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation);
      }
    }

    // If valid blink input is given
    if (doBlink) {
      // Start executing after the given phase
      if (getLastTime() - blinkPhase >= 0) {
        // Turn lights on for given duration

        // Constant slope
        if (blinkSlope == 0) {
          if (getLastTime() - blinkPhase < blinkCycle / blinkCycleRate + lightsOnDuration) {
            rgbled_7.setColor(0, targetRedIntensity, targetGreenIntensity, targetBlueIntensity);
            rgbled_7.show();
          }
        }

        // Rising slope
        if (blinkSlope > 0) {
          if (getLastTime() - blinkPhase < blinkCycle / blinkCycleRate + brightness * lightsOnDuration / 100) {
            rgbled_7.setColor(0, round(blinkSlope * brightness * targetRedIntensity / 100),
                                 round(blinkSlope * brightness * targetGreenIntensity / 100),
                                 round(blinkSlope * brightness * targetBlueIntensity / 100));
            rgbled_7.show();
          } else {
            if (brightness < 100) {
              brightness += 1;
            }
          }
        }

        // Falling slope
        if (blinkSlope < 0) {
          if (getLastTime() - blinkPhase < blinkCycle / blinkCycleRate + brightness * lightsOnDuration / 100) {
            rgbled_7.setColor(0, round(blinkSlope * (brightness - 100) * targetRedIntensity / 100),
                                 round(blinkSlope * (brightness - 100) * targetGreenIntensity / 100),
                                 round(blinkSlope * (brightness - 100) * targetBlueIntensity / 100));
            rgbled_7.show();
          } else {
            if (brightness < 100) {
              brightness += 1;
            }
          }
        }

        // Turn lights off for given duration
        if (getLastTime() - blinkPhase > blinkCycle / blinkCycleRate + lightsOnDuration && 
            getLastTime() - blinkPhase < (blinkCycle + 1) / blinkCycleRate) {
          rgbled_7.setColor(0, 0, 0, 0);
          rgbled_7.show();
        }

        // If blink cycle has finished
        if (getLastTime() - blinkPhase > (blinkCycle + 1) / blinkCycleRate) {
          // Increase count of cycles by one
          blinkCycle += 1;

          // Reset to one
          brightness = 1;

          // Change target red, green and blue light intensities
          SetTargetIntensities(blinkTemperature, blinkTemperatureStandardDeviation);

          // Change lights on and off durations
          SetBlinkDurations(blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation);
        }
      }
    }

    // If valid beep input is given
    if (doBeep) {
      // Start executing after the given phase
      if (getLastTime() - beepPhase >= 0) {
        if (beepSlope == 0) {
          if (getLastTime() - beepPhase < beepCycle / beepCycleRate + soundDuration) {
            // Play the sound at the current pitch for the given duration
            buzzer.tone(currentPitch, soundDuration * 1000);
          }
        }

        if (beepSlope > 0) {
          if(getLastTime() - beepPhase < beepCycle / beepCycleRate + semitone * soundDuration / 12) {
            // Play the sound at the current pitch for the given duration
            buzzer.tone(currentPitch, soundDuration / 12 * 1000);
          } else {
            // Change current pitch to next rising semitone
            currentPitch = exp(log(targetPitch) + beepSlope * semitone / 12 * log(2));
            
            // Increase semitone by one
            if (semitone < 12) {
              semitone += 1;
            }
          }

        }

        if (beepSlope < 0) {
          if (getLastTime() - beepPhase < beepCycle / beepCycleRate + semitone * soundDuration / 12) {
            // Play the sound at the current pitch for the given duration
            buzzer.tone(currentPitch, soundDuration / 12 * 1000);
          } else {
            // Change current pitch to next falling semitone
            currentPitch = exp(log(targetPitch) + beepSlope * semitone / 12 * log(2));
            
            // Increase semitone by one
            if (semitone < 12) {
              semitone += 1;
            }
          }
        }
        
        // Play either a random note or silence based on the given probability for the silence duration
        if (getLastTime() - beepPhase > beepCycle / beepCycleRate + soundDuration && 
            getLastTime() - beepPhase < (beepCycle + 1) / beepCycleRate) {
          PlayRandomSoundWithProbability(beepSlope, beepRandomSoundProbability, beepPitchStandardDeviation);
        }

        if (getLastTime() - beepPhase > (beepCycle + 1) / beepCycleRate) {
          // Increase count of cycles by one
          beepCycle += 1;

          // Reset to one
          semitone = 1;

          // Change  the target pitch
          SetTargetPitch(beepPitch, beepPitchStandardDeviation);

          // Change sound and silence durations
          SetBeepDurations(beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation);
        }
      }
    }
  }

  motor_9.run(0);
  motor_10.run(0);
}

// Checks whether the input parameters for wander behavior are within valid ranges
void CheckValidWanderInput(double speed, double slope, double roundness, double cycleRate, double cycleStandardDeviation, double speedStandardDeviation, double phase)
{
  // Validate each input parameter against its acceptable range
  boolean isValidSpeed = speed >= 20 && speed <= 100;
  boolean isValidSlope = slope >= -5 && slope <= 5;
  boolean isValidRoundness = roundness >= 0 && roundness <= 1;
  boolean isCycleRatePositive = cycleRate > 0;
  boolean isValidStandardDeviation = cycleStandardDeviation >= 0 && speedStandardDeviation >= 0;
  boolean isValidPhase = phase >= 0;

  // Set doWander to true only if all parameters are valid
  doWander = isValidSpeed && isValidSlope && isValidRoundness && isCycleRatePositive && isValidStandardDeviation && isValidPhase;
}

void CheckValidBlinkInput(double temperature, double slope, double lightsOnToOffRatio, double cycleRate, double cycleStandardDeviation, double temperatureStandardDeviation, double phase) 
{
  boolean isValidTemperature = temperature >= 0 && temperature <= 1;
  boolean isValidSlope = slope >= -5 && slope <= 5;
  boolean isValidLightsOnToOffRatio = lightsOnToOffRatio <= 1 && lightsOnToOffRatio > 0;
  boolean isCycleRatePositive = cycleRate > 0;
  boolean isValidStandardDeviation = cycleStandardDeviation >= 0 && temperatureStandardDeviation >= 0;
  boolean isValidPhase = phase >= 0;

  doBlink = isValidTemperature && isValidSlope && isValidLightsOnToOffRatio && isCycleRatePositive && isValidStandardDeviation && isValidPhase;
}

void CheckValidBeepInput(double pitch, double slope, double soundToSilenceRatio, double cycleRate, double cycleStandardDeviation, double pitchStandardDeviation, double randomSoundProbability, double phase) 
{
  boolean isValidPitch = pitch >= 80 && pitch <= 3000;
  boolean isValidSlope = (slope < 0 && slope >= log(40 / pitch) / log(2)) || (slope > 0 && slope <= log(6000 / pitch) / log(2)) || slope == 0;
  boolean isValidSoundToSilenceRatio = soundToSilenceRatio <= 1 && soundToSilenceRatio >= 0;
  boolean isCycleRatePositive = cycleRate > 0;
  boolean isValidStandardDeviation = cycleStandardDeviation >= 0 && pitchStandardDeviation >= 0;
  boolean isValidRandomSoundProbability = randomSoundProbability <= 1 && randomSoundProbability >= 0;
  boolean isValidPhase = phase >= 0;

  doBeep = isValidPitch && isValidSlope && isValidSoundToSilenceRatio && isCycleRatePositive && isValidStandardDeviation && isValidRandomSoundProbability && isValidPhase;
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

void _delay(double seconds) 
{
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime) _loop();
}

void MagentaTransition(double duration)
{
  rgbled_7.setColor(0, 255, 0, 255);
  rgbled_7.show();
  _delay(duration);
  rgbled_7.setColor(0, 0, 0, 0);
  rgbled_7.show();
}

void setup() 
{
  // Initialize remote control signaling
  ir.begin();

  rgbled_7.fillPixelsBak(0, 2, 1);

  // Initialize random seed
  randomSeed(0);

  while (true) {
    // If right key is pressed on the remote control
    if (ir.keyPressed(9)) {

      // CONSTANT GENERAL INPUT PARAMETERS FOR ALL EMOTIONS
      duration = 20;
      stayInBounds = true;

      // CONSTANT BASE BEHAVIORS INPUT PARAMETERS FOR ALL EMOTIONS
      wanderSlope = 0;
      wanderCycleStandardDeviation = 0.4;
      wanderSpeedStandardDeviation = 0;
      wanderPhase = 0;
      
      blinkLightsOnToOffRatio = 0.85;
      blinkCycleStandardDeviation = 0.325; 
      blinkTemperatureStandardDeviation = 0.025;
      blinkPhase = 0;

      beepCycleStandardDeviation = 0.2;
      beepSoundToSilenceRatio = 0.625;
      beepPitchStandardDeviation = 67.5;
      beepRandomSoundProbability = 0;
      beepPhase = 0;

      // run 465

      wanderSpeed = 83.67183213122189;
      wanderRoundness = 0.160992425866425;
      wanderCycleRate = 3.4402176127769053;
      blinkTemperature = 0.1743239276111126;
      blinkSlope = 1.0;
      blinkCycleRate = 5.078071024734527;
      beepPitch = 805.3288906812668;
      beepSlope = 0.0;
      beepCycleRate = 1.6089837476611135;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 466

      wanderSpeed = 43.41584254987538;
      wanderRoundness = 0.537783931940794;
      wanderCycleRate = 1.6764164534397423;
      blinkTemperature = 0.8227958790957928;
      blinkSlope = -0.0;
      blinkCycleRate = 3.0694526927545667;
      beepPitch = 446.29347138106823;
      beepSlope = -1.0;
      beepCycleRate = 4.6472696349956095;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 467

      wanderSpeed = 49.40875777974725;
      wanderRoundness = 0.4204469202086329;
      wanderCycleRate = 4.809580514207482;
      blinkTemperature = 0.4475710168480873;
      blinkSlope = -1.0;
      blinkCycleRate = 1.6324044754728677;
      beepPitch = 687.1659813448787;
      beepSlope = -0.0;
      beepCycleRate = 4.262798743788153;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 468

      wanderSpeed = 79.11871993914247;
      wanderRoundness = 0.7938319016247988;
      wanderCycleRate = 3.046922728419304;
      blinkTemperature = 0.5491751208901405;
      blinkSlope = 0.0;
      blinkCycleRate = 3.727636980358511;
      beepPitch = 146.20043504983187;
      beepSlope = 1.0;
      beepCycleRate = 2.5130901746451855;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 469

      wanderSpeed = 66.11174595542252;
      wanderRoundness = 0.3547184141352772;
      wanderCycleRate = 1.1511452635750177;
      blinkTemperature = 0.6851394129917026;
      blinkSlope = 0.0;
      blinkCycleRate = 5.723419256974012;
      beepPitch = 581.4032671041787;
      beepSlope = -0.0;
      beepCycleRate = 3.3253194405697286;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 470

      wanderSpeed = 60.97698119468987;
      wanderRoundness = 0.977846797555685;
      wanderCycleRate = 3.94468563888222;
      blinkTemperature = 0.3179925633594394;
      blinkSlope = -1.0;
      blinkCycleRate = 2.254865095950663;
      beepPitch = 222.36875304952264;
      beepSlope = 1.0;
      beepCycleRate = 2.929590758867562;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 471

      wanderSpeed = 31.814510691910986;
      wanderRoundness = 0.095130386762321;
      wanderCycleRate = 2.5198026127181947;
      blinkTemperature = 0.9426454594358802;
      blinkSlope = -0.0;
      blinkCycleRate = 0.8977695954963565;
      beepPitch = 913.136978726834;
      beepSlope = 0.0;
      beepCycleRate = 1.1709991777315736;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 472

      wanderSpeed = 96.71405287459493;
      wanderRoundness = 0.7216881830245256;
      wanderCycleRate = 5.3145494679920375;
      blinkTemperature = 0.0598683645948767;
      blinkSlope = 1.0;
      blinkCycleRate = 4.282395212445408;
      beepPitch = 372.17224380001426;
      beepSlope = -1.0;
      beepCycleRate = 5.60623357957229;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 473

      wanderSpeed = 94.38790833577514;
      wanderRoundness = 0.4592944169417023;
      wanderCycleRate = 2.151872866321355;
      blinkTemperature = 0.2771706515923142;
      blinkSlope = -0.0;
      blinkCycleRate = 3.3560867593623698;
      beepPitch = 650.3401182591915;
      beepSlope = -0.0;
      beepCycleRate = 3.896376236807555;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 474

      wanderSpeed = 38.23796143755317;
      wanderRoundness = 0.8322548940777779;
      wanderCycleRate = 5.720057221595198;
      blinkTemperature = 0.7225052053108811;
      blinkSlope = 1.0;
      blinkCycleRate = 1.3269909275695682;
      beepPitch = 292.18707866966724;
      beepSlope = 1.0;
      beepCycleRate = 2.831274955533445;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 475

      wanderSpeed = 58.92869316972792;
      wanderRoundness = 0.2148584043607115;
      wanderCycleRate = 0.772491667419672;
      blinkTemperature = 0.0966927083209157;
      blinkSlope = 0.0;
      blinkCycleRate = 2.6769634345546365;
      beepPitch = 954.9414550885558;
      beepSlope = 0.0;
      beepCycleRate = 0.557728941552341;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 476

      wanderSpeed = 72.81279453076422;
      wanderRoundness = 0.592194339260459;
      wanderCycleRate = 4.339469542726874;
      blinkTemperature = 0.9014036925509572;
      blinkSlope = -1.0;
      blinkCycleRate = 4.794016130734235;
      beepPitch = 413.0935290828347;
      beepSlope = -1.0;
      beepCycleRate = 5.660993169527501;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 477

      wanderSpeed = 76.79205803200603;
      wanderRoundness = 0.0250901216641068;
      wanderCycleRate = 5.193602581508458;
      blinkTemperature = 0.7702876459807158;
      blinkSlope = -1.0;
      blinkCycleRate = 3.997162687126547;
      beepPitch = 876.1329968459904;
      beepSlope = 0.0;
      beepCycleRate = 1.4891827907413244;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 478

      wanderSpeed = 55.83275927230716;
      wanderRoundness = 0.6522021219134331;
      wanderCycleRate = 2.657507295720279;
      blinkTemperature = 0.2312495838850736;
      blinkSlope = 0.0;
      blinkCycleRate = 0.5061171939596534;
      beepPitch = 517.9807552136481;
      beepSlope = -1.0;
      beepCycleRate = 5.239809945691377;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 479

      wanderSpeed = 41.367003777995706;
      wanderRoundness = 0.2694231336936354;
      wanderCycleRate = 3.813521075528115;
      blinkTemperature = 0.605559328570962;
      blinkSlope = 1.0;
      blinkCycleRate = 1.9479385511949656;
      beepPitch = 730.6171144358814;
      beepSlope = -0.0;
      beepCycleRate = 4.339043487329036;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 480

      wanderSpeed = 90.37339807488024;
      wanderRoundness = 0.8921215292066336;
      wanderCycleRate = 1.276282416190952;
      blinkTemperature = 0.3946429286152124;
      blinkSlope = -0.0;
      blinkCycleRate = 5.353712644893676;
      beepPitch = 188.77010708674788;
      beepSlope = 1.0;
      beepCycleRate = 1.878336424008012;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      MagentaTransition(2);
    }
    _loop();
  }
}

void _loop() 
{
  // Used for the remote control
  ir.loop();
}

void loop() 
{
  _loop();
}