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

      // run 337

      wanderSpeed = 84.41951716318727;
      wanderRoundness = 0.4489371478557586;
      wanderCycleRate = 4.708995615132153;
      blinkTemperature = 0.100563308224082;
      blinkSlope = -1.0;
      blinkCycleRate = 2.4287157519720495;
      beepPitch = 178.2523013651371;
      beepSlope = 0.0;
      beepCycleRate = 1.2260738387703896;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 338

      wanderSpeed = 43.770452830940485;
      wanderRoundness = 0.8221350917592645;
      wanderCycleRate = 3.2059861524030566;
      blinkTemperature = 0.9052158799022436;
      blinkSlope = 0.0;
      blinkCycleRate = 5.898948738351464;
      beepPitch = 762.2833821922541;
      beepSlope = -1.0;
      beepCycleRate = 5.040988935623318;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 339

      wanderSpeed = 49.06269242055714;
      wanderRoundness = 0.1893491726368665;
      wanderCycleRate = 3.3283138605766;
      blinkTemperature = 0.2794643510133028;
      blinkSlope = 1.0;
      blinkCycleRate = 4.461690712720156;
      beepPitch = 528.553276322782;
      beepSlope = -0.0;
      beepCycleRate = 4.5700914287008345;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 340

      wanderSpeed = 78.37957982905209;
      wanderRoundness = 0.5659765293821692;
      wanderCycleRate = 1.8265318586491048;
      blinkTemperature = 0.7247357238084078;
      blinkSlope = -0.0;
      blinkCycleRate = 1.0780718992464244;
      beepPitch = 844.5214444771409;
      beepSlope = 1.0;
      beepCycleRate = 2.2166409753262997;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 341

      wanderSpeed = 67.01323123648763;
      wanderRoundness = 0.0042355209589004;
      wanderCycleRate = 2.3366214116103947;
      blinkTemperature = 0.6034499118104577;
      blinkSlope = -0.0;
      blinkCycleRate = 3.2489182627759874;
      beepPitch = 402.5208001025021;
      beepSlope = -0.0;
      beepCycleRate = 3.633100913371891;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 342

      wanderSpeed = 61.177791226655245;
      wanderRoundness = 0.6310947993770242;
      wanderCycleRate = 5.5575414770282805;
      blinkTemperature = 0.3924722382798791;
      blinkSlope = 1.0;
      blinkCycleRate = 5.258544033393264;
      beepPitch = 986.5527861751616;
      beepSlope = 1.0;
      beepCycleRate = 2.6326180221512914;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 343

      wanderSpeed = 31.622245581820607;
      wanderRoundness = 0.2636900078505277;
      wanderCycleRate = 0.9553234167397022;
      blinkTemperature = 0.7668426083400846;
      blinkSlope = 0.0;
      blinkCycleRate = 3.901322647929192;
      beepPitch = 302.7046631090343;
      beepSlope = 0.0;
      beepCycleRate = 0.7886121505871415;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 344

      wanderSpeed = 95.82111251540482;
      wanderRoundness = 0.8871427616104484;
      wanderCycleRate = 4.177365874871612;
      blinkTemperature = 0.2277442263439297;
      blinkSlope = -1.0;
      blinkCycleRate = 1.8077683332376184;
      beepPitch = 618.6736426316202;
      beepSlope = -1.0;
      beepCycleRate = 5.9994634366594255;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 345

      wanderSpeed = 93.49496849812569;
      wanderRoundness = 0.1808942891657352;
      wanderCycleRate = 0.6679574539884925;
      blinkTemperature = 0.4527948824688792;
      blinkSlope = 0.0;
      blinkCycleRate = 0.6748406174592674;
      beepPitch = 333.4710732102394;
      beepSlope = -0.0;
      beepCycleRate = 3.5132168526761234;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 346

      wanderSpeed = 38.045696849003434;
      wanderRoundness = 0.5579518722370267;
      wanderCycleRate = 4.491562412120402;
      blinkTemperature = 0.5543348519131541;
      blinkSlope = -1.0;
      blinkCycleRate = 4.166893467307091;
      beepPitch = 916.6267480701208;
      beepSlope = 1.0;
      beepCycleRate = 3.2250752290710807;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 347

      wanderSpeed = 59.129502680152655;
      wanderRoundness = 0.4252273384481668;
      wanderCycleRate = 2.0385368443094194;
      blinkTemperature = 0.1791706150397658;
      blinkSlope = -0.0;
      blinkCycleRate = 5.51665616594255;
      beepPitch = 261.01477574557066;
      beepSlope = 0.0;
      beepCycleRate = 0.8647721512243152;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 348

      wanderSpeed = 73.71427929028869;
      wanderRoundness = 0.7978713167831302;
      wanderCycleRate = 5.861019409727305;
      blinkTemperature = 0.8275851076468825;
      blinkSlope = 1.0;
      blinkCycleRate = 2.1125601804815233;
      beepPitch = 577.8583498671651;
      beepSlope = -1.0;
      beepCycleRate = 5.364624943118542;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 349

      wanderSpeed = 76.05291844345629;
      wanderRoundness = 0.3660741634666919;
      wanderCycleRate = 3.6210299567319457;
      blinkTemperature = 0.9382233805954456;
      blinkSlope = 1.0;
      blinkCycleRate = 1.4900994370691478;
      beepPitch = 107.5542044825852;
      beepSlope = 0.0;
      beepCycleRate = 1.7970458921045065;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 350

      wanderSpeed = 55.48669443465769;
      wanderRoundness = 0.988982249982655;
      wanderCycleRate = 1.5176854548044505;
      blinkTemperature = 0.0553869195282459;
      blinkSlope = -0.0;
      blinkCycleRate = 3.520874012261629;
      beepPitch = 690.7106772996485;
      beepSlope = -1.0;
      beepCycleRate = 4.942588389385492;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 351

      wanderSpeed = 41.72161353752017;
      wanderRoundness = 0.1216381285339593;
      wanderCycleRate = 4.990987861528993;
      blinkTemperature = 0.6801007837057114;
      blinkSlope = -1.0;
      blinkCycleRate = 4.962569484487176;
      beepPitch = 484.99442068859935;
      beepSlope = -0.0;
      beepCycleRate = 3.956738088745624;

      WanderBlinkBeep(duration, stayInBounds,
                   wanderSpeed, wanderSlope, wanderRoundness, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                   blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                   0, 0, 0, 0, 0, 0, 0, 0);

      WanderBlinkBeep(duration, stayInBounds,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);

      // run 352

      wanderSpeed = 91.12108258530498;
      wanderRoundness = 0.7489216728135943;
      wanderCycleRate = 2.886415898799896;
      blinkTemperature = 0.3128917068243027;
      blinkSlope = 0.0;
      blinkCycleRate = 2.846523509826511;
      beepPitch = 801.8389134667814;
      beepSlope = 1.0;
      beepCycleRate = 2.2713174615055323;

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