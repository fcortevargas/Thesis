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
float turnDuration;
float forwardDuration;
float lineTurnDuration;
float targetForwardSpeed;
float targetTurnSpeed;
int acceleration;
int wanderCycle;

// Blink control variables
float lightsOnDuration;
float lightsOffDuration;
float targetRedIntensity;
float targetGreenIntensity;
float targetBlueIntensity;
int brightness;
int blinkCycle;

// Beep control variables
float soundDuration;
float silenceDuration;
float targetPitch;
float currentPitch;
float semitone;
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
double wanderTurnToForwardRatio;
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

// Helper method to set the forward, turn, and line turn durations for the wander base behavior
void SetWanderDurations(double speed, double turnToForwardRatio, double cycleRate, double cycleStandardDeviation)
{
  // Set base forward duration based on input cycle rate
  forwardDuration = (1 - turnToForwardRatio) / cycleRate;

  // Add random normal variation to the base turn duration
  forwardDuration += GenerateGaussian(cycleStandardDeviation);

  // Cap the forward duration to be positive
  forwardDuration < 0 ? 0 : forwardDuration;

  // Cap the forward duration based on input cycle rate
  forwardDuration > 1 / cycleRate ? 1 / cycleRate : forwardDuration;

  // Set turn duration
  turnDuration = 1 / cycleRate - forwardDuration;

  // Set line turn duration
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

  // Cap the lights off duration to be positive
  lightsOffDuration < 0 ? 0 : lightsOffDuration;

  // Cap the lights off duration based on input cycle rate
  lightsOffDuration > 1 / cycleRate ? 1 / cycleRate : lightsOffDuration;

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

  // Cap the silence duration to be positive
  silenceDuration < 0 ? 0 : silenceDuration;

  // Cap the silence duration based on input cycle rate
  silenceDuration > 1 / cycleRate ? 1 / cycleRate : silenceDuration;

  // Set sound duration
  soundDuration = 1 / cycleRate - silenceDuration;
}

// Helper method to set the target forward and turn speed for the wander base behavior
void SetTargetSpeeds(double speed, double roundness, double speedStandardDeviation)
{
  // Add random normal variation to the input wander speed to get the target forward speed
  targetForwardSpeed = speed + GenerateGaussian(speedStandardDeviation);

  // Cap forward speed to 25 if it's lower than 25
  if (targetForwardSpeed < 25) {
    targetForwardSpeed = 25;
  }

  // Cap forward speed to 100 if it's higher than 100
  if (targetForwardSpeed > 100) {
    targetForwardSpeed = 100;
  }

  // Set target turn speed based on input roundness and target speed
  targetTurnSpeed = -targetForwardSpeed / 2 + 4 * targetForwardSpeed * roundness - 4 * targetForwardSpeed * pow(roundness, 2);
}

// Helper method to set the target red, blue and green intensity for the blink base behavior
void SetTargetIntensities(double temperature, double temperatureStandardDeviation)
{
  // Add random normal variation to the input blink temperature to get the target temperature
  double targetTemperature = temperature + GenerateGaussian(temperatureStandardDeviation);

  // Cap temperature to zero if it's lower than zero
  if (targetTemperature < 0) {
    targetTemperature = 0;
  }

  // Cap temperature to one if it's higher than one
  if (targetTemperature > 1) {
    targetTemperature = 1;
  }

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

  // Cap target pitch to 80 Hz if it's lower than 80 Hz
  if (targetPitch < 80) {
    targetPitch = 80;
  }

  // Cap target pitch to 80 Hz if it's higher than 80 Hz
  if (targetPitch > 3000) {
    targetPitch = 3000;
  }

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
    _delay(float(silenceDuration)); // Else play silence
  }
}

// Method to execute robot's base behaviors wander, blink and beep
void WanderBlinkBeep(double duration, boolean stayInBounds,
                     double wanderSpeed, double wanderSlope, double wanderRoundness, double wanderTurnToForwardRatio, double wanderCycleRate, double wanderCycleStandardDeviation, double wanderSpeedStandardDeviation, double wanderPhase, 
                     double blinkTemperature, double blinkSlope, double blinkLightsOnToOffRatio, double blinkCycleRate, double blinkCycleStandardDeviation, double blinkTemperatureStandardDeviation, double blinkPhase,
                     double beepPitch, double beepSlope, double beepSoundToSilenceRatio, double beepCycleRate, double beepCycleStandardDeviation, double beepPitchStandardDeviation, double beepRandomSoundProbability, double beepPhase) 
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

  rgbled_7.setColor(0, 0, 0, 0);
  rgbled_7.show();
  motor_9.run(0);
  motor_10.run(0);
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
  boolean isValidCycleRate = cycleRate > 0 && cycleRate <= 4.5;
  boolean isValidStandardDeviation = cycleStandardDeviation >= 0 && pitchStandardDeviation >= 0;
  boolean isValidRandomSoundProbability = randomSoundProbability <= 1 && randomSoundProbability >= 0;
  boolean isValidPhase = phase >= 0;

  doBeep = isValidPitch && isValidSlope && isValidSoundToSilenceRatio && isValidCycleRate && isValidStandardDeviation && isValidRandomSoundProbability && isValidPhase;
  
  if (!doBeep) {
    DebugConditions(isValidPitch);
    DebugConditions(isValidSlope);
    DebugConditions(isValidSoundToSilenceRatio);
    DebugConditions(isValidCycleRate);
    DebugConditions(isValidStandardDeviation);
    DebugConditions(isValidRandomSoundProbability);
    DebugConditions(isValidPhase);
  } else {
    rgbled_7.setColor(0, 255, 255, 255);
    rgbled_7.show();
  }
}

void DebugConditions(boolean condition)
{
  if (condition) {
    rgbled_7.setColor(0, 0, 255, 0);
    rgbled_7.show();
    _delay(1);
    rgbled_7.setColor(0,0,0,0);
    rgbled_7.show();
    _delay(1);
  } else {
    rgbled_7.setColor(0, 255, 0, 0);
    rgbled_7.show();
    _delay(1);
    rgbled_7.setColor(0,0,0,0);
    rgbled_7.show();
    _delay(1);
  }
}

void _delay(float seconds) 
{
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime) _loop();
}

void setup() 
{
  // Initialize remote control signaling
  ir.begin();

  rgbled_7.fillPixelsBak(0, 2, 1);
  timesButtonPressed = 0;

  // Initialize random seed
  randomSeed(0);

  while (true) {
    // If right key is pressed on the remote control
    if (ir.keyPressed(9)) {
      timesButtonPressed += 1;

      // CONSTANT GENERAL INPUT PARAMETERS FOR ALL EMOTIONS
      duration = 20;
      stayInBounds = true;

      // CONSTANT BASE BEHAVIORS INPUT PARAMETERS FOR ALL EMOTIONS
      wanderSlope = 0;
      wanderTurnToForwardRatio = 0.8;
      wanderSpeedStandardDeviation = 0;
      wanderPhase = 0;

      blinkLightsOnToOffRatio = 0.85;
      blinkPhase = 0;

      beepCycleStandardDeviation = 0.2;
      beepPhase = 0;

      // Emotion 1 blink & wander
      if (timesButtonPressed == 0) {
        // FINE-TUNED WANDER AND BLINK INPUT PARAMETERS
        wanderSpeed = 100;
        wanderRoundness = 1;
        wanderCycleRate = 0.5;
        wanderCycleStandardDeviation = 0.5;

        blinkTemperature = 0.5;
        blinkSlope = 0;
        blinkCycleRate = 5;
        blinkTemperatureStandardDeviation = 0.4;

        // RANDOMIZED WANDER AND BLINK INPUT PARAMETERS
        blinkCycleStandardDeviation = GetRandomNumber(0, 0.5); // random value between 0 and 0.5

        // Run base behaviors
        WanderBlinkBeep(duration, stayInBounds,
                        wanderSpeed, wanderSlope, wanderRoundness, wanderTurnToForwardRatio, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                        blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                        0, 0, 0, 0, 0, 0, 0, 0);
      }

      // Emotion 1 beep
      if (timesButtonPressed == 1) {
        // FINE-TUNED BEEP INPUT PARAMETERS
        beepPitch = 700;
        beepSlope = 1;
        beepCycleRate = 3;
        beepPitchStandardDeviation = 100;
        beepRandomSoundProbability = 0.4;

        // INTERPOLATED BEEP INPUT PARAMETERS
        beepSoundToSilenceRatio = 0.1431 * beepCycleRate + 0.3496;

        // RANDOMIZED BEEP INPUT PARAMETERS
        // beepSoundToSilenceRatio = GetRandomNumber(0, 1);

        // Run base behaviors
        WanderBlinkBeep(duration, stayInBounds,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0,
                        beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);
      }

      // Emotion 2 blink & wander
      if (timesButtonPressed == 0) {
        // FINE-TUNED WANDER AND BLINK INPUT PARAMETERS
        wanderSpeed = 35;
        wanderRoundness = 0.5;
        wanderCycleRate = 0.5;
        wanderCycleStandardDeviation = 0.2;

        blinkTemperature = 0;
        blinkSlope = -1;
        blinkCycleRate = 0.1;
        blinkTemperatureStandardDeviation = 0.1;

        // RANDOMIZED WANDER AND BLINK INPUT PARAMETERS
        blinkCycleStandardDeviation = GetRandomNumber(0, 0.5); // random value between 0 and 0.5

        // Run base behaviors
        WanderBlinkBeep(duration, stayInBounds,
                        wanderSpeed, wanderSlope, wanderRoundness, wanderTurnToForwardRatio, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                        blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                        0, 0, 0, 0, 0, 0, 0, 0);
      }

      // Emotion 2 beep
      if (timesButtonPressed == 2) {
        // FINE-TUNED BEEP INPUT PARAMETERS
        beepPitch = 100;
        beepSlope = -1;
        beepCycleRate = 0.5;
        beepPitchStandardDeviation = 20;
        beepRandomSoundProbability = 0.2;

        // INTERPOLATED BEEP INPUT PARAMETERS
        beepSoundToSilenceRatio = 0.1431 * beepCycleRate + 0.3496;

        // RANDOMIZED BEEP INPUT PARAMETERS
        // beepSoundToSilenceRatio = GetRandomNumber(0, 1);

        // Run base behaviors
        WanderBlinkBeep(duration, stayInBounds,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0,
                        beepPitch, beepSlope,beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);
      }

      // Emotion 3 blink & wander
      if (timesButtonPressed == 0) {
        // FINE-TUNED WANDER AND BLINK INPUT PARAMETERS
        wanderSpeed = 100;
        wanderRoundness = 0;
        wanderCycleRate = 5;
        wanderCycleStandardDeviation = 1;

        blinkTemperature = 1;
        blinkSlope = 1;
        blinkCycleRate = 6;
        blinkTemperatureStandardDeviation = 0;

        // RANDOMIZED WANDER AND BLINK INPUT PARAMETERS
        blinkCycleStandardDeviation = GetRandomNumber(0, 0.5); // random value between 0 and 0.5

        // Run base behaviors
        WanderBlinkBeep(duration, stayInBounds,
                        wanderSpeed, wanderSlope, wanderRoundness, wanderTurnToForwardRatio, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                        blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                        0, 0, 0, 0, 0, 0, 0, 0);
      }

      // Emotion 3 beep
      if (timesButtonPressed == 3) {
        // FINE-TUNED BEEP INPUT PARAMETERS
        beepPitch = 900;
        beepSlope = 1;
        beepCycleRate = 4;
        beepPitchStandardDeviation = 100;
        beepRandomSoundProbability = 0.4;

        // INTERPOLATED BEEP INPUT PARAMETERS
        beepSoundToSilenceRatio = 0.1431 * beepCycleRate + 0.3496;

        // RANDOMIZED BEEP INPUT PARAMETERS
        // beepSoundToSilenceRatio = GetRandomNumber(0, 1);

        // Run base behaviors
        WanderBlinkBeep(duration, stayInBounds,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0,
                        beepPitch, beepSlope, beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);
      }

      // Emotion 4 blink & wander
      if (timesButtonPressed == 0) {
        // FINE-TUNED WANDER AND BLINK INPUT PARAMETERS
        wanderSpeed = 40;
        wanderRoundness = 0;
        wanderCycleRate = 6;
        wanderCycleStandardDeviation = 0.1;

        blinkTemperature = 0.3;
        blinkSlope = -1;
        blinkCycleRate = 0.1;
        blinkTemperatureStandardDeviation = 0.05;

        // RANDOMIZED WANDER AND BLINK INPUT PARAMETERS
        blinkCycleStandardDeviation = GetRandomNumber(0, 0.5); // random value between 0 and 0.5

        // Run base behaviors
        WanderBlinkBeep(duration, stayInBounds,
                        wanderSpeed, wanderSlope, wanderRoundness, wanderTurnToForwardRatio, wanderCycleRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase,
                        blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase,
                        0, 0, 0, 0, 0, 0, 0, 0);
      }

      // Emotion 4 beep
      if (timesButtonPressed == 4) {
        // FINE-TUNED BEEP INPUT PARAMETERS
        beepPitch = 200;
        beepSlope = -1;
        beepCycleRate = 0.2;
        beepPitchStandardDeviation = 50;
        beepRandomSoundProbability = 0.1;

        // INTERPOLATED BEEP INPUT PARAMETERS
        beepSoundToSilenceRatio = 0.1431 * beepCycleRate + 0.3496;

        // RANDOMIZED BEEP INPUT PARAMETERS
        // beepSoundToSilenceRatio = GetRandomNumber(0, 1);

        // Run base behaviors
        WanderBlinkBeep(duration, stayInBounds,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0,
                        beepPitch, beepSlope, beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);
      
        // Reset the count of times the remote control button has been pressed
        timesButtonPressed = 0;
      }
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