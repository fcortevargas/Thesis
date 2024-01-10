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
float soundDuration = 0;
float silenceDuration = 0;
float currentPitch = 0;
float semitone;
int beepCycle;

// Booleans to determine if input is valid
boolean doWander;
boolean doBeep;
boolean doBlink;

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

// Helper method to set the forward, turn, and line turn durations for the wander base behavior
void SetWanderDurations(double speed, double roundness, double turnRate, double cycleStandardDeviation)
{
  // Set base turn duration based on input roundness and turn rate
  turnDuration = (0.2 * roundness + 0.6) / turnRate;

  // Add random normal variation to the base turn duration
  turnDuration += GenerateGaussian(cycleStandardDeviation);

  // Cap the turn duration to be positive
  turnDuration < 0 ? 0 : turnDuration;

  // Cap the turn duration based on input turn rate
  turnDuration > 1 / turnRate ? 1 / turnRate : turnDuration;

  // Set forward duration
  forwardDuration = 1 / turnRate - turnDuration;

  // Set line turn duration
  if (speed <= 100 && speed >= 50){
    lineTurnDuration = 0.5;
  } else {
    lineTurnDuration = 0.0008 * pow(speed, 2) - 0.11 * speed + 4;
  }
}

// Helper method to set the lights on and off durations for the blink base behavior
void SetBlinkDurations(String mode, double lightsOnToOffRatio, double tempo, double cycleStandardDeviation)
{
  // Set lights off duration based on input tempo
  lightsOffDuration = (1 - lightsOnToOffRatio) / tempo;

  // Add random normal variation to the base turn duration
  lightsOffDuration += GenerateGaussian(cycleStandardDeviation);

  // Cap the turn duration to be positive
  lightsOffDuration < 0 ? 0 : lightsOffDuration;

  // Cap the turn duration based on input turn rate
  lightsOffDuration > 1 / tempo ? 1 / tempo : lightsOffDuration;

  // Set forward duration
  lightsOnDuration = 1 / tempo - lightsOffDuration;
}

// Helper method to set the sound and silence durations for the blink base behavior
void SetBeepDurations(String intonation, double soundToSilenceRatio, double tempo)
{
  // Initialize sound and silence duration
  soundDuration = soundToSilenceRatio / tempo;
  silenceDuration = (1 - soundToSilenceRatio) / tempo;
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
  targetTurnSpeed = targetForwardSpeed * roundness - targetForwardSpeed / 2;
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

// Method to execute robot's base behaviors wander, blink and beep
void WanderBlinkBeep(double duration, 
                     double wanderSpeed, String wanderAcceleration, double wanderRoundness, double wanderTurnRate, double wanderCycleStandardDeviation, double wanderSpeedStandardDeviation, double wanderPhase, boolean stayInBounds, 
                     double blinkTemperature, String blinkMode, double blinkLightsOnToOffRatio, double blinkTempo, double blinkCycleStandardDeviation, double blinkTemperatureStandardDeviation, double blinkPhase,
                     double beepPitch, String beepIntonation, double beepSoundToSilenceRatio, double beepTempo, double beepCycleStandardDeviation, double beepPitchStandardDeviation, double beepPhase) 
{
  // INPUT CHECKS AND VARIABLE INITIALIZATION

  // Check if wander input is valid
  CheckValidWanderInput(wanderSpeed, wanderAcceleration, wanderRoundness, wanderTurnRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase);

  // If wander input is valid, initialize wander variables
  if (doWander) {
    // Set the target forward and turning speeds based on given input
    SetTargetSpeeds(wanderSpeed, wanderRoundness, wanderSpeedStandardDeviation);
    
    // Set turn, line turn and forward durations based on given input
    SetWanderDurations(wanderSpeed, wanderRoundness, wanderTurnRate, wanderCycleStandardDeviation);

    // Variable used to determine if robot should turn left or right
    wanderCycle = 0;

    acceleration = 1; 
  }

  // Check if blink input is valid
  CheckValidBlinkInput(blinkTemperature, blinkMode, blinkLightsOnToOffRatio, blinkTempo, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase);

  // If blink input is valid, initialize blink variables
  if (doBlink) {
    // Set the target red, green and blue light intensities based on given input
    SetTargetIntensities(blinkTemperature, blinkTemperatureStandardDeviation);

    // Set lights on and off durations based on given input
    SetBlinkDurations(blinkMode, blinkLightsOnToOffRatio, blinkTempo, blinkCycleStandardDeviation);

    // Variable used to control the blink cycles
    blinkCycle = 0;

    // Variable used to compute the brightness of the light 
    brightness = 1;
  }

  // Check if beep input is valid
  CheckValidBeepInput(beepPitch, beepIntonation, beepSoundToSilenceRatio, beepTempo, beepCycleStandardDeviation, beepPitchStandardDeviation, beepPhase);

  if (doBeep) {
    // Initialize current pitch to target pitch
    currentPitch = beepPitch;

    // Set sound and silence durations based on given input
    SetBeepDurations(beepIntonation, beepSoundToSilenceRatio, beepTempo);

    // Variable used to control the beep cycles
    beepCycle = 0;

    // Variable used to calculate the frequency of each beep note
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

        // Constant mode
        if (wanderAcceleration.equals("Constant")){
          if (getLastTime() - wanderPhase < wanderCycle / wanderTurnRate + forwardDuration) {
            move(1, targetForwardSpeed / 100.0 * 255);
          } 
        }

        // Rising mode
        if (wanderAcceleration.equals("Rising")){
          if (getLastTime() - wanderPhase < wanderCycle / wanderTurnRate + acceleration * forwardDuration / 100) {
            move(1, acceleration * targetForwardSpeed / 100 / 100.0 * 255);
          } else {
            if (acceleration < 100) {
              acceleration += 1;
            }
          }
        }

        // Falling mode
        if (wanderAcceleration.equals("Falling")){
          if (getLastTime() - wanderPhase < wanderCycle / wanderTurnRate + acceleration * forwardDuration / 100) {
            move(1, (100 - acceleration) * targetForwardSpeed / 100 / 100.0 * 255);
          } else {
            if (acceleration < 100) {
              acceleration += 1;
            }
          }
        }

        // Turn for given duration
        if (getLastTime() - wanderPhase > wanderCycle / wanderTurnRate + forwardDuration && 
            getLastTime() - wanderPhase < (wanderCycle + 1) / wanderTurnRate) {
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
        if (getLastTime() - wanderPhase > (wanderCycle + 1) / wanderTurnRate) {
          // Increase cycle count by one
          wanderCycle += 1;

          // Reset to one
          acceleration = 1;

          // Change target forward and turning speeds
          SetTargetSpeeds(wanderSpeed, wanderRoundness, wanderSpeedStandardDeviation);

          // Change forward and turn durations
          SetWanderDurations(wanderSpeed, wanderRoundness, wanderTurnRate, wanderCycleStandardDeviation);
        }
    }

    // If valid blink input is given
    if (doBlink) {
      // Start executing after the given phase
      if (getLastTime() - blinkPhase >= 0) {
        // Turn lights on for given duration

        // Constant mode
        if (blinkMode.equals("Constant")) {
          if (getLastTime() - blinkPhase < blinkCycle / blinkTempo + lightsOnDuration) {
            rgbled_7.setColor(0, targetRedIntensity, targetGreenIntensity, targetBlueIntensity);
            rgbled_7.show();
          }
        }

        // Rising mode
        if (blinkMode.equals("Rising")) {
          if (getLastTime() - blinkPhase < blinkCycle / blinkTempo + brightness * lightsOnDuration / 100) {
            rgbled_7.setColor(0, round(brightness * targetRedIntensity / 100),
                                 round(brightness * targetGreenIntensity / 100),
                                 round(brightness * targetBlueIntensity / 100));
            rgbled_7.show();
          } else {
            if (brightness < 100) {
              brightness += 1;
            }
          }
        }

        // Falling mode
        if (blinkMode.equals("Falling")) {
          if (getLastTime() - blinkPhase < blinkCycle / blinkTempo + brightness * lightsOnDuration / 100) {
            rgbled_7.setColor(0, round((100 - brightness) * targetRedIntensity / 100),
                                 round((100 - brightness) * targetGreenIntensity / 100),
                                 round((100 - brightness) * targetBlueIntensity / 100));
            rgbled_7.show();
          } else {
            if (brightness < 100) {
              brightness += 1;
            }
          }
        }

        // Rising-Falling mode
        if (blinkMode.equals("Rising-Falling")) {
          if (getLastTime() - blinkPhase < blinkCycle / blinkTempo + brightness * lightsOnDuration / 200) {
            if (brightness < 100) {
              rgbled_7.setColor(0, round(brightness * targetRedIntensity / 100),
                                   round(brightness * targetGreenIntensity / 100),
                                   round(brightness * targetBlueIntensity / 100));
              rgbled_7.show();
            } else {
              rgbled_7.setColor(0, round((200 - brightness) * targetRedIntensity / 100),
                                   round((200 - brightness) * targetGreenIntensity / 100),
                                   round((200 - brightness) * targetBlueIntensity / 100));
              rgbled_7.show();
            }
          } else {
            if (brightness < 200) {
              brightness += 1;
            }
          }
        }

        // Falling-Rising mode
        if (blinkMode.equals("Falling-Rising")) {
          if (getLastTime() - blinkPhase < blinkCycle / blinkTempo + brightness * lightsOnDuration / 200) {
            if (brightness < 100) {
              rgbled_7.setColor(0, round((100 - brightness) * targetRedIntensity / 100),
                                   round((100 - brightness) * targetGreenIntensity / 100),
                                   round((100 - brightness) * targetBlueIntensity / 100));
              rgbled_7.show();
            } else {
              rgbled_7.setColor(0, round((brightness - 100) * targetRedIntensity / 100),
                                   round((brightness - 100) * targetGreenIntensity / 100),
                                   round((brightness - 100) * targetBlueIntensity / 100));
              rgbled_7.show();
            }
          } else {
            if (brightness < 200) {
              brightness += 1;
            }
          }
        }

        // Turn lights off for given duration
        if (getLastTime() - blinkPhase > blinkCycle / blinkTempo + lightsOnDuration && 
            getLastTime() - blinkPhase < (blinkCycle + 1) / blinkTempo) {
          rgbled_7.setColor(0, 0, 0, 0);
          rgbled_7.show();
        }

        // If blink cycle has finished
        if (getLastTime() - blinkPhase > (blinkCycle + 1) / blinkTempo) {
          // Increase count of cycles by one
          blinkCycle += 1;

          // Reset to one
          brightness = 1;

          // Change target red, green and blue light intensities
          SetTargetIntensities(blinkTemperature, blinkTemperatureStandardDeviation);

          // Change lights on and off durations
          SetBlinkDurations(blinkMode, blinkLightsOnToOffRatio, blinkTempo, blinkCycleStandardDeviation);
        }
      }
    }

    // If valid beep input is given
    if (doBeep) {
      // Start executing after the given phase
      if (getLastTime() - beepPhase >= 0) {
        if (beepIntonation == "Constant") {
          if (getLastTime() - beepPhase < beepCycle / beepTempo + soundDuration) {
            buzzer.tone(currentPitch, soundDuration * 1000);
            _delay(float(silenceDuration));
          }
        }

        if (beepIntonation == "Rising") {
          if(getLastTime() - beepPhase < beepCycle / beepTempo + semitone * soundDuration / 12) {
            buzzer.tone(currentPitch, soundDuration / 12 * 1000);
            _delay(float(silenceDuration) / 12);
          } else {
            currentPitch = exp(log(beepPitch) + semitone / 12 * log(2));
            
            if (semitone < 12) {
              semitone += 1;
            }
          }

        }

        if (beepIntonation == "Falling") {
          if (getLastTime() - beepPhase < beepCycle / beepTempo + semitone * soundDuration / 12) {
            buzzer.tone(currentPitch, soundDuration / 12 * 1000);
            _delay(float(silenceDuration) / 12);
          } else {
            currentPitch = exp(log(beepPitch) - semitone / 12 * log(2));
            
            if (semitone < 12) {
              semitone += 1;
            }
          }
        }

        if (beepIntonation == "Rising-Falling") {
          if (getLastTime() - beepPhase < beepCycle / beepTempo + semitone * soundDuration / 24) {
            buzzer.tone(currentPitch, soundDuration / 24 * 1000);
            _delay(float(silenceDuration) / 24);
          } else {
            if (semitone < 12) {
              currentPitch = exp(log(beepPitch) + semitone / 12 * log(2));
            } else {
              currentPitch = exp(log(beepPitch) + (24 - semitone) / 12 * log(2));
            }

            if (semitone < 24) {
              semitone += 1;
            }
          }
        }

        if (beepIntonation == "Falling-Rising") {
          if (getLastTime() - beepPhase < beepCycle / beepTempo + semitone * soundDuration / 24) {
            buzzer.tone(currentPitch, soundDuration / 24 * 1000);
            _delay(float(silenceDuration) / 24);
          } else {
            if (semitone < 12) {
              currentPitch = exp(log(beepPitch) - semitone / 12 * log(2));
            } else {
              currentPitch = exp(log(beepPitch) - (24 - semitone) / 12 * log(2));
            }
            
            if (semitone < 24) {
              semitone += 1;
            }
          }
        }
      }

      if (getLastTime() - beepPhase > (beepCycle + 1) / beepTempo) {
        beepCycle += 1;
        semitone = 1;
      }
    }
  }

  rgbled_7.setColor(0, 0, 0, 0);
  rgbled_7.show();
  motor_9.run(0);
  motor_10.run(0);
}

void CheckValidWanderInput(double speed, String acceleration, double roundness, double turnRate, double cycleStandardDeviation, double speedStandardDeviation, double phase)
{
    boolean isValidSpeed = speed >= 25 && speed <= 100;
    boolean isTurnRatePositive = turnRate > 0;
    boolean isValidAcceleration = acceleration.equals("Constant") || acceleration.equals("Rising") || acceleration.equals("Falling");
    boolean isValidRoundness = roundness >= 0 && roundness <= 1;
    boolean isValidStandardDeviation = cycleStandardDeviation >= 0 && speedStandardDeviation >= 0;
    boolean isValidPhase = phase >= 0;

    if (isValidSpeed && isTurnRatePositive && isValidAcceleration && isValidRoundness && isValidStandardDeviation && isValidPhase) {
      doWander = true;
    } else {
      doWander = false;
    }
}

void CheckValidBlinkInput(double temperature, String mode, double lightsOnToOffRatio, double tempo, double cycleStandardDeviation, double temperatureStandardDeviation, double phase) 
{
  boolean isValidTemperature = temperature >= 0 && temperature <= 1;
  boolean isValidMode = mode.equals("Constant") || mode.equals("Rising") || mode.equals("Falling") || mode.equals("Rising-Falling") || mode.equals("Falling-Rising");
  boolean isValidLightsOnToOffRatio = lightsOnToOffRatio <= 1 && lightsOnToOffRatio >= 0;
  boolean isTempoPositive = tempo > 0;
  boolean isValidStandardDeviation = cycleStandardDeviation >= 0 && temperatureStandardDeviation >= 0;
  boolean isValidPhase = phase >= 0;

  if (isValidTemperature && isValidMode && isValidLightsOnToOffRatio && isTempoPositive && isValidStandardDeviation && isValidPhase) {
    doBlink = true;
  } else {
    doBlink = false;
  }
}

void CheckValidBeepInput(double pitch, String intonation, double soundToSilenceRatio, double tempo, double cycleStandardDeviation, double pitchStandardDeviation, double phase) 
{
  boolean isValidPitch = pitch > 0 && pitch < 5000;
  boolean isValidIntonation = intonation.equals("Constant") || intonation.equals("Rising") || intonation.equals("Falling") || intonation.equals("Rising-Falling") || intonation.equals("Falling-Rising");
  boolean isValidSoundToSilenceRatio = soundToSilenceRatio <= 1 && soundToSilenceRatio >= 0;
  boolean isTempoPositive = tempo > 0;
  boolean isValidStandardDeviation = cycleStandardDeviation >= 0 && pitchStandardDeviation >= 0;
  boolean isValidPhase = phase >= 0;

  if (isValidPitch && isValidIntonation && isValidSoundToSilenceRatio && isTempoPositive && isValidStandardDeviation && isValidPhase) {
    doBeep = true;
  } else {
    doBeep = false;
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

      // Happy
      if (timesButtonPressed == 1) {
        WanderBlinkBeep(// duration,
                        5, 
                        // wanderSpeed, wanderAcceleration, wanderRoundness, wanderTurnRate, wanderCycleStandardDeviation, wanderSpeedStandardDeviation, wanderPhase, stayInBounds
                        0,              "Constant",         0,               0.5,            0.5,                          10,                           0,           true, 
                        // blinkTemperature, blinkMode,        blinkLightsOnToOffRatio, blinkTempo, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase
                        0.5,                 "",               0.9,                     5,          0.2,                         0.4,                               0, 
                        // beepPitch, beepIntonation,   beepSoundToSilenceRatio, beepTempo, beepCycleStandardDeviation, beepPitchStandardDeviation, beepPhase
                        400,          "Rising-Falling", 0.9,                     1,         0,                          0,                          0);
 
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