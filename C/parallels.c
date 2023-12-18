#include <Arduino.h>
#include <MeMCore.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// Timer control variables
double currentTime = 0;
double lastTime = 0;

MeLineFollower linefollower_2(2);
MeDCMotor motor_9(9);
MeDCMotor motor_10(10);
MeRGBLed rgbled_7(7, 2);
MeBuzzer buzzer;

// Wander control variables
float turnDuration;
float forwardDuration;
float turnSpeed;
int wanderCycle;

// Blink control variables
float targetRedIntensity;
float targetGreenIntensity;
float targetBlueIntensity;
float onDuration;
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
  motor_9.run((9) == M1 ? -(leftSpeed) : (leftSpeed));

  // Run right motor at given speed
  motor_10.run((10) == M1 ? -(rightSpeed) : (rightSpeed));
}

// Method to execute robot's base behaviors wander, blink and beep
void WanderBlinkBeep(double duration, 
                     double wanderSpeed, String wanderAcceleration, double wanderRoundness, double wanderTurnRate, double wanderPhase, boolean stayInBounds, 
                     double blinkTemperature, String blinkMode, double blinkTempo, double blinkPhase,
                     double beepPitch, String beepIntonation, double beepSoundToSilenceRatio, double beepTempo, double beepPhase) 
{
  // INPUT CHECKS AND VARIABLE INITIALIZATION

  // Check if wander input is valid
  CheckValidWanderInput(wanderSpeed, wanderAcceleration, wanderRoundness, wanderTurnRate, wanderPhase);

  // If wander input is valid, initialize wander variables
  if (doWander) {
    // Calculate turn duration based on input roundness and turn rate
    turnDuration = (0.2 * wanderRoundness + 0.6) / wanderTurnRate;

    // Cap the turn duration based on input roundness
    if (turnDuration > 1.4 * wanderRoundness + 0.6) {
      turnDuration = 1.4 * wanderRoundness + 0.6;
    }

    // Calculate forward duration
    forwardDuration = 1 / wanderTurnRate - turnDuration;

    // Calculate turn speed based on input roundness and speed
    turnSpeed = wanderSpeed * wanderRoundness - wanderSpeed / 2;

    // Variable used to determine if robot should turn left or right
    wanderCycle = 0;
  }

  // Check if blink input is valid
  CheckValidBlinkInput(blinkTemperature, blinkMode, blinkTempo, blinkPhase);

  // If blink input is valid, initialize blink variables
  if (doBlink) {
    // If temperature is neutral, choose a greenish yellow.
    if (blinkTemperature == 0.5) {
      targetRedIntensity = 200;
      targetGreenIntensity = 255;
      targetBlueIntensity = 0;
    }

    // If temperature is higher than 0.5, choose warm colors
    if (blinkTemperature > 0.5) {
      targetRedIntensity = round(110 * blinkTemperature + 145);
      targetGreenIntensity = round(-510 * blinkTemperature + 510);
      targetBlueIntensity = 0;
    }

    // If temperature is lower than 0.5, choose cool colors
    if (blinkTemperature < 0.5) {
      targetRedIntensity = round(100 * blinkTemperature);
      targetGreenIntensity = round(510 * blinkTemperature);
      targetBlueIntensity = round(-510 * blinkTemperature + 255);
    }

    // Initialize duration that lights are turned on
    if (blinkMode == "Constant") {
      onDuration = 1 / (2 * blinkTempo);

    } else {
      onDuration = 1 / blinkTempo;
    }

    // Variable used to control the blink cycles
    blinkCycle = 0;

    // Variable used to compute the brightness of the light 
    brightness = 1;
  }

  // Check if beep input is valid
  CheckValidBeepInput(beepPitch, beepIntonation, beepSoundToSilenceRatio, beepTempo, beepPhase);

  if (doBeep) {
    // Initialize current pitch to target pitch
    currentPitch = beepPitch;

    // Initialize sound and silence duration
    if (beepIntonation == "Constant") {
      soundDuration = beepSoundToSilenceRatio / beepTempo;
      silenceDuration = (1 - beepSoundToSilenceRatio) / beepTempo;
    }

    if (beepIntonation == "Rising" || beepIntonation == "Falling") {
      soundDuration = beepSoundToSilenceRatio / beepTempo / 12;
      silenceDuration = (1 - beepSoundToSilenceRatio) / beepTempo / 12;
    } else {
      soundDuration = beepSoundToSilenceRatio / beepTempo / 24;
      silenceDuration = (1 - beepSoundToSilenceRatio) / beepTempo / 24;
    }

    // Variable used to control the beep cycles
    beepCycle = 0;

    // Variable used to calculate the frequency of each beep note
    semitone = 0;
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

            if ((0 ? (1 == 0 ? sensorReading == 0 : (sensorReading & 1) == 1)
                   : (1 == 0 ? sensorReading == 3 : (sensorReading & 1) == 0))) {
                move(3, 100.0 / 100.0 * 255);
                _delay(0.5);
                move(3, 0);
            }

            if ((0 ? (2 == 0 ? sensorReading == 0 : (sensorReading & 2) == 2)
                   : (2 == 0 ? sensorReading == 3 : (sensorReading & 2) == 0))) {
                move(4, 100.0 / 100.0 * 255);
                _delay(0.5);
                move(4, 0);
            }
          }
        }

        // Move forward for given duration
        if (getLastTime() - wanderPhase < wanderCycle / wanderTurnRate + forwardDuration) {
          move(1, wanderSpeed / 100.0 * 255);
        } else {
          if (fmod(wanderCycle, 2) == 0) {
            // Turn right?
            motor_9.run(-1 * wanderSpeed / 100.0 * 255);
            motor_10.run(turnSpeed / 100.0 * 255);
          } else {
            // Turn left?
            motor_9.run(-1 * turnSpeed / 100.0 * 255);
            motor_10.run(wanderSpeed / 100.0 * 255);
          }
        }

        // After turn cycle is finished, increase by one
        if (getLastTime() - wanderPhase > (wanderCycle + 1) / wanderTurnRate) {
          wanderCycle += 1;
        }
    }

    // If valid blink input is given
    if (doBlink) {
      // Start executing after the given phase
      if (getLastTime() - blinkPhase >= 0) {
        // Constant mode
        if (blinkMode.equals("Constant")) {
          if (getLastTime() - blinkPhase < blinkCycle / blinkTempo + onDuration) {
            rgbled_7.setColor(0, targetRedIntensity, targetGreenIntensity, targetBlueIntensity);
            rgbled_7.show();
          } else {
            rgbled_7.setColor(0, 0, 0, 0);
            rgbled_7.show();
          }
        }

        // Rising mode
        if (blinkMode.equals("Rising")) {
          if (getLastTime() - blinkPhase < blinkCycle / blinkTempo + brightness * onDuration / 100) {
            rgbled_7.setColor(0, round(brightness * targetRedIntensity / 100),
                                 round(brightness * targetGreenIntensity / 100),
                                 round(brightness * targetBlueIntensity / 100));

            rgbled_7.show();
          } else {
            brightness += 1;
          }
        }

        // Falling mode
        if (blinkMode.equals("Falling")) {
          if (getLastTime() - blinkPhase < blinkCycle / blinkTempo + brightness * onDuration / 100) {
            rgbled_7.setColor(0, round((100 - brightness) * targetRedIntensity / 100),
                                 round((100 - brightness) * targetGreenIntensity / 100),
                                 round((100 - brightness) * targetBlueIntensity / 100));
            rgbled_7.show();
          } else {
            brightness += 1;
          }
        }

        // Rising-Falling mode
        if (blinkMode.equals("Rising-Falling")) {
          if (getLastTime() - blinkPhase < blinkCycle / blinkTempo + brightness * onDuration / 200) {
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
            brightness += 1;
          }
        }

        // Falling-Rising mode
        if (blinkMode.equals("Falling-Rising")) {
          if (getLastTime() - blinkPhase < blinkCycle / blinkTempo + brightness * onDuration / 200) {
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
            brightness += 1;
          }
        }

        // If blink cycle has finished
        if ((getLastTime() - blinkPhase) > ((blinkCycle + 1)) / blinkTempo) {
          // Increase count of cycles by one
          blinkCycle += 1;

          // Reset to one
          brightness = 1;
        }
      }
    }

    // If valid beep input is given
    if (doBeep) {
      // Start executing after the given phase
      if (getLastTime() - beepPhase >= 0) {
        if (beepIntonation == "Constant") {
          if (getLastTime() - beepPhase < beepCycle / beepTempo + (semitone + 1) * soundDuration) {
            buzzer.tone(currentPitch, soundDuration * 1000);
            _delay(float(silenceDuration));
          }
        }

        if (beepIntonation == "Rising") {
          if(getLastTime() - beepPhase < beepCycle / beepTempo + (semitone + 1) * soundDuration) {
            buzzer.tone(currentPitch, soundDuration * 1000);
            _delay(float(silenceDuration));
          } else {
            semitone += 1;
            currentPitch = exp(log(beepPitch) + semitone / 12 * log(2));
          }

        }

        if (beepIntonation == "Falling") {
          if (getLastTime() - beepPhase < beepCycle / beepTempo + (semitone + 1) * soundDuration) {
            buzzer.tone(currentPitch, soundDuration * 1000);
            _delay(float(silenceDuration));
          } else {
            semitone += 1;
            currentPitch = exp(log(beepPitch) - semitone / 12 * log(2));
          }
        }

        if (beepIntonation == "Rising-Falling") {
          if (getLastTime() - beepPhase < beepCycle / beepTempo + (semitone + 1) * soundDuration) {
            buzzer.tone(currentPitch, soundDuration * 1000);
            _delay(float(silenceDuration));
          } else {
            semitone += 1;
            if (semitone < 12) {
              currentPitch = exp(log(beepPitch) + semitone / 12 * log(2));
            } else {
              currentPitch = exp(log(beepPitch) + (24 - semitone) / 12 * log(2));
            }
          }
        }

        if (beepIntonation == "Falling-Rising") {
          if (getLastTime() - beepPhase < beepCycle / beepTempo + (semitone + 1) * soundDuration) {
            buzzer.tone(currentPitch, soundDuration * 1000);
            _delay(float(silenceDuration));
          } else {
            semitone += 1;
            if (semitone < 12) {
              currentPitch = exp(log(beepPitch) - semitone / 12 * log(2));
            } else {
              currentPitch = exp(log(beepPitch) - (24 - semitone) / 12 * log(2));
            }
          }
        }
      }

      if (getLastTime() - beepPhase > (beepCycle + 1) / beepTempo) {
        beepCycle += 1;
        semitone = 0;
      }
    }
  }

  rgbled_7.setColor(0, 0, 0, 0);
  rgbled_7.show();
  motor_9.run(0);
  motor_10.run(0);
}

void CheckValidWanderInput(double speed, String acceleration, double roundness, double turnRate, double phase)
{
    boolean isValidSpeed = speed >= 0 && speed <= 100;
    boolean isTurnRatePositive = turnRate > 0;
    boolean isValidAcceleration = acceleration.equals("Constant") || acceleration.equals("Rising") || acceleration.equals("Falling");
    boolean isValidRoundness = roundness >= 0 && roundness <= 1;
    boolean isValidPhase = phase >= 0;

    if (isValidSpeed && isTurnRatePositive && isValidAcceleration && isValidRoundness && isValidPhase) {
      doWander = true;
    } else {
      doWander = false;
    }
}

void CheckValidBlinkInput(double temperature, String mode, double tempo, double phase) 
{
  boolean isValidTemperature = temperature >= 0 && temperature <= 1;
  boolean isValidMode = mode.equals("Constant") || mode.equals("Rising") || mode.equals("Falling") || mode.equals("Rising-Falling") || mode.equals("Falling-Rising");
  boolean isTempoPositive = tempo > 0;
  boolean isValidPhase = phase >= 0;

  if (isValidTemperature && isValidMode && isTempoPositive && isValidPhase) {
    doBlink = true;
  } else {
    doBlink = false;
  }
}

void CheckValidBeepInput(double pitch, String intonation, double soundToSilenceRatio, double tempo, double phase) 
{
  boolean isValidPitch = pitch > 0 && pitch < 5000;
  boolean isValidIntonation = intonation.equals("Constant") || intonation.equals("Rising") || intonation.equals("Falling") || intonation.equals("Rising-Falling") || intonation.equals("Falling-Rising");
  boolean isValidSoundToSilenceRatio = soundToSilenceRatio <= 1 && soundToSilenceRatio >= 0;
  boolean isTempoPositive = tempo > 0;
  boolean isValidPhase = phase >= 0;

  if (isValidPitch && isValidIntonation && isTempoPositive && isValidSoundToSilenceRatio && isValidPhase) {
    doBeep = true;
  } else {
    doBeep = false;
  }
}

void _delay(float seconds) {
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime) _loop();
}

void setup() {
  pinMode(A7, INPUT);
  rgbled_7.fillPixelsBak(0, 2, 1);
  timesButtonPressed = 0;
  while (true) {
    if ((0 ^ (analogRead(A7) > 10 ? 0 : 1))) {
      timesButtonPressed += 1;

      if (timesButtonPressed == 1) {
        WanderBlinkBeep(// duration,
                        5, 
                        // wanderSpeed, wanderAcceleration, wanderRoundness, wanderTurnRate, wanderPhase, stayInBounds
                        0,              "",                 0,               0,              0,           false, 
                        // blinkTemperature, blinkMode,        blinkTempo, blinkPhase
                        0,                   "",               0,          0, 
                        // beepPitch, beepIntonation,   beepSoundToSilenceRatio, beepTempo, beepPhase
                        440,          "Falling-Rising", 0.5,                     0.5,         0);
        timesButtonPressed = 0;
      }
    }
    _loop();
  }
}

void _loop() 
{
}

void loop() 
{
  _loop();
}