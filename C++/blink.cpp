#include <Arduino.h> // Core Arduino library for basic functions and macros
#include <MeMCore.h> // Library for Makeblock electronic modules like motors and sensors
#include <SoftwareSerial.h> // Library for serial communication on digital pins
#include <Wire.h> // Library for I2C communication
#include <stdlib.h> // Standard library for general operations like random numbers
#include <math.h> // Math library for advanced mathematical operations

// Timer control variables to track the current and last time measurements
double currentTime = 0;
double lastTime = 0;

// Initialize an RGB LED module connected to port 7 on the main board
MeRGBLed rgbled_7(7, 2);

// Variables for controlling the blinking behavior of the LED
double lightsOnDuration; // How long the lights stay on during a cycle
double lightsOffDuration; // How long the lights stay off during a cycle
double targetRedIntensity; // Target intensity for the red component
double targetGreenIntensity; // Target intensity for the green component
double targetBlueIntensity; // Target intensity for the blue component
int brightness; // Current brightness level
int blinkCycle; // Counter for the number of blink cycles

// Boolean flag to check if the input parameters for blinking are valid
boolean doBlink;

// General input parameters for controlling the overall behavior duration
double duration = 10; // Duration for the blink behavior

// Input parameters specifically for the blinking behavior
double blinkTemperature = 0.9; // Determines the color temperature for the LED
double blinkSlope = 1; // Determines how the intensity changes over time
double blinkLightsOnToOffRatio = 0.9; // Ratio of on-time to off-time
double blinkCycleRate = 2; // How many cycles per second
double blinkCycleStandardDeviation = 0.5; // Variability in the cycle rate
double blinkTemperatureStandardDeviation = 0.1; // Variability in the color temperature
double blinkPhase = 0; // Initial phase delay before starting to blink

// Function to calculate elapsed time since the last reset
double getLastTime() 
{ 
  return currentTime = millis() / 1000.0 - lastTime; 
}

// Generates a Gaussian-distributed random number based on standard deviation
double GenerateGaussian(double standardDeviation) 
{
    double u1 = rand() / (RAND_MAX + 1.0);
    double u2 = rand() / (RAND_MAX + 1.0);
    double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    return z0 * standardDeviation;
}

// Generates a random number within a specified range
double GetRandomNumber(double min, double max)
{
    double randomNumber = (double)rand() / RAND_MAX;
    return (randomNumber * (max - min)) + min;
}

// Caps a number to be within a specified range
void CapNumber(double* number, double lowerLimit, double upperLimit)
{
    if (*number < lowerLimit) {
        *number = lowerLimit;
    }
    if (*number > upperLimit) {
        *number = upperLimit;
    }
}

// Checks if the input parameters for the blinking behavior are within valid ranges
void CheckValidBlinkInput(double temperature, double slope, double lightsOnToOffRatio, double cycleRate, double cycleStandardDeviation, double temperatureStandardDeviation, double phase) 
{
  boolean isValidTemperature = temperature >= 0 && temperature <= 1;
  boolean isValidSlope = slope >= -5 && slope <= 5;
  boolean isValidLightsOnToOffRatio = lightsOnToOffRatio <= 1 && lightsOnToOffRatio > 0;
  boolean isCycleRatePositive = cycleRate > 0;
  boolean isValidStandardDeviation = cycleStandardDeviation >= 0 && temperatureStandardDeviation >= 0;
  boolean isValidPhase = phase >= 0;

  // If all conditions are met, the input is considered valid
  doBlink = isValidTemperature && isValidSlope && isValidLightsOnToOffRatio && isCycleRatePositive && isValidStandardDeviation && isValidPhase;
}

// Sets the durations for the lights being on and off based on the input parameters
void SetBlinkDurations(double lightsOnToOffRatio, double cycleRate, double cycleStandardDeviation)
{
  lightsOffDuration = (1 - lightsOnToOffRatio) / cycleRate;
  lightsOffDuration += GenerateGaussian(cycleStandardDeviation);
  CapNumber(&lightsOffDuration, 0, 1 / cycleRate);
  lightsOnDuration = 1 / cycleRate - lightsOffDuration;
}

// Determines the target intensities for the RGB components of the LED based on temperature
void SetTargetIntensities(double temperature, double temperatureStandardDeviation)
{
  double targetTemperature = temperature + GenerateGaussian(temperatureStandardDeviation);
  CapNumber(&targetTemperature, 0, 1);

  // Adjust the RGB intensities based on the calculated target temperature
  if (targetTemperature == 0.5) {
    targetRedIntensity = 200;
    targetGreenIntensity = 255;
    targetBlueIntensity = 0;
  } else if (targetTemperature > 0.5) {
    targetRedIntensity = round(110 * targetTemperature + 145);
    targetGreenIntensity = round(-510 * targetTemperature + 510);
    targetBlueIntensity = 0;
  } else if (targetTemperature < 0.5) {
    targetRedIntensity = round(100 * targetTemperature);
    targetGreenIntensity = round(510 * targetTemperature);
    targetBlueIntensity = round(-510 * targetTemperature + 255);
  }
}

// Method to execute the blink base behavior
void Blink(double duration,
           double blinkTemperature, double blinkSlope, double blinkLightsOnToOffRatio, double blinkCycleRate, double blinkCycleStandardDeviation, double blinkTemperatureStandardDeviation, double blinkPhase) 
{
  // INPUT CHECKS AND VARIABLE INITIALIZATION

  // Validates the input parameters to ensure they are within acceptable ranges for the blink behavior.
  CheckValidBlinkInput(blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase);

  // Proceeds only if the input parameters are validated successfully.
  if (doBlink) {
    // Sets the color intensities for the RGB LED based on the calculated temperature, including randomness.
    SetTargetIntensities(blinkTemperature, blinkTemperatureStandardDeviation);

    // Determines the durations for which the LED will stay on and off during each blink cycle, including randomness.
    SetBlinkDurations(blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation);

    // Initializes the counter that keeps track of the number of completed blink cycles.
    blinkCycle = 0;

    // Initializes the brightness adjustment factor; used when blinkSlope != 0 to vary intensity.
    brightness = 1;
  }

  // MAIN LOOP

  // Resets the timer to keep track of the blink behavior's duration.
  lastTime = millis() / 1000.0;

  // Continues blinking for the specified duration.
  while (!(getLastTime() > duration)) {
    // Placeholder for tasks that need continuous execution within the loop.

    // Checks again if blink behavior should continue based on the doBlink flag.
    if (doBlink) {
      // Delays the start of blinking until after the specified phase delay.
      if (getLastTime() - blinkPhase >= 0) {
        // Handles constant intensity blinking without any slope for intensity change.
        if (blinkSlope == 0) {
          // Checks if it's time to turn the lights on within the current blink cycle.
          if (getLastTime() - blinkPhase < blinkCycle / blinkCycleRate + lightsOnDuration) {
            // Sets the LED color using the target intensity values.
            rgbled_7.setColor(0, targetRedIntensity, targetGreenIntensity, targetBlueIntensity);
            rgbled_7.show();
          }
        }
        // Handles increasing intensity blinking when blinkSlope is positive.
        else if (blinkSlope > 0) {
          // Gradually increases brightness until it reaches the maximum value.
          if (getLastTime() - blinkPhase < blinkCycle / blinkCycleRate + brightness * lightsOnDuration / 100) {
            // Adjusts the LED color intensity based on the current brightness.
            rgbled_7.setColor(0, round(blinkSlope * brightness * targetRedIntensity / 100),
                                 round(blinkSlope * brightness * targetGreenIntensity / 100),
                                 round(blinkSlope * brightness * targetBlueIntensity / 100));
            rgbled_7.show();
          } else {
            // Increases the brightness for the next iteration, if it has not reached the maximum.
            if (brightness < 100) {
              brightness += 1;
            }
          }
        }
        // Handles decreasing intensity blinking when blinkSlope is negative.
        else if (blinkSlope < 0) {
          // Gradually decreases brightness until it reaches the minimum value.
          if (getLastTime() - blinkPhase < blinkCycle / blinkCycleRate + brightness * lightsOnDuration / 100) {
            // Adjusts the LED color intensity based on the current brightness.
            rgbled_7.setColor(0, round(blinkSlope * (brightness - 100) * targetRedIntensity / 100),
                                 round(blinkSlope * (brightness - 100) * targetGreenIntensity / 100),
                                 round(blinkSlope * (brightness - 100) * targetBlueIntensity / 100));
            rgbled_7.show();
          } else {
            // Increases the brightness for the next iteration, if it has not reached the maximum.
            if (brightness < 100) {
              brightness += 1;
            }
          }
        }

        // Turns the lights off after the on-duration within the current cycle.
        if (getLastTime() - blinkPhase > blinkCycle / blinkCycleRate + lightsOnDuration && 
            getLastTime() - blinkPhase < (blinkCycle + 1) / blinkCycleRate) {
          rgbled_7.setColor(0, 0, 0, 0);
          rgbled_7.show();
        }

        // Prepares for the next blink cycle once the current cycle completes.
        if (getLastTime() - blinkPhase > (blinkCycle + 1) / blinkCycleRate) {
          // Increments the blink cycle counter to track the number of completed cycles.
          blinkCycle += 1;

          // Resets the brightness to its initial value for the next cycle.
          brightness = 1;

          // Re-calculates the target color intensities for the next cycle to include variability.
          SetTargetIntensities(blinkTemperature, blinkTemperatureStandardDeviation);

          // Re-calculates the durations for lights on and off for the next cycle to include variability.
          SetBlinkDurations(blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation);
        }
      }
    }
  }

  // Turns off the LED at the end of the blinking behavior to ensure it does not stay on.
  rgbled_7.setColor(0, 0, 0, 0);
  rgbled_7.show();
}

void setup() 
{
  // Initial setup for the RGB LED.
  rgbled_7.fillPixelsBak(0, 2, 1); // Pre-configure the LED with a base color or pattern.
  randomSeed(0); // Initialize the random number generator seed.

  // Start the blinking behavior with the specified parameters.
  Blink(duration, blinkTemperature, blinkSlope, blinkLightsOnToOffRatio, blinkCycleRate, blinkCycleStandardDeviation, blinkTemperatureStandardDeviation, blinkPhase);
}

// Placeholder loop function, required for Arduino structure but not used
void _loop() 
{
  
}

void loop() 
{
  _loop(); // Invoke the placeholder loop function within the main loop.
}