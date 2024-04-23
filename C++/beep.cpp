#include <Arduino.h> // Core Arduino library for basic input/output functions, types, and constants.
#include <MeMCore.h> // Library for Makeblock electronic modules, including the buzzer.
#include <SoftwareSerial.h> // Library for serial communication on digital pins.
#include <Wire.h> // Library for I2C communication.
#include <stdlib.h> // Standard library for general utility functions.
#include <math.h> // Library for mathematical functions.

// Timer control variables to measure elapsed time.
double currentTime = 0; // Current time in seconds since the start.
double lastTime = 0; // Time at the last significant event to calculate elapsed time.

// Arduino component for sound generation.
MeBuzzer buzzer; // Buzzer object for emitting sounds.

// Beep control variables for managing beep pattern.
double soundDuration; // Duration of the sound in each beep cycle.
double silenceDuration; // Duration of silence in each beep cycle.
double targetPitch; // Desired pitch of the beep in Hertz.
double currentPitch; // Current pitch of the beep in Hertz, used during execution.
double semitone; // Incremental change in pitch between beeps, related to musical semitones.
int beepCycle; // Counter for the number of completed beep cycles.

// Flag to determine if the beep input parameters are valid.
boolean doBeep;

// General input parameter for the behavior's duration.
double duration = 10; // Duration for the beep behavior in seconds.

// Specific input parameters for beep behavior customization.
double beepPitch = 400; // Base pitch of the beep in Hertz.
double beepSlope = 1; // Determines how the pitch changes over time.
double beepSoundToSilenceRatio = 0.9; // Ratio of sound duration to silence duration in each cycle.
double beepCycleRate = 2; // Number of beep cycles per second.
double beepCycleStandardDeviation = 0.5; // Variability in the cycle rate to introduce randomness.
double beepPitchStandardDeviation = 100; // Variability in the beep pitch to introduce randomness.
double beepRandomSoundProbability = 0.3; // Probability of playing a random sound instead of the target pitch.
double beepPhase = 0; // Initial phase delay before starting the beep behavior.

// Function to calculate elapsed time since the program started in seconds.
double getLastTime() 
{ 
  return currentTime = millis() / 1000.0 - lastTime; 
}

// Custom delay function to pause execution for a specified number of seconds.
void _delay(double seconds) 
{
  long endTime = millis() + seconds * 1000; // Calculate end time in milliseconds.
  while (millis() < endTime) _loop(); // Wait until the end time is reached.
}

// Generates a Gaussian-distributed random number based on standard deviation.
double GenerateGaussian(double standardDeviation) 
{
    double u1 = rand() / (RAND_MAX + 1.0); // Generate uniform random number u1.
    double u2 = rand() / (RAND_MAX + 1.0); // Generate uniform random number u2.
    double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2); // Box-Muller transform for Gaussian distribution.
    return z0 * standardDeviation;
}

// Generates a random number within the specified range [min, max].
double GetRandomNumber(double min, double max)
{
    double randomNumber = (double)rand() / RAND_MAX; // Generate a uniform random number between 0 and 1.
    return (randomNumber * (max - min)) + min; // Scale and shift the number to the specified range.
}

// Caps a number to be within the specified lower and upper limits.
void CapNumber(double* number, double lowerLimit, double upperLimit)
{
    if (*number < lowerLimit) {
        *number = lowerLimit; // Set to lower limit if below it.
    }
    if (*number > upperLimit) {
        *number = upperLimit; // Set to upper limit if above it.
    }
}

// Validates the input parameters for the beep behavior.
void CheckValidBeepInput(double pitch, double slope, double soundToSilenceRatio, double cycleRate, double cycleStandardDeviation, double pitchStandardDeviation, double randomSoundProbability, double phase) 
{
  // Check each parameter against its valid range.
  boolean isValidPitch = pitch >= 80 && pitch <= 3000; // Valid pitch range.
  boolean isValidSlope = (slope < 0 && slope >= log(40 / pitch) / log(2)) || (slope > 0 && slope <= log(6000 / pitch) / log(2)) || slope == 0; // Valid slope conditions.
  boolean isValidSoundToSilenceRatio = soundToSilenceRatio <= 1 && soundToSilenceRatio >= 0; // Valid ratio range.
  boolean isCycleRatePositive = cycleRate > 0; // Cycle rate must be positive.
  boolean isValidStandardDeviation = cycleStandardDeviation >= 0 && pitchStandardDeviation >= 0; // Standard deviations must be non-negative.
  boolean isValidRandomSoundProbability = randomSoundProbability <= 1 && randomSoundProbability >= 0; // Valid probability range.
  boolean isValidPhase = phase >= 0; // Phase must be non-negative.

  // If all conditions are met, the input is considered valid.
  doBeep = isValidPitch && isValidSlope && isValidSoundToSilenceRatio && isCycleRatePositive && isValidStandardDeviation && isValidRandomSoundProbability && isValidPhase;
}

// Sets the durations for sound and silence based on the ratio and cycle rate.
void SetBeepDurations(double soundToSilenceRatio, double cycleRate, double cycleStandardDeviation)
{
  silenceDuration = (1 - soundToSilenceRatio) / cycleRate; // Calculate base silence duration.
  silenceDuration += GenerateGaussian(cycleStandardDeviation); // Add Gaussian randomness.
  CapNumber(&silenceDuration, 0, 1 / cycleRate); // Ensure the duration is within valid bounds.

  soundDuration = 1 / cycleRate - silenceDuration; // Calculate sound duration based on the remaining time.
}

// Sets the target pitch for the beep, including randomness.
void SetTargetPitch(double pitch, double pitchStandardDeviation)
{
  targetPitch = pitch + GenerateGaussian(pitchStandardDeviation); // Add randomness to the pitch.
  CapNumber(&targetPitch, 80, 3000); // Ensure pitch is within a valid range.

  currentPitch = targetPitch; // Initialize current pitch to target pitch for the start of behavior.
}

// Plays a sound or silence based on a given probability.
void PlayRandomSoundWithProbability(double slope, double randomSoundProbability, double pitchStandardDeviation)
{
  double randomNumber = GetRandomNumber(0, 1); // Generate a random number to compare against probability.

  double randomPitch; // Variable for the pitch of the random sound.
  if (slope == 0) {
    randomPitch = targetPitch + GenerateGaussian(pitchStandardDeviation); // Generate random pitch variation.
  } else {
    int randomSemitone = (int)GetRandomNumber(1, 12); // Random semitone for pitch variation.
    randomPitch = exp(log(targetPitch) + slope * randomSemitone / 12 * log(2)); // Calculate pitch based on semitone change.
  }

  if (randomNumber < randomSoundProbability) {
    buzzer.tone(randomPitch, silenceDuration * 1000); // Play the random sound if within probability.
  } else {
    _delay(silenceDuration); // Otherwise, maintain silence for the duration.
  }
}

// Method to execute the beep base behavior
void Beep(double duration,
          double beepPitch, double beepSlope, double beepSoundToSilenceRatio, double beepCycleRate, double beepCycleStandardDeviation, double beepPitchStandardDeviation, double beepRandomSoundProbability, double beepPhase) 
{
  // INPUT CHECKS AND VARIABLE INITIALIZATION

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

    // If valid beep input is given
    if (doBeep) {
      // Start executing after the given phase
      if (getLastTime() - beepPhase >= 0) {
        // Check if the beep slope is zero (constant pitch)
        if (beepSlope == 0) {
          if (getLastTime() - beepPhase < beepCycle / beepCycleRate + soundDuration) {
            // Play the sound at the current pitch for the given duration
            buzzer.tone(currentPitch, soundDuration * 1000);
          }
        }

        // Check if the beep slope is positive (rising pitch)
        if (beepSlope > 0) {
          if(getLastTime() - beepPhase < beepCycle / beepCycleRate + semitone * soundDuration / 12) {
            // Play the sound at the current pitch for the given duration
            buzzer.tone(currentPitch, soundDuration / 12 * 1000);
          } else {
            // Change current pitch to the next rising semitone
            currentPitch = exp(log(targetPitch) + beepSlope * semitone / 12 * log(2));
            
            // Increase semitone by one
            if (semitone < 12) {
              semitone += 1;
            }
          }

        }

        // Check if the beep slope is negative (falling pitch)
        if (beepSlope < 0) {
          if (getLastTime() - beepPhase < beepCycle / beepCycleRate + semitone * soundDuration / 12) {
            // Play the sound at the current pitch for the given duration
            buzzer.tone(currentPitch, soundDuration / 12 * 1000);
          } else {
            // Change current pitch to the next falling semitone
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

        // Check if the beep cycle has finished
        if (getLastTime() - beepPhase > (beepCycle + 1) / beepCycleRate) {
          // Increase the count of cycles by one
          beepCycle += 1;

          // Reset semitone to one
          semitone = 1;

          // Change the target pitch
          SetTargetPitch(beepPitch, beepPitchStandardDeviation);

          // Change sound and silence durations
          SetBeepDurations(beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation);
        }
      }
    }
  }
}

// Setup function to initialize the robot and start the wander behavior
void setup() 
{
  randomSeed(0); // Initialize the random number generator seed.

  // Start the beeping behavior with the specified parameters.
  Beep(duration, beepPitch, beepSlope, beepSoundToSilenceRatio, beepCycleRate, beepCycleStandardDeviation, beepPitchStandardDeviation, beepRandomSoundProbability, beepPhase);
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