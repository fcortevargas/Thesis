// generated by mBlock5 for mBot
// codes make you happy

#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

double currentTime = 0;
double lastTime = 0;

// Motion

MeDCMotor motor_9(9);
MeDCMotor motor_10(10);

float current_power = 0;

void move(int direction, int speed) {
  int leftSpeed = 0;
  int rightSpeed = 0;
  if(direction == 1) {
    leftSpeed = speed;
    rightSpeed = speed;
  } else if(direction == 2) {
    leftSpeed = -speed;
    rightSpeed = -speed;
  } else if(direction == 3) {
    leftSpeed = -speed;
    rightSpeed = speed;
  } else if(direction == 4) {
    leftSpeed = speed;
    rightSpeed = -speed;
  }
  motor_9.run((9) == M1 ? -(leftSpeed) : (leftSpeed));
  motor_10.run((10) == M1 ? -(rightSpeed) : (rightSpeed));
}
void MoveForward_N_N(double power, double duration){

  move(1, power / 100.0 * 255);
  _delay(duration);
  move(1, 0);

}
void AccelerateForward_N_N_N(double start_power, double end_power, double duration){
  current_power = start_power;
  while(!(current_power == end_power))
  {
    _loop();

    move(1, current_power / 100.0 * 255);
    _delay(duration / 5);
    move(1, 0);
    current_power += ((end_power - start_power)) / 5;

  }

}

void MoveJerky_N_N_N(double jerkiness, double power, double duration){
  current_power = power;

  move(1, 100 / 100.0 * 255);
  for(int count=0;count<int((jerkiness * duration) / (0.2 * current_power));count++){
    lastTime = millis() / 1000.0;
    while(!(getLastTime() > (0.1 * current_power) / jerkiness))
    {
      _loop();

      move(3, current_power / 100.0 * 255);

    }
    lastTime = millis() / 1000.0;
    while(!(getLastTime() > (0.1 * current_power) / jerkiness))
    {
      _loop();

      move(4, current_power / 100.0 * 255);

    }
  }

}
void MoveRound_B_B_N_N_N(boolean left, boolean right, double roundness, double power, double duration){
  lastTime = millis() / 1000.0;
  current_power = power;
  while(!(getLastTime() == duration))
  {
    _loop();
    if(left){

      motor_9.run(-1 * current_power/100.0*255);
      motor_10.run((current_power - roundness)/100.0*255);

    }
    if((right)  &&  (!(left))){

      motor_9.run(-1 * (current_power - roundness)/100.0*255);
      motor_10.run(current_power/100.0*255);

    }

  }

}
void StopMoving(){
  motor_9.run(0);
  motor_10.run(0);
  current_power = 0;

}

// Light

MeRGBLed rgbled_7(7, 2);

float intensity_blue = 0;
float intensity_red = 0;
float intensity_green = 0;

double getLastTime(){
  return currentTime = millis() / 1000.0 - lastTime;
}
void SmoothTurnOffLights_B_B(boolean left, boolean right){
  while(!((intensity_red == 0.000000)  &&  ((intensity_green == 0.000000)  &&  (intensity_blue == 0.000000))))
  {
    _loop();
    if(intensity_red > 0){
      intensity_red += -1;

    }
    if(intensity_green > 0){
      intensity_green += -1;

    }
    if(intensity_blue > 0){
      intensity_blue += -1;

    }
    lastTime = millis() / 1000.0;
    while(!(getLastTime() > 0.001))
    {
      _loop();
    }
    if((left)  &&  (right)){

      rgbled_7.setColor(0, intensity_red, intensity_green, intensity_blue);
      rgbled_7.show();

    }
    if(left){

      rgbled_7.setColor(2, intensity_red, intensity_green, intensity_blue);
      rgbled_7.show();

    }
    if(right){

      rgbled_7.setColor(1, intensity_red, intensity_green, intensity_blue);
      rgbled_7.show();

    }

  }

}
void SmoothTurnOnLights_B_B_N_N_N(boolean left, boolean right, double R, double G, double B){
  intensity_red = 0;
  intensity_green = 0;
  intensity_blue = 0;
  while(!((intensity_red == R)  &&  ((intensity_green == G)  &&  (intensity_blue == B))))
  {
    _loop();
    if(intensity_red < R){
      intensity_red += 1;

    }
    if(intensity_green < G){
      intensity_green += 1;

    }
    if(intensity_blue < B){
      intensity_blue += 1;

    }
    lastTime = millis() / 1000.0;
    while(!(getLastTime() > 0.001))
    {
      _loop();
    }
    if((left)  &&  (right)){

      rgbled_7.setColor(0, intensity_red, intensity_green, intensity_blue);
      rgbled_7.show();

    }
    if(left){

      rgbled_7.setColor(2, intensity_red, intensity_green, intensity_blue);
      rgbled_7.show();

    }
    if(right){

      rgbled_7.setColor(1, intensity_red, intensity_green, intensity_blue);
      rgbled_7.show();

    }

  }

}
void SmoothChangeColors_B_B_N_N_N(boolean left, boolean right, double R, double G, double B){
  while(!((intensity_red == R)  &&  ((intensity_green == G)  &&  (intensity_blue == B))))
  {
    _loop();
    if(intensity_red < R){
      intensity_red += 1;

    }else{
      if(intensity_red > R){
        intensity_red += -1;

      }

    }
    if(intensity_green < G){
      intensity_green += 1;

    }else{
      if(intensity_green > G){
        intensity_green += -1;

      }

    }
    if(intensity_blue < B){
      intensity_blue += 1;

    }else{
      if(intensity_blue > B){
        intensity_blue += -1;

      }

    }
    lastTime = millis() / 1000.0;
    while(!(getLastTime() > 0.001))
    {
      _loop();
    }
    if((left)  &&  (right)){

      rgbled_7.setColor(0, intensity_red, intensity_green, intensity_blue);
      rgbled_7.show();

    }
    if(left){

      rgbled_7.setColor(2, intensity_red, intensity_green, intensity_blue);
      rgbled_7.show();

    }
    if(right){

      rgbled_7.setColor(1, intensity_red, intensity_green, intensity_blue);
      rgbled_7.show();

    }

  }

}
void SetLightIntensities_B_B_N_N_N(boolean left, boolean right, double R, double G, double B){
  intensity_red = R;
  intensity_green = G;
  intensity_blue = B;
  if((left)  &&  (right)){

    rgbled_7.setColor(0, intensity_red, intensity_green, intensity_blue);
    rgbled_7.show();

  }
  if(left){

    rgbled_7.setColor(2, intensity_red, intensity_green, intensity_blue);
    rgbled_7.show();

  }
  if(right){

    rgbled_7.setColor(1, intensity_red, intensity_green, intensity_blue);
    rgbled_7.show();

  }

}

// Sound

MeBuzzer buzzer;

void Silence_N(double duration){

  buzzer.tone(0, duration * 1000);
  _delay(float(duration));

}
void HighPitchRisingNotes_N(double duration){

  buzzer.tone(1568, duration * 1000);
  _delay(0.02);

  buzzer.tone(1760, duration * 1000);
  _delay(0.02);

  buzzer.tone(1976, duration * 1000);
  _delay(0.02);

  buzzer.tone(2093, duration * 1000);
  _delay(0.02);

  buzzer.tone(2349, duration * 1000);
  _delay(0.02);

  buzzer.tone(2637, duration * 1000);
  _delay(0.02);

}
void HighPitchRisingFallingNotes_N(double duration){

  buzzer.tone(1568, duration * 1000);
  _delay(0.02);

  buzzer.tone(1760, duration * 1000);
  _delay(0.02);

  buzzer.tone(1976, duration * 1000);
  _delay(0.02);

  buzzer.tone(2093, duration * 1000);
  _delay(0.02);

  buzzer.tone(2349, duration * 1000);
  _delay(0.02);

  buzzer.tone(2637, duration * 1000);
  _delay(0.02);

  buzzer.tone(2349, duration * 1000);
  _delay(0.02);

  buzzer.tone(2093, duration * 1000);
  _delay(0.02);

  buzzer.tone(1976, duration * 1000);
  _delay(0.02);

  buzzer.tone(1760, duration * 1000);
  _delay(0.02);

  buzzer.tone(1568, duration * 1000);
  _delay(0.02);

}
void HighPitchFallingRisingNotes_N(double duration){

  buzzer.tone(2637, duration * 1000);
  _delay(0.02);

  buzzer.tone(2349, duration * 1000);
  _delay(0.02);

  buzzer.tone(2093, duration * 1000);
  _delay(0.02);

  buzzer.tone(1976, duration * 1000);
  _delay(0.02);

  buzzer.tone(1760, duration * 1000);
  _delay(0.02);

  buzzer.tone(1568, duration * 1000);
  _delay(0.02);

  buzzer.tone(1760, duration * 1000);
  _delay(0.02);

  buzzer.tone(1976, duration * 1000);
  _delay(0.02);

  buzzer.tone(2093, duration * 1000);
  _delay(0.02);

  buzzer.tone(2349, duration * 1000);
  _delay(0.02);

  buzzer.tone(2637, duration * 1000);
  _delay(0.02);

}
void HighPitchFallingNotes_N(double duration){

  buzzer.tone(2637, duration * 1000);
  _delay(0.02);

  buzzer.tone(2349, duration * 1000);
  _delay(0.02);

  buzzer.tone(2093, duration * 1000);
  _delay(0.02);

  buzzer.tone(1976, duration * 1000);
  _delay(0.02);

  buzzer.tone(1760, duration * 1000);
  _delay(0.02);

  buzzer.tone(1568, duration * 1000);
  _delay(0.02);

}
void MediumPitchRisingNotes_N(double duration){

  buzzer.tone(392, duration * 1000);
  _delay(0.02);

  buzzer.tone(440, duration * 1000);
  _delay(0.02);

  buzzer.tone(494, duration * 1000);
  _delay(0.02);

  buzzer.tone(523, duration * 1000);
  _delay(0.02);

  buzzer.tone(587, duration * 1000);
  _delay(0.02);

  buzzer.tone(659, duration * 1000);
  _delay(0.02);

}
void MediumPitchFallingNotes_N(double duration){

  buzzer.tone(330, duration * 1000);
  _delay(0.02);

  buzzer.tone(294, duration * 1000);
  _delay(0.02);

  buzzer.tone(262, duration * 1000);
  _delay(0.02);

  buzzer.tone(247, duration * 1000);
  _delay(0.02);

  buzzer.tone(220, duration * 1000);
  _delay(0.02);

  buzzer.tone(196, duration * 1000);
  _delay(0.02);

}
void MediumPitchFallingRisingNotes_N(double duration){

  buzzer.tone(330, duration * 1000);
  _delay(0.02);

  buzzer.tone(294, duration * 1000);
  _delay(0.02);

  buzzer.tone(262, duration * 1000);
  _delay(0.02);

  buzzer.tone(247, duration * 1000);
  _delay(0.02);

  buzzer.tone(220, duration * 1000);
  _delay(0.02);

  buzzer.tone(196, duration * 1000);
  _delay(0.02);

  buzzer.tone(220, duration * 1000);
  _delay(0.02);

  buzzer.tone(247, duration * 1000);
  _delay(0.02);

  buzzer.tone(262, duration * 1000);
  _delay(0.02);

  buzzer.tone(294, duration * 1000);
  _delay(0.02);

  buzzer.tone(330, duration * 1000);
  _delay(0.02);

}
void MediumPitchRisingFallingNotes_N(double duration){

  buzzer.tone(196, duration * 1000);
  _delay(0.02);

  buzzer.tone(220, duration * 1000);
  _delay(0.02);

  buzzer.tone(247, duration * 1000);
  _delay(0.02);

  buzzer.tone(262, duration * 1000);
  _delay(0.02);

  buzzer.tone(294, duration * 1000);
  _delay(0.02);

  buzzer.tone(330, duration * 1000);
  _delay(0.02);

  buzzer.tone(294, duration * 1000);
  _delay(0.02);

  buzzer.tone(262, duration * 1000);
  _delay(0.02);

  buzzer.tone(247, duration * 1000);
  _delay(0.02);

  buzzer.tone(220, duration * 1000);
  _delay(0.02);

  buzzer.tone(196, duration * 1000);
  _delay(0.02);

}
void LowPitchRisingNotes_N(double duration){

  buzzer.tone(98, duration * 1000);
  _delay(0.02);

  buzzer.tone(110, duration * 1000);
  _delay(0.02);

  buzzer.tone(123, duration * 1000);
  _delay(0.02);

  buzzer.tone(131, duration * 1000);
  _delay(0.02);

  buzzer.tone(147, duration * 1000);
  _delay(0.02);

  buzzer.tone(165, duration * 1000);
  _delay(0.02);

}
void LowPitchFallingNotes_N(double duration){

  buzzer.tone(165, duration * 1000);
  _delay(0.02);

  buzzer.tone(147, duration * 1000);
  _delay(0.02);

  buzzer.tone(131, duration * 1000);
  _delay(0.02);

  buzzer.tone(123, duration * 1000);
  _delay(0.02);

  buzzer.tone(110, duration * 1000);
  _delay(0.02);

  buzzer.tone(98, duration * 1000);
  _delay(0.02);

}
void LowPitchFallingRisingNotes_N(double duration){

  buzzer.tone(165, duration * 1000);
  _delay(0.02);

  buzzer.tone(147, duration * 1000);
  _delay(0.02);

  buzzer.tone(131, duration * 1000);
  _delay(0.02);

  buzzer.tone(123, duration * 1000);
  _delay(0.02);

  buzzer.tone(110, duration * 1000);
  _delay(0.02);

  buzzer.tone(98, duration * 1000);
  _delay(0.02);

  buzzer.tone(110, duration * 1000);
  _delay(0.02);

  buzzer.tone(123, duration * 1000);
  _delay(0.02);

  buzzer.tone(131, duration * 1000);
  _delay(0.02);

  buzzer.tone(147, duration * 1000);
  _delay(0.02);

  buzzer.tone(165, duration * 1000);
  _delay(0.02);

}
void LowPitchRisingFallingNotes_N(double duration){

  buzzer.tone(98, duration * 1000);
  _delay(0.02);

  buzzer.tone(110, duration * 1000);
  _delay(0.02);

  buzzer.tone(123, duration * 1000);
  _delay(0.02);

  buzzer.tone(131, duration * 1000);
  _delay(0.02);

  buzzer.tone(147, duration * 1000);
  _delay(0.02);

  buzzer.tone(165, duration * 1000);
  _delay(0.02);

  buzzer.tone(147, duration * 1000);
  _delay(0.02);

  buzzer.tone(131, duration * 1000);
  _delay(0.02);

  buzzer.tone(123, duration * 1000);
  _delay(0.02);

  buzzer.tone(110, duration * 1000);
  _delay(0.02);

  buzzer.tone(98, duration * 1000);
  _delay(0.02);

}

// General

void _delay(float seconds) {
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}

void setup() {
  // Turn on both lights to red
  rgbled_7.fillPixelsBak(0, 2, 1);
  SmoothTurnOnLights_B_B_N_N_N(1.000000 == 1.000000, 1.000000 == 1.000000, 255, 0, 0);
  // High pitch rising notes
  HighPitchRisingFallingNotes_N(0.025);
  Silence_N(0.05);
  // Move Forward
  MoveForward_N_N(100, 1);

  // Smooth switch all lights to cyan
  SmoothChangeColors_B_B_N_N_N(1.000000 == 1.000000, 1.000000 == 1.000000, 0, 255, 255);
  // Play note
  buzzer.tone(880, 0.3 * 1000);
  _delay(0.02);
  Silence_N(0.05);
  // Move round
  MoveRound_B_B_N_N_N(1.000000 == 1.000000, 1.000000 == 0.000000, 60, 100, 5);

  // Switch all lights to yellow
  SetLightIntensities_B_B_N_N_N(1.000000 == 1.000000, 1.000000 == 1.000000, 255, 255, 0);
  // Medium pitch falling-rising notes
  MediumPitchFallingRisingNotes_N(0.025);
  Silence_N(0.05);
  // Move forward
  MoveForward_N_N(100, 1);

  // Switch left light to green
  SetLightIntensities_B_B_N_N_N(1.000000 == 1.000000, 1.000000 == 0.000000, 0, 255, 0);
  // Low pitch rising notes
  LowPitchRisingNotes_N(0.05);
  Silence_N(0.05);
  // Move jerky
  MoveJerky_N_N_N(100, 100, 5);

  // Switch left light to green
  SetLightIntensities_B_B_N_N_N(1.000000 == 0.000000, 1.000000 == 1.000000, 0, 255, 0);
  // Medium pitch falling notes 
  MediumPitchFallingNotes_N(0.05);
  Silence_N(0.05);
  // Move forward
  MoveForward_N_N(100, 1);

  // Smooth turn off all lights
  SmoothTurnOffLights_B_B(1.000000 == 1.000000, 1.000000 == 1.000000);
  // Stop moving
  StopMoving();

}

void _loop() {
}

void loop() {
  _loop();
}