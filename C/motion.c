// generated by mBlock5 for mBot
// codes make you happy

#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

double currentTime = 0;
double lastTime = 0;
MeDCMotor motor_9(9);
MeDCMotor motor_10(10);
MeBuzzer buzzer;
MeRGBLed rgbled_7(7, 2);

float current_power = 0;

double getLastTime(){
  return currentTime = millis() / 1000.0 - lastTime;
}
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

void _delay(float seconds) {
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}

void setup() {
  rgbled_7.fillPixelsBak(0, 2, 1);
  lastTime = millis() / 1000.0;
  if(getLastTime() < 50){

    move(1, 50 / 100.0 * 255);

    buzzer.tone(700, 0.5 * 1000);

    buzzer.tone(800, 0.5 * 1000);

    buzzer.tone(900, 0.5 * 1000);

    buzzer.tone(1000, 0.5 * 1000);

  }
  MoveRound_B_B_N_N_N(1.000000 == 1.000000, 1.000000 == 0.000000, 20, 60, 5);

  buzzer.tone(1000, 0.5 * 1000);

  buzzer.tone(900, 0.5 * 1000);

  buzzer.tone(800, 0.5 * 1000);

  buzzer.tone(700, 0.5 * 1000);

  rgbled_7.setColor(0,255,0,0);
  rgbled_7.show();

  move(1, 50 / 100.0 * 255);
  MoveJerky_N_N_N(30, 30, 5);
  StopMoving();

}

void _loop() {
}

void loop() {
  _loop();
}

