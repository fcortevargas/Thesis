// generated by mBlock5 for mBot
// codes make you happy

#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

double currentTime = 0;
double lastTime = 0;
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

void _delay(float seconds) {
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}

void setup() {
  rgbled_7.fillPixelsBak(0, 2, 1);
  SmoothTurnOnLights_B_B_N_N_N(1.000000 == 1.000000, 1.000000 == 1.000000, 255, 85, 0);
  _delay(1);
  SmoothChangeColors_B_B_N_N_N(1.000000 == 1.000000, 1.000000 == 1.000000, 0, 255, 255);
  _delay(1);
  SetLightIntensities_B_B_N_N_N(1.000000 == 1.000000, 1.000000 == 1.000000, 255, 255, 0);
  _delay(1);
  SetLightIntensities_B_B_N_N_N(1.000000 == 1.000000, 1.000000 == 0.000000, 0, 255, 0);
  _delay(1);
  SetLightIntensities_B_B_N_N_N(1.000000 == 0.000000, 1.000000 == 1.000000, 0, 255, 0);
  _delay(1);
  SmoothChangeColors_B_B_N_N_N(1.000000 == 1.000000, 1.000000 == 1.000000, 255, 255, 255);
  _delay(1);
  SmoothTurnOffLights_B_B(1.000000 == 1.000000, 1.000000 == 1.000000);

}

void _loop() {
}

void loop() {
  _loop();
}