// generated by mBlock5 for mBot
// codes make you happy

#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

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

void _delay(float seconds) {
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}

void setup() {
  HighPitchRisingFallingNotes_N(0.025);
  Silence_N(0.05);

  buzzer.tone(880, 0.3 * 1000);
  _delay(0.02);
  Silence_N(0.05);
  MediumPitchFallingRisingNotes_N(0.025);
  Silence_N(0.05);
  LowPitchRisingNotes_N(0.05);
  Silence_N(0.05);
  MediumPitchFallingNotes_N(0.05);
  Silence_N(0.05);

}

void _loop() {
}

void loop() {
  _loop();
}

