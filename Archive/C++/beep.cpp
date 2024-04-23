// generated by mBlock5 for mBot
// codes make you happy

#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

MeBuzzer buzzer;

float soundDuration = 0;
float silenceDuration = 0;
float currentPitch = 0;
float numberOfBeeps = 0;

void Beep_N_N_N_S(double Pitch, double Tempo, double Duration, String Intonation){
  if(Duration * Tempo > 1){
    numberOfBeeps = Duration * Tempo;

  }else{
    numberOfBeeps = 1;

  }
  if(((Duration / Tempo == 1.000000)  ||  (Tempo < 1))  ||  (numberOfBeeps == 1.000000)){
    silenceDuration = 0;

  }else{
    if((Tempo > 1)  &&  (Tempo < 20)){
      silenceDuration = (1 / Tempo) * 0.2;

    }else{
      silenceDuration = (1 / Tempo) * 0.5;

    }

  }
  if(Tempo > 1){
    soundDuration = ((1 / Tempo - silenceDuration));

  }else{
    soundDuration = Duration;

  }
  for(int count3=0;count3<int(numberOfBeeps);count3++){
    if(Intonation == "Flat"){

      buzzer.tone(Pitch, soundDuration * 1000);
      _delay(float(silenceDuration));

    }
    if(Intonation == "Falling"){
      currentPitch = (Pitch + 100);
      for(int count=0;count<200;count++){

        buzzer.tone(currentPitch, soundDuration / 200 * 1000);
        currentPitch = ((currentPitch - 1));
      }
      _delay(float(silenceDuration));

    }
    if(Intonation == "Rising"){
      currentPitch = ((Pitch - 100));
      for(int count2=0;count2<200;count2++){

        buzzer.tone(currentPitch, soundDuration / 200 * 1000);
        currentPitch = (currentPitch + 1);
      }
      _delay(float(silenceDuration));

    }
  }

}

void _delay(float seconds) {
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}

void setup() {
  Beep_N_N_N_S(600, 0.1, 0.2, "Falling");
  Beep_N_N_N_S(500, 1, 0.25, "Flat");
  Beep_N_N_N_S(400, 1, 0.3, "Flat");
  Beep_N_N_N_S(600, 0.5, 0.25, "Rising");
  Beep_N_N_N_S(700, 1, 0.3, "Flat");
  Beep_N_N_N_S(500, 1, 0.1, "Flat");
  Beep_N_N_N_S(300, 1, 0.2, "Flat");
  Beep_N_N_N_S(200, 2, 0.1, "Flat");

}

void _loop() {
}

void loop() {
  _loop();
}
