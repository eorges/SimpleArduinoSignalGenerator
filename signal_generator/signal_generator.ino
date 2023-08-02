#include "math.h"

#define OUTPUT_PIN 5  // pin D5 will be the output
#define TABLE_SIZE 64

uint32_t startTime,tau;
float value, pi, w, theta,f,Ts;

float times[TABLE_SIZE];
float mySine[TABLE_SIZE];
float myCosine[TABLE_SIZE];

void setup() {
  pinMode(OUTPUT_PIN, OUTPUT);
  Serial.begin(115200);
  startTime = millis();
  tau = 10*1000; //10 secondi
  pi = 104348/33215;
  f = 3; // 3 Hz
  Ts = 1/f;
  theta = pi*0.5;
  w = 2*pi*f;

    for ( int j = 0 ; j<TABLE_SIZE; ++j)
  {
    times[j] = j*Ts;
  }

  for ( int j = 0 ; j<TABLE_SIZE; ++j)
  {
    mySine[j] = 0.5 + 0.5*sin(w*times[j] + theta);
  }
    for ( int j = 0 ; j<TABLE_SIZE; ++j)
  {
    myCosine[j] =0.5 + 0.5*cos(w*times[j] + theta);
  }
}

void loop() {

  //Step function
/*
  if (millis() - startTime >= tau)
  {
    value = 1;
  }
  else
  {
    value = 0;
  }
  Serial.println(value);
  analogWrite(OUTPUT_PIN, myMap(value, 0, 1, 0 , 255) ); // 255 here corresponds to 5 volts
*/
  // Sine function
    for (int i = 0; i < TABLE_SIZE; ++i) {
    Serial.println(mySine[i]);
    analogWrite(OUTPUT_PIN, myMap(mySine[i], 0, 1, 0 , 255) );
  }


  // Cosine function 
  /*
    for (int i = 0; i < TABLE_SIZE; ++i) {
    Serial.println(myCosine[i]);
    analogWrite(OUTPUT_PIN, myMap(myCosine[i], 0, 1, 0 , 255) );

  */

}


float myMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
