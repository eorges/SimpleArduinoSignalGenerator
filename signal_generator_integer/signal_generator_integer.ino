#include "math.h"

#define OUTPUT_1 5  // pin D5 will be output 1
#define OUTPUT_2 6  // pin D6 will be output 2
#define OUTPUT_3 3  // pin D6 will be output 3
#define OUTPUT_4 10 // pin D10 will be output 4
#define OUTPUT_5 11 // pin D11 will be output 5

#define TABLE_SIZE 128

uint32_t startTime, tau, timer1;
float value, pi, w, theta, f, Ts , sqWavePeriod;
uint8_t sqWaveValue;

float times[TABLE_SIZE];
uint8_t mySine[TABLE_SIZE];
uint8_t myCosine[TABLE_SIZE];
uint8_t myTriangle[TABLE_SIZE];

void setup() {
  pinMode(OUTPUT_1, OUTPUT);
  pinMode(OUTPUT_2, OUTPUT);
  pinMode(OUTPUT_3, OUTPUT);
  pinMode(OUTPUT_4, OUTPUT);
  pinMode(OUTPUT_5, OUTPUT);

  Serial.begin(115200);
  startTime = millis();
  tau = 10 * 1000; //10 secondi
  pi = 104348 / 33215;
  f = 3; // 3 Hz
  Ts = 1 / f;
  theta = pi * 0.5;
  w = 2 * pi * f;

  sqWavePeriod = 0.01 * 1000; // 1 secondo
  sqWaveValue = 1;

  for ( int j = 0 ; j < TABLE_SIZE; ++j)
  {
    times[j] = j * Ts;
  }

  for ( int j = 0 ; j < TABLE_SIZE; ++j)
  {
    mySine[j] = myMap(0.5 + 0.5 * sin(w * times[j] + theta), 0, 1, 0 , 255);

  }
  for ( int j = 0 ; j < TABLE_SIZE; ++j)
  {
    myCosine[j] = myMap(0.5 + 0.5 * cos(w * times[j] + theta), 0, 1, 0 , 255);
  }

    for ( int j = 0 ; j < TABLE_SIZE; ++j)
  {
    myTriangle[j] = myMap((2 * abs((j % (TABLE_SIZE / 2)) - TABLE_SIZE / 4)) / float(TABLE_SIZE / 2), 0, 1, 0, 255);
  }

}

void loop() {

/*
  //Step function

  if (millis() - startTime >= tau)
  {
    value = 255;
  }
  else
  {
    value = 0;
  }
  Serial.println(value);
  analogWrite(OUTPUT_3, value ); // 255 here corresponds to 5 volts
*/


  /*
    // Sine function
      for (int i = 0; i < TABLE_SIZE; ++i) {
      Serial.println(mySine[i]);
      analogWrite(OUTPUT_1,mySine[i] );
    }
  */


  /*
    // Cosine function

      for (int i = 0; i < TABLE_SIZE; ++i) {
      Serial.println(myCosine[i]);
      analogWrite(OUTPUT_2, myCosine[i] );
    }
  */


/*
  // Square Wave

  Serial.println(myMap(sqWaveValue, 0, 1, 0, 255 ));
  analogWrite(OUTPUT_4, myMap(sqWaveValue, 0, 1, 0, 255 ));
  if ( millis() - timer1 >= sqWavePeriod)
  {
    timer1 = millis();
    sqWaveValue = !sqWaveValue;
  }
*/

  // Triangular wave
      for (int i = 0; i < TABLE_SIZE; ++i) {
      Serial.println(myTriangle[i]);
      analogWrite(OUTPUT_5, myTriangle[i] );
    }
}

uint8_t myMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
