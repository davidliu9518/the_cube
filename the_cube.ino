//https://www.youtube.com/watch?v=bqfPZXEuyuc&t=715s
//Code for shift registers

/*******************************************************************************

  Bare Conductive Proximity LED Fader
  -----------------------------------

  Prox_LED.ino - maps the intensity of the onboard D13 LED on the Touch Board to
  the proximity of a hand to an 85mm square electrode connected to E0 on the board

  Based on code by Jim Lindblom and plenty of inspiration from the Freescale
  Semiconductor datasheets and application notes.

  Bare Conductive code written by Stefan Dzisiewski-Smith.

  This work is licensed under a MIT license https://opensource.org/licenses/MIT

  Copyright (c) 2016, Bare Conductive

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*******************************************************************************/
//used prox_LED code from Bare Conductive

#include <MPR121.h>
#include <Wire.h>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */


#ifdef __AVR__
#include <avr/power.h>
#endif


// mapping and filter definitions
#define LOW_DIFF 0
#define HIGH_DIFF 100
#define filterWeight 0.3f // 0.0f to 1.0f - higher value = more smoothing
float lastProx = 0;

// the electrode to monitor
#define ELECTRODE 0
int DS_pin = 10;
int STCP_pin = 11;
int SHCP_pin = 12;
int matrix_size = 8;
int num_solenoids = matrix_size * matrix_size;
int delayval = 100; // default delay for 0.1s
boolean registers[64]; //64 solenoids
float a;
boolean touch_flag = 0;
int touch_cycle_ctr = 0;
double start;
double diff_t;

void setup()
{
  //Initialize MPR121
  Serial.begin(9600);
  //while(!Serial);
  if (!MPR121.begin(0x5A)) {
    Serial.println("error setting up MPR121");
    switch (MPR121.getError()) {
      case NO_ERROR:
        Serial.println("no error");
        break;
      case ADDRESS_UNKNOWN:
        Serial.println("incorrect address");
        break;
      case READBACK_FAIL:
        Serial.println("readback failure");
        break;
      case OVERCURRENT_FLAG:
        Serial.println("overcurrent on REXT pin");
        break;
      case OUT_OF_RANGE:
        Serial.println("electrode out of range");
        break;
      case NOT_INITED:
        Serial.println("not initialised");
        break;
      default:
        Serial.println("unknown error");
        break;
    }
    while (1);
  }

  // slow down some of the MPR121 baseline filtering to avoid
  // filtering out slow hand movements
  MPR121.setRegister(MPR121_NHDF, 0x01); //noise half delta (falling)
  MPR121.setRegister(MPR121_FDLF, 0x3F); //filter delay limit (falling)


  // this is the touch threshold - setting it low makes it more like a proximity trigger
  // default value is 40 for touch
  MPR121.setTouchThreshold(40);

  // this is the release threshold - must ALWAYS be smaller than the touch threshold
  // default value is 20 for touch
  MPR121.setReleaseThreshold(20);

  // initial data update
  MPR121.updateTouchData();

  //shift registers
  pinMode(DS_pin, OUTPUT);
  pinMode(STCP_pin, OUTPUT);
  pinMode(SHCP_pin, OUTPUT);

  writereg();

  for (int i = 0; i < num_solenoids; i++)
  {
    registers[i] = HIGH;
    delay(20);
    writereg();
  }

  for (int i = num_solenoids - 1; i > 0; i--)
  {

    registers[i] = LOW;
    delay(20);
    writereg();

  }//lit one by one and then turn off one by one. in order.

}


void writereg()
{
  digitalWrite(STCP_pin, LOW);

  for (int i = num_solenoids - 1; i >= 0; i--)
  {
    digitalWrite(SHCP_pin, LOW);
    digitalWrite(DS_pin, registers[i] );
    digitalWrite(SHCP_pin, HIGH);
  }

  digitalWrite(STCP_pin, HIGH);
}

void random_propagation() {
  for (int i = 0; i < num_solenoids; i++)
  {
    registers[i] = 0; //generating random array. for random motion
  }
  int r = rand() % matrix_size;
  int c = rand() % matrix_size; //0-7
  //random coordinates
  int square_size = rand() % matrix_size + 1; //1-8
  int l = min((matrix_size - 1 - r), (matrix_size - 1 - c));
  square_size = min(square_size, l);
  for (int i = r; i <= r + square_size; i++) {
    for (int j = c; j <= c + square_size; j++) {
      registers[i * matrix_size + j] = 1;
    }
  }

  writereg();
  delay(1000);

  for (int i = 0; i < num_solenoids; i++)
  {
    registers[i] = 0; //generating random array. for random motion
  }
  writereg();

}

void loop()
{
  // update all of the data from the MPR121
  MPR121.updateAll();

  // read the difference between the measured baseline and the measured continuous data
  int reading = MPR121.getBaselineData(ELECTRODE) - MPR121.getFilteredData(ELECTRODE);

  // print out the reading value for debug
  //  Serial.println(reading);

  // constrain the reading between our low and high mapping values
  unsigned int prox = constrain(reading, LOW_DIFF, HIGH_DIFF);

  // implement a simple (IIR lowpass) smoothing filter
  lastProx = (filterWeight * lastProx) + ((1 - filterWeight) * (float)prox); \
  Serial.println(prox);

  //  // map the LOW_DIFF..HIGH_DIFF range to 0..255 (8-bit resolution for analogWrite)
  //  uint8_t thisOutput = (uint8_t)map(lastProx, LOW_DIFF, HIGH_DIFF, 0, 255);

  //   output the mapped value to the LED
  //    get proximity
  if (prox >= 90) {
    touch_flag = true;
  }
  if (!touch_flag) {
    //not been touched
    for (int i = 0; i < num_solenoids; i++)
    {
      registers[i] = (rand() > (RAND_MAX / 2)); //generating random array. for random motion
    }
    writereg();
    delay(max(200, 20 * lastProx));
  }
  else {
    for  (int i = 0; i < num_solenoids; i++)
    {
      registers[i] = 0; //zero
    }
    writereg();
    for (int i = 0; i < 1000; i++) {
      delay(10); //for loop as timer for 10s
      Serial.println(i);

      MPR121.updateTouchData();
      if (MPR121.getTouchData(0)) {
        Serial.print("electrode ");
        Serial.print(0);
        Serial.println(" was just touched");
        while (MPR121.getTouchData(0)) {//wait for release
          MPR121.updateTouchData();
          Serial.println("please release");
        }//released
        i = 0;
        random_propagation();
      }
    }

  }
  touch_flag = 0;

}
