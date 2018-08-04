//Script to familiarize user with the SpikerBox
// 1. Read muscle EMG
// 2. Print to screen the value of the reading

// Necessary ibraries
#include <BoeBot.h>
BoeBot MyBoeBot;

//------------------------------------------------------------------------------
// Variables that the user might want to modify
//------------------------------------------------------------------------------
#define CHANNEL  A0  //SpikerBox analog channel
#define SAMPLE_TIME 25 //Sampling time

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  //1. Measure EMG
  int analogReading = MyBoeBot.analogSample(SAMPLE_TIME, CHANNEL, 0);

  //Print EMG raw data
  Serial.println(analogReading);
  delay(175);
}
