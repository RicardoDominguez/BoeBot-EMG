//Code for the BoetBot with the claw attached

//Recieve bytes from ESP8266 through serial and interprete it as a set of
//actions for the motors and the claw

// Necessary ibraries
#include <Servo.h>
#include <BoeBot.h>

BoeBot MyBoeBot;

//------------------------------------------------------------------------------
// Variables that the user might want to modify
//------------------------------------------------------------------------------
#define MOTOR_SPEED   3   //Speed at which the motors will operate (from 1 to 10,
                            //10 beign the maximum speed)
#define MULT_SPEEDS   0   //1 if BoeBot will recieve signals to only move forward
                            //at different speeds. 0 if BoeBot will move at const
                            //speed forward/backward.
#define debug         0   //Echos debugg messages to the serial if TRUE

//Set up BoeBot and Serial
void setup()
{
  Serial.begin(9600);

  MyBoeBot.boebot_setup();
  MyBoeBot.stop(); //Make sure the robot starts still
  MyBoeBot.openClaw(); //The initial claw state is opened
}

void loop()
{
  //Variables used
  static int byte_read;

  //If there is an avaliable reading, interpret it
  if (Serial.available()){
    byte_read = Serial.read();
    MyBoeBot.interpretEMGserial(byte_read, MOTOR_SPEED, MULT_SPEEDS, debug);
  }
}
