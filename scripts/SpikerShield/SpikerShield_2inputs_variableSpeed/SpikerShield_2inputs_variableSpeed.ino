//Read EMG, two input, n_speeds states (6 default)

//1. Read muscle flext through EMG for two inputs (two arms)
//2. Divide the signal into n_speeds states, associated with different levels of
//   speed including the stop state.
//3. Write the appropiate discrete state to Serial, connected to the ESP8266 wifi
//   module

// Necessary ibraries
#include <BoeBot.h>
BoeBot MyBoeBot;

//------------------------------------------------------------------------------
// Variables that the user might want to modify
//------------------------------------------------------------------------------
int channels[2] = {A0, A5}; //Analog channel from which the EMG signal is read
                            //channel[0] for left hand
                            //channel[1] for right hand
#define NUMBER_REQUEST_B4_SWITCH 0 //The robot has to recieve NUMBER_REQUEST_B4_SWITCH
                                   // signals indicating to change its action before it
                                   //actually sends a different action. //This gives
                                   // consistency to the output against noisy signals
#define n_speeds 8    //Number of different speeds to be considered (including stop state)
                           //From 2 to 10.
#define baseline 0    //Lowest speed possible (from 0 to 10-n_speeds)

const float calibration_bias = 0.5; //The actual reading max will be max reading * calibration_bias
#define calibration_time 2000 //Miliseconds of sampling per arm

const int debug = 0; //If TRUE prints additional messages to screen

//------------------------------------------------------------------------------
// Variables used throughout the code
//------------------------------------------------------------------------------

//To store the thresholds from calibration
int thresholds[2 * n_speeds];

//Constants used for serial comunication
const int d_stop = MyBoeBot.d_stop;
//Action that will be sent to the ESP8266 through serial
int action_to_send[2] = {d_stop, d_stop};


void setup(){
  Serial.begin(9600);

  //Calibrate device
  MyBoeBot.calibrate_N_states(calibration_time, n_speeds, channels, thresholds,
      calibration_bias);
}

void loop()
{
    for(int arm=0; arm<2; arm++){
      //Get reading
      int analogReadings = MyBoeBot.analogSample(30, channels[arm], 0);
      //Get flex intenstity (from 0 to n_speeds-1)
      int motor_action = n_speeds-1;
      for(int state = 0; state < n_speeds-1; state++){
        if (analogReadings < thresholds[arm*n_speeds + state]){
          motor_action = state;
          break;
        }
      }

    if(motor_action == 0){
      action_to_send[arm] = 0;
    } else {
      action_to_send[arm] = motor_action + baseline;
    }
  }

  //What will be sent to ESP8266
  int serial_send = action_to_send[1]*10 + action_to_send[0];

  if(debug == 1){
    Serial.println(serial_send);
  } else {
    Serial.write(serial_send);
  }
}
