//Read EMG, two input, three states

//1. Read muscle flext through EMG for two inputs (two arms)
//2. Divide the signal into three states, associated with STOP, FORWARD, BACKWARD
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
#define NUMBER_REQUEST_B4_SWITCH 2 //The robot has to recieve NUMBER_REQUEST_B4_SWITCH
                                   // signals indicating to change its action before it
                                   //actually sends a different action. //This gives
                                   // consistency to the output against noisy signals
//Calibration parameters
float upper_bias[2] = {0.8, 0.8};  //Higher bias means more preference towards
float lower_bias[2] = {0.6, 0.6}; //lower intensity values
#define calibration_time 1000 //Miliseconds of sampling per discrete state

const int debug = 0; //If TRUE prints additional messages to screen


//------------------------------------------------------------------------------
// Variables used throughout the code
//------------------------------------------------------------------------------
//Constants used for serial comunication
const int d_stop = MyBoeBot.d_stop;
const int d_forward = MyBoeBot.d_forward;
const int d_backward = MyBoeBot.d_backward;

//To store the thresholds from calibration
int upper_thresh[4], lower_thresh[4];

//Action that will be sent to the ESP8266 through serial
int action_to_send[2] = {d_stop, d_stop};
//Previous action sent
int last_motor_action[2] = {d_stop, d_stop};
//Number of times requested to change the action
int times_requested_switch[2] = {0, 0};

void setup(){
  Serial.begin(9600);

  //Calibrate device
  MyBoeBot.calibrate_2hands(3, calibration_time, channels, upper_bias,
        lower_bias, upper_thresh, lower_thresh);
}

void loop()
{
   //Thresholds used to decide the state of flex
   int emg_limit[2];

   for(int arm = 0; arm<2; arm++){
     if(action_to_send[arm] == d_stop){
        emg_limit[0] = upper_thresh[arm*2 + 0];
        emg_limit[1] = upper_thresh[arm*2 + 1];
     } else if (action_to_send[arm] == d_forward){
        emg_limit[0] = lower_thresh[arm*2 + 0];
        emg_limit[1] = upper_thresh[arm*2 + 1];
     } else if (action_to_send[arm] == d_backward){
        emg_limit[0] = lower_thresh[arm*2 + 0];
        emg_limit[1] = lower_thresh[arm*2 + 1];
     }

     //Measure EMG
     int analogReading = MyBoeBot.analogSample(15, channels[arm], 0);

     //Action that corresponds to the current state of flex
     int motor_action;
     if (analogReading < emg_limit[0]){ //Stop if lower than lowest limit
       motor_action = d_stop;
     } else if (analogReading < emg_limit[1]){ //Forward if between lower and upper limit
       motor_action = d_forward;
     } else { //Backward if higher than upper limit
       motor_action = d_backward;
     }

     //Chech if the action has changed enough times
     if(MyBoeBot.shouldSwitch(NUMBER_REQUEST_B4_SWITCH, times_requested_switch+arm,
            motor_action, last_motor_action+arm)){
       action_to_send[arm] = motor_action;
     }
   }

  //Byte that will be sent through serial to ESP8266
   int serial_send = MyBoeBot.getSerialSend(action_to_send, 2);

   if(debug){ //If in debug mode
    Serial.println(serial_send);
   } else { //If not in debug mode properly send the byte to serial
    Serial.write(serial_send);
   }
}
