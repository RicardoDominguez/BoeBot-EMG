//Read EMG, two input, two states

//1. Read muscle flext through EMG for two inputs (two arms)
//2. Divide the signal into two states, associated with STOP and FORWARD
//   or FORWARD and BACKWARD
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
float upper_bias[1] = {0.5};  //Higher bias means more preference towards
float lower_bias[1] = {0.4}; //lower intensity values
#define calibration_time 1000 //Miliseconds of sampling per discrete state

const int go_backward = 1;  //If 0, actions are STOP/FORWARD, if 1, actions are FORWARD/BACKWARD

const int debug = 0; //If TRUE prints additional messages to screen


//------------------------------------------------------------------------------
// Variables used throughout the code
//------------------------------------------------------------------------------
//Constants used for serial comunication
const int d_stop = MyBoeBot.d_stop;
const int d_forward = MyBoeBot.d_forward;
const int d_backward = MyBoeBot.d_backward;

int action_noFlex;
int action_flex;

//To store the thresholds from calibration
int upper_thresh[2], lower_thresh[2];

//Action that will be sent to the ESP8266 through serial
int action_to_send[2];
//Previous action sent
int last_motor_action[2];
//Number of times requested to change the action
int times_requested_switch[2] = {0, 0};


void setup(){
  Serial.begin(9600);

  //Calibrate device
  MyBoeBot.calibrate_2hands(2, calibration_time, channels, upper_bias,
        lower_bias, upper_thresh, lower_thresh);

  //Choose wether STOP/FORWARD or FORWARD/BACKWARD
  if(go_backward == 0){
    action_noFlex = d_stop;
    action_flex = d_forward;
  } else {
    action_noFlex = d_forward;
    action_flex = d_backward;
  }

  last_motor_action[0] = action_noFlex;
  last_motor_action[1] = action_flex;
  action_to_send[0] = action_noFlex;
  action_to_send[1] = action_flex;
}

void loop()
{
  //Thresholds used to decide the state of flex
  int emg_limit;
  for(int arm = 0; arm<2; arm++){
    if(action_to_send[arm] == action_noFlex){
       emg_limit = upper_thresh[arm];
    } else if (action_to_send[arm] == action_flex){
       emg_limit = lower_thresh[arm];
    }

    //Measure EMG
    int analogReading = MyBoeBot.analogSample(15, channels[arm], 0);

    //Action that corresponds to the current state of flex
    int motor_action;
    if (analogReading > emg_limit){ //Action_flex if abouve upper limit
      motor_action = action_flex;
    } else { //Action_noFlex if below limit
      motor_action = action_noFlex;
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
