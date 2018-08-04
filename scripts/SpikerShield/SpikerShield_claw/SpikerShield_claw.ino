//Read EMG from one input, operate the claw

//1. Read muscle flext through EMG for one inputs (one arms)
//2. Divide the signal into two states, associated with OPEN or CLOSED
//3. Drive the claw according to the obtained state

//Servo instance
#include <Servo.h>
#include <BoeBot.h>
BoeBot MyBoeBot;

//------------------------------------------------------------------------------
// Variables that the user might want to modify
//------------------------------------------------------------------------------
int channel = A0; //Analog channel from which the EMG signal is read
#define NUMBER_REQUEST_B4_SWITCH 2 //The claw has to recieve NUMBER_REQUEST_B4_SWITCH
                                   // signals indicating to change its action before it
                                   //actually sends a different action. //This gives
                                   // consistency to the output against noisy signals
//Calibration parameters
float upper_bias = 0.45;  //Higher bias means more preference towards
float lower_bias = 0.15; //lower intensity values
#define calibration_time 1000 //Miliseconds of sampling per discrete state

const int debug = 1; //If TRUE prints additional messages to screen

#define SERVO_PIN 2   //pin for servo motor


//------------------------------------------------------------------------------
// Variables used throughout the code
//------------------------------------------------------------------------------
//Constants used for serial comunication
const int d_open = MyBoeBot.d_open;
const int d_closed = MyBoeBot.d_closed;


//To store the thresholds from calibration
int upper_thresh, lower_thresh;

//Action that will be sent to the ESP8266 through serial
int action_to_send = d_open;
//Previous action sent
int last_motor_action = MyBoeBot.d_open;
//Number of times requested to change the action
int times_requested_switch = 0;

void setup(){
  Serial.begin(9600);
  
  MyBoeBot.calibrate_claw(calibration_time, channel, upper_bias,
        lower_bias, &upper_thresh, &lower_thresh);

  if(debug==1){
    Serial.print("Upper threshold: ");
    Serial.println(upper_thresh);
    Serial.print("Lower threshold: ");
    Serial.println(lower_thresh);
    delay(2000);
  }
  MyBoeBot.openClaw();
}

void loop()
{
      int emg_limit;

      //Open/close claw if appropiate
      //Logic to decide if claw should be open or closed
     if (action_to_send == d_closed){
        emg_limit = lower_thresh;
     } else {
        emg_limit = upper_thresh;
     }

     //Measure EMG
     int analogReading = MyBoeBot.analogSample(50, channel, 0);
     if(debug == 1){
      Serial.println(analogReading);
     }

     //Action that corresponds to the current state of flex
     int motor_action;
     if (analogReading>=emg_limit){
        motor_action = d_closed;
     } else {
        motor_action = d_open;
     }

      //If appropiate to change state, do so
      if(MyBoeBot.shouldSwitch(NUMBER_REQUEST_B4_SWITCH, &times_requested_switch,
            motor_action, &last_motor_action)){
        if(motor_action == d_closed){
          action_to_send = d_closed;
          Serial.println("Close");
          MyBoeBot.closeClaw();
        } else if (action_to_send == d_closed){
          action_to_send == d_open;
          Serial.println("Open");
          MyBoeBot.openClaw();
        }
      }
}
