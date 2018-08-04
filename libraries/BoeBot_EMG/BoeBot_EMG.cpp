#include "BoeBot.h"

// Necesary libraries
#include <Arduino.h>

// Servo pins for both wheels and the claw
#define LEFT_MOTOR_PIN  13
#define RIGHT_MOTOR_PIN 12
#define CLAW_SERVO_PIN  11

// Values written to open/close the claw
#define CLAW_OPEN_POSITION    180
#define CLAW_CLOSED_POSITION  105

// Calibration parameters for controlling the wheels
#define LEFT_MOT_ZERO     1524
#define RIGHT_MOT_ZERO    1520
#define LEFT_MOTOR_SLOPE  15
#define RIGHT_MOTOR_SLOPE 15

//------------------------------------------------------------------------------
// Initilization / setup functions
//------------------------------------------------------------------------------

BoeBot::BoeBot() { }

void BoeBot::boebot_setup()
{
    Serial.begin(9600); // For calibration
}

//------------------------------------------------------------------------------
// Motor control functions
//------------------------------------------------------------------------------

// Sets motor speed, with speeds in the range [-10, 10]
void BoeBot::write2motors(int left_serv, int right_serv)
{
    float left_motor_command = LEFT_MOT_ZERO + left_serv*LEFT_MOTOR_SLOPE;
    float right_motor_command = RIGHT_MOT_ZERO - right_serv*RIGHT_MOTOR_SLOPE;

    servoLeft.attach(LEFT_MOTOR_PIN);
    servoRight.attach(RIGHT_MOTOR_PIN);

    servoLeft.writeMicroseconds(left_motor_command);
    servoRight.writeMicroseconds(right_motor_command);
}

void BoeBot::stop(void)
{
    write2motors(0, 0);
}

//------------------------------------------------------------------------------
// Functions for operating the claw
//------------------------------------------------------------------------------

void BoeBot::openClaw()
{
    Claw.attach(CLAW_SERVO_PIN);
    Claw.write(CLAW_OPEN_POSITION);
}

void BoeBot::closeClaw()
{
    Claw.attach(CLAW_SERVO_PIN);
    Claw.write(CLAW_CLOSED_POSITION);
}

//------------------------------------------------------------------------------
// Calibration functions
//------------------------------------------------------------------------------

/* BoeBot::calibrate_2hands
Use when:
   - 2 EMG inputs (i.e. using both arms)
   - 2 or 3 discrete states (i.e. stop/forwards or stop/forwards/backwards)

Inputs:
   - states: 2 or 3 depending on stop/forwards or stop/forwards/backwards
   - calibration_time: time in ms for each calibration
   - channels: array containing analog pins for samplign the EMG signals

Calibration values returned by refenrence:
   - upper_bias
   - lower_bias
   - upper_thresh
   - lower_thresh
*/
void BoeBot::calibrate_2hands(int states, int calibration_time, int* channels,
      float* upper_bias, float* lower_bias, int* upper_thresh, int* lower_thresh)
{
    Serial.println("Lets calibrate the device");
    Serial.println("Both arms will be calibrate at the same time.");
    delay(2000);

    // Get the sample readings for each flex state
    int calibrationReading[states*3];
    for(int state = 0; state<states; state++)
    {
        if (state == 0)
          Serial.println("Please, relax your arm completely");
        else if (state == 1)
          Serial.println("Now to command forward, flex at medium strength");
        else if (state == 2)
          Serial.println("Now to command backward, flex strongly");
        printCountdown(3);
        Serial.println("Recording data...");

        calibrationReading[state] = analogSample(calibration_time/2, channels[0], 1);
        calibrationReading[states + state] = analogSample(calibration_time/2, channels[1], 1);
      }

    // Turn readings into thresholds
    getEMGthresholds(calibrationReading, 2, states-1, upper_bias, lower_bias, upper_thresh, lower_thresh, 1);
}

/* BoeBot::calibrate_N_states
Use when:
   - 2 EMG inputs
   - N discrete states (i.e. different speed levels)

Inputs:
   - calibration_time: time in ms for each calibration
   - n_speeds: number of discrete states (N)
   - channels: array containing analog pins for samplign the EMG signals

Input calibration parameters:
    - bias

Calibration values returned by refenrence:
   - thresholds
*/
void BoeBot::calibrate_N_states(int calibration_time, int n_speeds, int* channels,
    int* thresholds, float bias)
{
    Serial.println("Lets calibrate the device");
    delay(2000);

    //First index for left/right channel
    //Second index for minimum/maximum
    int readingLimits[2][2];\

    //For each arm...
    for(int arm = 0; arm < 2; arm++)
    {
        if(arm == 0)
          Serial.println("Calibrating left arm");
        else if(arm==1)
          Serial.println("Calibrating right arm");

        Serial.println("Please flex from relaxed state to full flex");
        printCountdown(3);
        Serial.println("Recording data...");

        //Sample maximum and minimum EMG values
        analogSampleLimits(calibration_time, channels[arm], &(readingLimits[arm][1]),
            &(readingLimits[arm][0]), 1);
        readingLimits[arm][1] = bias * readingLimits[arm][1];

        //Compute the bands for the arm
        int delta = ((readingLimits[arm][1] - readingLimits[arm][0]) / n_speeds) + 1;
        Serial.print("Delta: ");
        Serial.println(delta);
        Serial.println("Bands: ");

        for(int band = 0; band<n_speeds; band++)
        {
          thresholds[arm*n_speeds + band] = readingLimits[arm][0] + (band+1) * delta;
          Serial.println(thresholds[arm*n_speeds + band]);
        }
    }
    delay(2000);
}

/* BoeBot::calibrate_claw
Use when:
   - 1 EMG input
   - 2 discrete states (i.e. open/closed)

Inputs:
   - calibration_time: time in ms for each calibration
   - channels: pin for samplign the EMG signals

Input calibration parameters:
    - upper_bias
    - lower_bias

Calibration values returned by refenrence:
   - upper_thresh
   - lower_thresh
*/
void BoeBot::calibrate_claw(int calibration_time, int channel, float upper_bias,
      float lower_bias, int* upper_thresh, int* lower_thresh)
{
    Serial.println("Lets calibrate the claw");
    delay(2000);

    int calibrationReading[2];
    for(int state = 0; state < 2; state++)
    {
      if (state == 0)
        Serial.println("First, to calibrate the open claw position, relax your arm");
      else
        Serial.println("Now, to calibrate the closed claw position, flex");
      printCountdown(3);
      Serial.println("Recording data...");

      calibrationReading[state] = analogSample(calibration_time, channel, 1);
    }

    (*upper_thresh) = upper_bias*(calibrationReading[1]-calibrationReading[0]) +calibrationReading[0];
    (*lower_thresh) = lower_bias*((*upper_thresh)-calibrationReading[0]) +calibrationReading[0];

    Serial.print("Lower threshold: ");
    Serial.println(*lower_thresh);
    Serial.print("Upper threshold: ");
    Serial.println(*upper_thresh);
    delay(2000);
}

/* BoeBot::getEMGthresholds
Computes the thresholds that will be used to distinguish between the discrete
events while reading the EMG signals

Inputs:
   - calibrationReading: EMG samples, has size [arms][states]
   - verb: if 1 prints aditional information

Input calibration parameters:
   - upper_bias
   - lower_bias

Calibration values returned by refenrence:
   - upper_thresh
   - lower_thresh
*/
void BoeBot::getEMGthresholds(int* calibrationReading, int arms, int states, float* upper_bias,
      float* lower_bias, int* upper_thresh, int* lower_thresh, int verb)
{
  for(int arm=0; arm<arms; arm++)
  {
     for(int s = 0; s<states; s++)
     {
       upper_thresh[arm*states + s] = upper_bias[s]*(calibrationReading[arm*(states+1) + s + 1]
            -calibrationReading[arm*(states+1) + s])+calibrationReading[arm*(states+1) + s];
       lower_thresh[arm*states + s] = lower_bias[s]*(upper_thresh[arm*states + s]
           -calibrationReading[arm*(states+1) + s])+ calibrationReading[arm*(states+1) + s];
     }

     //Echo
     if(verb==1)
     {
       Serial.print("Arm: ");
       Serial.println(arm);
       for(int s = 0; s<states; s++)
       {
         Serial.print("Upper threshold: ");
         Serial.println(upper_thresh[arm*states + s]);
         Serial.print("Lower threshold: ");
         Serial.println(lower_thresh[arm*states + s]);
        }
        delay(2000);
     }
  }
}

//------------------------------------------------------------------------------
// Functions for analog sampling
//------------------------------------------------------------------------------

// Continously sample analog input 'channel' for 'sample_time' miliseconds, and
// return the average value of the samples. If 'verb' is 1, additional sampling
// information is printed.
int BoeBot::analogSample(int sample_time, int* channel, int verb)
{
  unsigned long sampling_start_time = millis();
  unsigned long sum_values = 0;
  int n_samples = 0; //Number of samples taken

  int reading;
  while(sample_time > (millis() - sampling_start_time)) // For 'sample_time' ms
  {
    reading = analogRead(channel);
    sum_values = sum_values + reading;
    n_samples++;

    if(verb == 1)
    {
      Serial.print("Reading: ");
      Serial.println(reading);
    }
  }

  if(verb == 1)
  {
    Serial.print("Average: ");
    Serial.println(sum_values/n_samples);
  }

  return sum_values/n_samples; // Compute the average of all the samples
}

// Continously sample analog input 'channel' for 'sample_time' miliseconds, and
// return by reference the maximum and minimum readings observed. If 'verb' is 1,
// additional sampling information is printed.
void BoeBot::analogSampleLimits(int sample_time, int* channel, int* max_sample,
        int* min_sample, int verb)
{
  unsigned long sampling_start_time = millis();
  int minVal = 10000; //Initialized greater than the minimum values can be
  int maxVal = -1; // Initialized lower than the maximum value can be

  int reading;
  while(sample_time>(millis()-sampling_start_time)) // For 'sample_time' ms
  {
    reading = analogRead(channel);

    if(reading > maxVal)
    {
      maxVal = reading;
    }

    if(reading < minVal)
    {
      minVal = reading;
    }

    if(verb == 1)
    {
      Serial.print("Reading: ");
      Serial.println(reading);
    }
  }

  //Return by reference
  *min_sample = minVal;
  *max_sample = maxVal;

  if(verb == 1)
  {
    Serial.print("Minimum value: ");
    Serial.println(minVal);
    Serial.print("Maximum value: ");
    Serial.println(maxVal);
  }
}

//------------------------------------------------------------------------------
// Functions for serial communication
//------------------------------------------------------------------------------

//Given the array "actions" containing the signals to send (d_stop, d_forward...)
//combine them into a single int that will be sent to the ESP8266 via serial
int BoeBot::getSerialSend(int* actions, int n_actions)
{
  int int_to_send = actions[0] + actions[1]* 10; // At least 2 actions

  if(n_actions == 3) // If a 3rd action is included...
  {
    int_to_send = int_to_send + actions[2]*100;
  }

  return int_to_send;
}

/*
Read each of the three digits of the byte recieved through serial by the BoeBot
and activate servos accordingly.

Inputs:
  -byte_read: bite recieved
  -motor_speed: constant motor speed if not using multiple speeds
  -multSpeeds:
      if 1 used for two inputs, multiple speeds, only stop/forward
      if 0 used for constant speed, stop/forward/backward
  -verb: if 1 prints additional information
*/
void BoeBot::interpretEMGserial(int byte_read, int motor_speed, int multSpeeds, int verb)
{
  if(verb == 1)
  {
    Serial.print("Byte recieved: ");
    Serial.println(byte_read);
  }

  // Read each of the three digits of the byte recieved
  // i.e.byte 254:
  //  digits_recieved[2] = 2,
  //  digits_recieved[1] = 5,
  //  digits_recieved[0] = 4
  int digits_recieved[3];
  digits_recieved[2] = byte_read / 100;
  byte_read = byte_read - digits_recieved[2]*100;
  digits_recieved[1] = byte_read / 10;
  byte_read = byte_read - digits_recieved[1]*10;
  digits_recieved[0] = byte_read;

  if(verb==1)
  {
    Serial.print("Motor left: ");
    Serial.println(digits_recieved[0]);
    Serial.print("Motor right: ");
    Serial.println(digits_recieved[1]);
    Serial.print("Claw: ");
    Serial.println(digits_recieved[2]);
  }

  if(multSpeeds == 1)
  {
    //Make sure input speed is within speed bounds
    for(int i = 0; i<2; i++)
    {
      if(digits_recieved[i] > 10)
        digits_recieved[i] = 10;
      else if(digits_recieved[i]<0)
        digits_recieved[i] = 0;
    }

    //Write to motor
    write2motors(digits_recieved[0], digits_recieved[1]);

  } else {
    //Interpret the digits for the left and right motor
    int motor_action[2];
    for(int i=0; i<2; i++)
    {
      if(digits_recieved[i] == d_forward)
        motor_action[i] = 1;
      else if(digits_recieved[i] == d_backward)
        motor_action[i] = -1;
      else
        motor_action[i] = 0;
    }

    //Write to the motors
    write2motors(motor_action[0]*motor_speed, motor_action[1]*motor_speed);
  }

  //Interpret claw digit
  if(digits_recieved[2] == d_open)
    openClaw();
  else if (digits_recieved[2] == d_closed)
    closeClaw();
}

/*
Returns 1 if a new command should be sent, 0 otherwise

New commands will be sent if the number of function calls requesting to send a
new command is greater than 'n_times_before_switch'
*/
int BoeBot::shouldSwitch(int n_times_before_switch,
    int* times_requested_switch, int motor_action, int* last_motor_action)
{
   if((*last_motor_action) != motor_action) // New command requested
   {
     (*times_requested_switch) = 0; // Reset counter of function calls
     (*last_motor_action) = motor_action;
   }
   else
   {
     if ((*times_requested_switch) > n_times_before_switch) // Enough function calls
     {
       return 1;
     }
     else  // Add 1 to counter of function calls
     {
       (*times_requested_switch) = (*times_requested_switch) + 1;
     }
   }

   return 0; //Conditions not meet to switch
 }

//------------------------------------------------------------------------------
// Misc. functions
//------------------------------------------------------------------------------

//Print a countdown to serial
void BoeBot::printCountdown(int secs)
{
   for(int i=0; i < secs; i++)
   {
     Serial.print(secs-i);
     Serial.println("...");
     delay(1000);
   }
 }
