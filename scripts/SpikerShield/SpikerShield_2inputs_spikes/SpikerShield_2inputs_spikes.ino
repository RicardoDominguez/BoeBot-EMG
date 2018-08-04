//Control the BoeBot by flexing your hands repeatedly
// 1. Listen for spikes
// 2. Count the number of spikes close to each other
//  (A spike occurs when you flex strongly during a short period of time)
//  (Spikes close to each other are spikes which occurs within a predetermined
//  number of seconds to each other)
// 3. Interpret the number of spikes counted into a movement action.

// Necessary ibraries
#include <BoeBot.h>
BoeBot MyBoeBot;

//------------------------------------------------------------------------------
// Variables that the user might want to modify
//------------------------------------------------------------------------------
int channels[2] = {A0, A5}; //Analog channel from which the EMG signal is read
                            //channel[0] for left hand
                            //channel[1] for right hand
//Calibration parameters
float bias[2] = {0.5, 0.4};  //Higher bias means more preference towards
                             //lower intensity values
#define calibration_time 1000 //Miliseconds of sampling per discrete state

const int max_time_last_spike = 500; //Max time two spikes will be considered contigous (ms)

const int debug = 0; //If TRUE prints additional messages to screen


#define n_Spikes_stop  1
#define n_Spikes_forw  2
#define n_Spikes_backw 3

//Constants used for serial comunication
const int d_stop = MyBoeBot.d_stop;
const int d_forward = MyBoeBot.d_forward;
const int d_backward = MyBoeBot.d_backward;

//Calibration variables
int thresholds[2][2]; //Values used to detect if user is flexing
float calibration_bias[] = {0.5, 0.7}; //Determines the thresholds
//Higher bias means more preference towards the second value in calibration

//Variables to count peaks
int spike_now = 0;	//Wether the user is flexing
int spike_past[2] = {0, 0};	//Wether the user was flexing in last time step
int nSpikes[2] = {0, 0};		//Number of current contiguous spikes
unsigned long time_last_spike[2]; //When last spike occurred
unsigned long now; //Used to store current time
int action_to_send[2] = {0,0};

//Variables for EMG readings
int analogReading; //measured value for EMG


void setup(){
	Serial.begin(9600);
  calibrate();
}


void loop()
{
	for(int arm=0; arm<2; arm++){
	  //Measure EMG
   analogReading = MyBoeBot.analogSample(25, channels[arm], 0);
   
   //Check if in a spike
   if (analogReading>thresholds[arm][1]){
     spike_now = 1;
   } else if (analogReading<thresholds[arm][0]){
     spike_now = 0;
   }

   //If a spike just ended, update time_last_spike, add 1 to nSpikes counter
   now = millis();
   if ((spike_now == 0)&&(spike_past[arm] == 1)){
     time_last_spike[arm] = now;
     nSpikes[arm] = nSpikes[arm] + 1;
   }
   spike_past[arm] = spike_now;

   //If too long has passed since last spike:
   //Take an action depending of the number of contiguous spikes detected
   //Set nSpikes counter to 0
   if ((now-time_last_spike[arm])>max_time_last_spike){
     switch (nSpikes[arm]){
        case n_Spikes_stop:
          action_to_send[arm] = d_stop;
          break;
        case n_Spikes_forw:
          action_to_send[arm] = d_forward;
          break;
        case n_Spikes_backw:
          action_to_send[arm] = d_backward;
          break;
     }
     nSpikes[arm] = 0;
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

//Gets the maximum value out of all samples read on sample_time
int analogSampleMax(int sample_time, int channel){
  unsigned long sampling_start_time = millis();
  int maximum = 0;
  int reading;
  while(sample_time>(millis()-sampling_start_time)){
    reading = analogRead(channel);
    if (reading>maximum){
      maximum = reading;
    }
    Serial.print("Reading: ");
    Serial.println(reading);
  }
  Serial.print("Max: ");
  Serial.println(maximum);
  return maximum;
}

//Get thresholds in order to distinguish between flexed and relaxed
void calibrate(){
  int readingIntensity[2][2];
  Serial.println("Lets calibrate the device");
  Serial.println("Both arms will be calibrated at the same time");
  delay(2000);
  
	//Get average value for readings with no flex
  Serial.println("First the normal values, relax your arm");
  MyBoeBot.printCountdown(3);
  Serial.println("Recording data...");
  readingIntensity[0][0] = MyBoeBot.analogSample(calibration_time, channels[0], 1);
  readingIntensity[1][0] = MyBoeBot.analogSample(calibration_time, channels[1], 1);

	//Get average value for readings flexing
  Serial.println("Now the flexing state, flex");
  MyBoeBot.printCountdown(3);
  Serial.println("Recording data...");
  readingIntensity[0][1] = analogSampleMax(calibration_time, channels[0]);
  readingIntensity[1][1] = analogSampleMax(calibration_time, channels[1]);
  
	//Calculate the thresholds according to previous readings
  thresholds[0][0] = calibration_bias[0]*(readingIntensity[0][1]-readingIntensity[0][0])+readingIntensity[0][0];
  thresholds[0][1] = calibration_bias[1]*(readingIntensity[0][1]-readingIntensity[0][0])+readingIntensity[0][0];
  thresholds[1][0] = calibration_bias[0]*(readingIntensity[1][1]-readingIntensity[1][0])+readingIntensity[1][0];
  thresholds[1][1] = calibration_bias[1]*(readingIntensity[1][1]-readingIntensity[1][0])+readingIntensity[1][0];
  
  //Print results
  Serial.println("Arm 1: ");
  Serial.print("Threshold 1: ");
  Serial.println(thresholds[0][0]);
  Serial.print("Threshold 2: ");
  Serial.println(thresholds[0][1]);
  Serial.println("Arm 2: ");
  Serial.print("Threshold 1: ");
  Serial.println(thresholds[1][0]);
  Serial.print("Threshold 2: ");
  Serial.println(thresholds[1][1]);
  delay(2000);
  Serial.println("Done");
 }

