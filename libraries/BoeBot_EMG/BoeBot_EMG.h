/*
This library includes the functionality required to control a BoeBot (or similar)
using input EEG analog signals, including serial/wifi communication between the
Arduino used for sampling the EEG signals and the BoeBot.
*/

#ifndef BoeBot_h
#define BoeBot_h

// Necesary libraries
#include <Servo.h>

class BoeBot
{
public:
    // Initilization / setup functions
    BoeBot();
    void boebot_setup();

    // Motor control functions
    void write2motors(int left_serv, int right_serv);
    void stop();

    // Functions for operating the claw
    void openClaw();
    void closeClaw();

    // Calibration functions
    void calibrate_2hands(int states, int calibration_time, int *channels,
          float *upper_bias, float *lower_bias, int *upper_thresh, int *lower_thresh);
    void calibrate_N_states(int calibration_time, int n_speeds, int *channels,
          int *thresholds, float bias);
    void calibrate_claw(int calibration_time, int channel, float upper_bias,
          float lower_bias, int *upper_thresh, int *lower_thresh);
    void getEMGthresholds(int *calibrationReading, int arms, int states, float *upper_bias,
          float *lower_bias, int *upper_thresh, int *lower_thresh, int verb);

    // Functions for analog sampling
    int  analogSample(int sample_time, int *channel, int verb);
    void analogSampleLimits(int sample_time, int *channel, int *max_sample, int *min_sample, int verb);

    // Functions for serial communication
    int getSerialSend(int *actions, int n_actions);
    void interpretEMGserial(int byte_read, int motor_speed, int multSpeeds, int verb);
    int shouldSwitch(int n_times_before_switch, int *times_requested_switch,
          int motor_action, int *last_motor_action);

    // Misc. functions
    void printCountdown(int secs);

    //Definitions of the digit associated with each action transmitted for the
    //comunication between the EMG Spikerbox and the BoeBot
    const int d_stop     = 0;
    const int d_forward  = 1;
    const int d_backward = 2;
    const int d_no_claw  = 0;
    const int d_open     = 1;
    const int d_closed   = 2;

private:
    Servo servoLeft;
    Servo servoRight;
    Servo Claw;
};

#endif
