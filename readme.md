# Robot control using electromyography (EMG)

This Arduino library allows you to control a [differential drive robot](https://groups.csail.mit.edu/drl/courses/cs54-2001s/diffdrive.html) by flexing your muscles! 

- [Introduction](#introduction)
- [Repository contents](#repository-contents)
- [How EMG signals are translated into commands](#how-emg-signals-are-translated-into-commands)
- [Steps for a working demo](#steps-for-a-working-demo)
- [Using the ESP8266](#using-the-esp8266)


## Introduction

Muscle electrical activity is measured using [electromyography](https://en.wikipedia.org/wiki/Electromyography) or EMG.

The hardware used for the development and testing of the library was:
 * Differential drive robot: [BoeBot](https://www.parallax.com/product/boe-bot-robot).
 * EMG signal measurement device: [Backyard Brains Arduino SpikerShield](https://backyardbrains.com/products/muscleSpikerShield).
 * Wifi communication between measurement device and robot: [EPS8266](https://en.wikipedia.org/wiki/ESP8266).

The library can be easily modified to suit other robots or measurement devices based on the Arduino platform. Feel free to use the library, modify it, give me suggestions or raise issues.

This library was developed by [Ricardo Dominguez Olmedo](https://github.com/RicardoDominguez) during a placement with [Sheffield Robotics](http://www.sheffieldrobotics.ac.uk/) at the Department of Computer Science, The University of Sheffield.

## Repository contents

The [BoeBot-EMG library](/libraries/BoeBot-EMG/) provides the backbone for:
 * Analog sampling of the EMG signals outputted by the SpikerShield.
 * Calibration and filtering to convert the EMG signals sampled into commands for the robot.
 * Communication between the Arduino fitted with the SpikerShields and the Arduino within the BoeBot.
 * Differential drive control of the BoeBot.

Additional, several scripts are included to demonstrate the functionality of the library:
 * [SpikerShield](/scripts/SpikerShield/)
   * [*SpikerShield_2inputs_2actions*](/scripts/SpikerShield/SpikerShield_2inputs_2actions/SpikerShield_2inputs_2actions.ino): reads EMG data from two different analog inputs, interprets it into two discrete actions for motor control (stop and forward, or forward and backward), and sends the information to the ESP8266 via serial.
   * [*SpikerShield_2inputs_3actions*](/scripts/SpikerShield/SpikerShield_2inputs_3actions/SpikerShield_2inputs_3actions.ino): reads EMG data from two different analog inputs, interprets it into three discrete actions for motor control (stop, forward and backward), and sends the information to the ESP8266 via serial.
   * [*SpikerShield_2inputs_spike*](/scripts/SpikerShield/SpikerShield_2inputs_spikes/SpikerShield_2inputs_spikes.ino): reads EMG data from two different analog inputs, counting continuous fast flexes from the user (called spikes). Interprets the number of spikes into three discrete actions for motor control(stop, forward and backward), and sends the information the the ESP8266 via serial.
   * [*SpikerShield_2inputs_variableSpeed*](/scripts/SpikerShield/SpikerShield_2inputs_variableSpeed/SpikerShield_2inputs_variableSpeed.ino): reads EMG data from two different analog inputs, interprets it into several discrete velocities and sends the information to the ESP8266 via serial.
   * [*SpikerShield_claw*](/scripts/SpikerShield/SpikerShield_claw/SpikerShield_claw.ino): reads EMG data from a single analog input, interprets it into two discrete actions for claw control (open, closed), and drives the attached claw accordingly.
   * [*SpikerShield_print_raw_EMG_data*](/scripts/SpikerShield/SpikerShield_print_raw_EMG_data/print_raw_EMG_data.ino): reads the EMG signal from a single SpikerShield and prints it to the Serial for the user to see. This way an user can familiarize itself with how the SpikerShield interprets muscle flexing.
  * [BoeBot/*BoeBot_receive*](/scripts/BoeBot/BoeBot_receive/BoeBot_receive.ino): reads serial data coming from the ESP8266 and interprets this data into motor commands.
 * [ESP8266](/scripts/ESP8266/)
   * [Server script](/scripts/ESP8266/ESP8266_server/init.lua): creates a server with the given ssid and pwd. It then listens indefinitely for incoming data, which sends through the Serial TX port to the Arduino in the BoeBot.
   * [Client script](/scripts/ESP8266/ESP8266_client/init.lua): tries to connect to the server with given ssid and pwd. When the connection is established, it listens indefinitely to the Serial RX port (connected to the Arduino with the SpikerShield), sending to the server all data received.

## How EMG signals are translated into commands

All scripts within SpikerShield/ work in a similar manner:
1. **Calibration**:
   1. **Measure the mean expected EMG activity of certain muscle flex states**. For instance the user is asked to relax his/her arm. Then several thousand EMG readings are sampled, computing the mean EMG activity corresponding to the muscle flex state *arm relaxed*, which can be assigned to a particular robot command such as *stop motor*.
   2. **Compute EMG magnitude thresholds** which are the boundaries between one state and another. For instance, if the mean EMG activity of the state *no flex* is 10 and the mean activity of the state *flex* is 30, then an appropriate EMG activity threshold between the two flex states might be 20. Thus, if we sample an EMG activity value above 20, we will assume that the user is flexing the muscle, whereas if we read a value under 20 we will assume the user is not flexing. However, we may not want the threshold to be the middle value between the mean expected EMG activity of two muscle flex states. The *calibration bias* value determines how the threshold is computed:
      * A calibration bias of 0.5 gives the middle value between the two means, because the threshold will be computed as 0.5\*(30-10) + 10 = 20. 
      * If we choose a bias is less than 0.5, then the threshold will be higher and therefore the user will have to flex more in order to be at the *flex state*. 
      * If the bias is greater than 0.5 then the threshold is lower and therefore the user does not have to flex as much to be at the *flex state*. 
 
   In the software, two thresholds are computed between two flex states: an upper and a lower threshold, each with their own calibration bias, which provides some filtering to the incoming signals. When transitioning between two flex states, the upper threshold is used if the current state is the one with lower mean expected EMG activity (i.e. *no flex*, thus if reading > upper threshold, transition to *flex*), and the lower threshold is used if the current state has higher mean expected EMG activity (i.e. *flex*, when reading < lower threshold, transition to *no flex*).
2. **Determining the flex state** at a particular instant of time. For example, we may have three flex states: *no flex*, *soft flex* and *hard flex*. As seen before, there will be an upper and lower threshold between every contiguous state (i.e, between *no flex* and *soft flex* and between *soft flex* and *hard flex*). If the current state is the one with lowest expected EMG activity, the upper thresholds must be exceded in order to transition to another state. If the current state is the one with highest expected EMG activity, in order to transition to another state the reading must be lower than the lower thresholds. If the current state is *soft flex*, the state will only transition to *no flex* if the current reading is less than the lower threshold between *no flex* and *soft flex*, and similarly the state will only transition to *hard flex* if the current reading is greater than the upper threshold between *soft flex* and *hard flex*.
3. **Determining whether to send a new command to the robot**. To prevent jerky behavior due to the noise in the EMG signals the SpikerShield must sense the same flex state *N* consecutive times before sending to the BoeBot the command corresponding to that flex state.
4. **Send a command to the ESP8266** via serial, to be received by the other ESP8266 connected to the BoeBot. The command for each individual motor is sent on the same byte, which is received and interpreted by the BoeBot.

## Steps for a working demo
1. Load BoeBot_recieve.ino to the BoeBot Arduino with the correct settings and digital and analog pin numbers. Make sure debug is set to 0.
4. Ensure that the switch on the bottom-left corner of the SpikerShield is set to *control*.
3. Connect the RX pin of the ESP8266 loaded with the server software to the BoeBot Arduino TX pin. Connect the TX pin of the ESP8266 loaded with the client software to the SpikerShield Arduino RX pin.
5. Plug the BoeBot battery to the BoeBot Arduino, but make sure that the BoeBot switch is at 0. Power on the ESP8266 in the SpikerShield. You should see a red light on the ESP8266.
1. Connect the required muscle electrodes to the user’s arms.
2. Power the SpikerShield Arduino with a 9V battery, and connect the Arduino to the computer using a cable. Check that the yellow LED at the top of the SpikerShield is powered on. Otherwise check that the SpikerShieldes are properly stacked and power is going through all of them.
3. Load one of the SpikerShield scripts into the SpikerShield. Open the Serial monitor, and click the Arduino's reset button. After calibration, power on the ESP8266 connected to the SpikerShield Arduino. Shortly after, a blue LED in the ESP8266 should start flashing. This indicates that the ESP8266 is communicating with the Arduino through Serial.
4. The ESP8266 on the BoeBot should also have a blue LED flashing, that means that it is communicating with the other ESP8266. Set the power switch on the BoeBot to 2.
5. Everything should work at this point.

## Using the ESP8266

The ESP8266 was flashed with NodeMCU firmware. Then it was programmed with ESPlorer using lua. There is one script for the client (ESP8266 connected to the Arduino with the SpikerShield) and another for the server (ESP8266 connected to the BoeBot). The scripts are named ‘init.lua’ as this is the file that the ESP8266 executes when powered on.
