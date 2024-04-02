AHVER9.ino  is Code for an artificial horizon display, Welch 4/2/24
This sketch uses  Bodmer’s  TFT display code and New UW Mahony AHRS for the LSM9DS1 - S.J. Remington 3/2021. I ran both  of these programs separately at first to get 1st the display working with Bodmers AHD test code – no sensor attached, then the UW Mahony to talk to the sensor and make sure that was working ok. I then merged the two sketches and tweaked the display for proper operation. 
Its setup for a 320 by 240 TFT display 
The library for this is in TFT_IL19341 ( this is an Adafruit GFX modified by Bodmer)
Uses  Arduino mega board
User_Setup.h has the Arduino mega pin definitions. SCK is 52, MOSI is 51 , RST is 44.  Modify this file for your Arduino interface pins. 
This sketch will not work right until the LSM9DS1 goes through a calibration. Every LSM9DS1 is different and needs calibration. (Yes this is a pain).  
Requires (Include) the Sparkfun LSM9DS1 library
Standard sensor orientation X North (yaw=0), Y West, Z up
NOTE: Sensor X axis is remapped to the opposite direction of the "X arrow" on the Adafruit sensor breakout!
Note: Roll orientation was reversed so Ver 9 (roll *-1) fixes it. (line 271) 

New Mahony filter error scheme uses Up (accel Z axis) and West (= Acc X Mag) as the orientation reference vectors
heavily modified from http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
Both the accelerometer and magnetometer MUST be properly calibrated for this program to work.
Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
To collect data for calibration, use the companion program LSM9DS1_cal_data, to generate the raw calibration data. ( for this to work the Arduino mega must be working and the LSM9DS1 must be connected to the Mega)  
Break the Accelerometer and gyro data into one CSV file and the Magnetometer data into another 2nd CSV separate file.
Run the two raw files through the Magneto .exe program (Running under Code blocks) to generate the calibration data.(I chose a deviation of 2 for bad data ignore). It took a while to understand how Magneto worked and to get it running correctly. 
Copy and paste the calibration data obtained from Magneto into the  AHVER8.ino  program.
Youtube video link  https://youtu.be/Dzm7vEwAZCk 
I’ve found this sketch runs fairly slow for large pitch and roll changes. Planning on moving this over to a Teensy 4.0 board as it should run much faster. Also moving to a TFT parallel data input would help speed it up. 


