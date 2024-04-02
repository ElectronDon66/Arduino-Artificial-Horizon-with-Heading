// AHVER9 is Code for artifical horizon display, Welch 4/2/24
// Uses  Bodmer  TFT display code and New UW Mahony AHRS for the LSM9DS1  S.J. Remington 3/2021
// Its setup for a 320 by 240 TFT display 
// The library for this is in TFT_IL19341
// Uses  Arduino mega
// User_Setup.h has the arduino mega pin definitions. SCK is 52,MOSI is 51 ,RST is 44
// This sketch will not work right until the LSM9DS1 goes through a calibration 
// Requires the Sparkfun LSM9DS1 library
// Standard sensor orientation X North (yaw=0), Y West, Z up
// NOTE: Sensor X axis is remapped to the opposite direction of the "X arrow" on the Adafruit sensor breakout!
// Note: (line 271) roll orientation was reversed for this sketch and 9DOF board Ver 8 was 180 degrees out 

// New Mahony filter error scheme uses Up (accel Z axis) and West (= Acc X Mag) as the orientation reference vectors
// heavily modified from http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// Both the accelerometer and magnetometer MUST be properly calibrated for this program to work.
// Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
//
// To collect data for calibration, use the companion program LSM9DS1_cal_data, to generate the raw calibration data 
// Break the Accelerometer and gyro data into one CSV file and the Magnetometer data into another CSV seperate file.
// Run the two raw files through the Magneto .exe program (Running under Code blocks) to generate the calibration data.
// Copy the calibration data obtained from Magneto into this program 


/*
  Adafruit 3V or 5V board
  Hardware setup: This library supports communicating with the
  LSM9DS1 over either I2C or SPI. This example demonstrates how
  to use I2C. The pin-out is as follows:
  LSM9DS1 --------- Arduino
   SCL ---------- SCL() pin 21)
   SDA ---------- SDA (Pin 20 )
   VIN ------------- 5V
   GND ------------- GND

   CSG, CSXM, SDOG, and SDOXM should all be pulled high.
   pullups on the ADAFRUIT breakout board do this.
*/
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.




#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>


//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// default settings gyro  245 d/s, accel = 2g, mag = 4G
LSM9DS1 imu;

// VERY IMPORTANT!
//These are the previously determined offsets and scale factors for accelerometer and magnetometer, using MPU9250_cal and Magneto
//The filter will produce meaningless results if these data are not correct

//Gyro scale 245 dps convert to radians/sec and offsets
float Gscale = (M_PI / 180.0) * 0.00875; //245 dps scale sensitivity = 8.75 mdps/LSB
int G_offset[3] = {-31, 26, 103}; // Put the gyro offset from LSM9DS1 cal sketch here ( The program original values were 75,31,142)

//Accel scale 16836.0 to normalize
float A_B[3]
{ -373.45,   -80.83, -634.65}; // Put the bias vector from magneto Accel file here and the correction table below 

float A_Ainv[3][3]  //Below 9 element matrix is the Accelerometer correction matrix obtained from Magneto
{ {  1.01720,  0.00638,  -0.00249},
  {  0.00638,  1.02092,  0.00093},
  {  -0.00249,  0.00093,  1.01409}
};

//Mag scale 2995.0 to normalize
float M_B[3]
{ 881.87, 1719.45,  2537.95}; //Put the bias vector from magneto Magnetometer here and the correction table below 

float M_Ainv[3][3]  // Below is the 9 element magnetometer matrix obtained from Magneto 
{ {  0.79484,  0.05784, 0.00029},
  {  0.05784,  0.85503,  -0.03127},
  { 0.00029,  -0.03127,  0.79224}
};

// local magnetic declination in degrees , this is you local magnetic declination. It can also be used to rotate the compass orientation to whatever you need 
float declination = -14.84;

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// Kp is not yet optimized. Ki is not used.
#define Kp 50.0
#define Ki 0.0

unsigned long now = 0, last = 0; //micros() timers for AHRS loop
float deltat = 0;  //loop time in seconds

#define PRINT_SPEED 300 // ms between angle prints
unsigned long lastPrint = 0; // Keep track of print time

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; //Euler angle output
// This is the end of the statements for the 9DOF*********************************************************************

// Use ONE of these three highly optimised libraries, comment out other two!




// For ILI9341 based TFT displays (note sketch is currently setup for a 320 x 240 display)
#include <TFT_ILI9341.h>         // Bodmer's graphics and font library for ILI9341 driver chip
TFT_ILI9341 tft = TFT_ILI9341(); // Invoke library, pins defined in User_Setup.h
//                                    https://github.com/Bodmer/TFT_ILI9341

#define REDRAW_DELAY 16 // minimum delay in milliseconds between display updates

#define HOR 316  // Horizon vector line length

#define BROWN      0x5140 //0x5960
#define SKY_BLUE   0x02B5 //0x0318 //0x039B //0x34BF
#define DARK_RED   0x8000
#define DARK_GREY  0x39C7
#define BLACK      0x0000

#define XC 120 // x coord of centre of horizon
#define YC 160 // y coord of centre of horizon

#define DEG2RAD 0.0174532925

int last_roll = 0; // the whole horizon graphic
int last_pitch = 0;
int pitchn =0 ;

// Variables for test only
int test_roll = 0;
int delta = 0;

unsigned long redrawTime = 0;

// #########################################################################
// Setup, runs once on boot up
// #########################################################################

void setup(void) {
  Serial.begin(115200);
  delay(10);
//Startup the 9DOF 
  
  while (!Serial); //wait for connection
  Serial.println();
  Serial.println("LSM9DS1 AHRS starting");

  Wire.begin();
  delay(10);

  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println(F("LSM9DS1 not detected"));
    while (1);
  }

  //End of setup for 9DOF
  
  
 //Below is setup for the horizon  
  tft.begin();
  tft.setRotation(0);

  tft.fillRect(0,  0, 240, 160, SKY_BLUE);
  tft.fillRect(0, 160, 240, 160, BROWN);

  // Draw the horizon graphic
  drawHorizon(0, 0);
  drawInfo();
  delay(2000); // Wait to permit visual check

  // Test roll and pitch
 // testRoll(); //get rid of test for now 
  //testPitch();// get rid of test for now

  tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
 
}


// #########################################################################
// Main loop, keeps looping around
// #########################################################################

void loop() {

  /*// Refresh the display at regular intervals
  if (millis() > redrawTime) {
    redrawTime = millis() + REDRAW_DELAY;

    
   Need to replace this chunk with get Roll,pitch and yaw from sensor 
   
   // Roll is in degrees in range +/-180
    int roll = random(361) - 180;

    // Pitch is in y coord (pixel) steps, 20 steps = 10 degrees on drawn scale
    // Maximum pitch shouls be in range +/- 80 with HOR = 316
    int pitch = random (161) - 80;

    
  }
  */

static char updated = 0; //flags for sensor updates
  static int loop_counter=0; //sample & update loop counter
  static float Gxyz[3], Axyz[3], Mxyz[3]; //centered and scaled gyro/accel/mag data

  // Update the sensor values whenever new data is available
  if ( imu.accelAvailable() ) {
    updated |= 1;  //acc updated
    imu.readAccel();
  }
  if ( imu.magAvailable() ) {
    updated |= 2; //mag updated
    imu.readMag();
    
  }
  if ( imu.gyroAvailable() ) {
    updated |= 4; //gyro updated
    imu.readGyro();
  }
  if (updated == 7) //all sensors updated?
  {
    updated = 0; //reset update flags
    loop_counter++;
    get_scaled_IMU(Gxyz, Axyz, Mxyz);

    // correct accel/gyro handedness
    // Note: the illustration in the LSM9DS1 data sheet implies that the magnetometer
    // X and Y axes are rotated with respect to the accel/gyro X and Y, but this is not case.

    Axyz[0] = -Axyz[0]; //fix accel/gyro handedness
    Gxyz[0] = -Gxyz[0]; //must be done after offsets & scales applied to raw data
   
    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;

    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                           Mxyz[0], Mxyz[1], Mxyz[2], deltat);

    if (millis() - lastPrint > PRINT_SPEED) {

      // Define Tait-Bryan angles, strictly valid only for approximately level movement
      // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
      // this code corrects for magnetic declination.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order.
      //
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.
      
      // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
      // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock
      roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
      pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
      // to degrees
      yaw   *= 180.0 / PI;
      pitch *= 180.0 / PI;
      roll = -1* roll * 180.0 / PI;
      

      // http://www.ngdc.noaa.gov/geomag-web/#declination
      //conventional nav, yaw increases CW from North, corrected for local magnetic declination

      yaw = -(yaw + declination);
      if (yaw < 0) yaw += 360.0;
      if (yaw >= 360.0) yaw -= 360.0;

      /* // Use for trouble shooting 
      Serial.print("ypr ");
      Serial.print(yaw, 0);
      Serial.print(", ");
      Serial.print(pitch, 0);
      Serial.print(", ");
      Serial.print(roll, 0);
*/

      updateHorizon(roll, pitch); //Moved this here because now we have the sensor roll and pitch
       
//      Serial.print(", ");  //prints 24 in 300 ms (80 Hz) with 16 MHz ATmega328
//      Serial.print(loop_counter);  //sample & update loops per print interval
      loop_counter = 0;
     Serial.println();
      lastPrint = millis(); // Update lastPrint time
    }
  }

}

// #########################################################################
// Update the horizon with a new roll (angle in range -180 to +180)
// #########################################################################

void updateHorizon(int roll, int pitch)
{
  //bool draw = 1;
  int delta_pitch = 0;
  int pitch_error = 0;
  int delta_roll  = 0;
  while ((last_pitch != pitch) || (last_roll != roll))
  {
    delta_pitch = 0;
    delta_roll  = 0;

    if (last_pitch < pitch) {
      delta_pitch = 1;
      pitch_error = pitch - last_pitch;
    }

    if (last_pitch > pitch) {
      delta_pitch = -1;
      pitch_error = last_pitch - pitch;
    }

    if (last_roll < roll) delta_roll  = 1;
    if (last_roll > roll) delta_roll  = -1;

    if (delta_roll == 0) {
      if (pitch_error > 1) delta_pitch *= 2;
    }

    drawHorizon(last_roll + delta_roll, last_pitch + delta_pitch);
    drawInfo();
  }
}

// #########################################################################
// Draw the horizon with a new roll (angle in range -180 to +180)
// #########################################################################

void drawHorizon(int roll, int pitch)
{
  // Calculate coordinates for line start
  float sx = cos(roll * DEG2RAD);
  float sy = sin(roll * DEG2RAD);

  int16_t x0 = sx * HOR;
  int16_t y0 = sy * HOR;
  int16_t xd = 0;
  int16_t yd = 1;
  int16_t xdn  = 0;
  int16_t ydn = 0;

  if (roll > 45 && roll <  135) {
    xd = -1;
    yd =  0;
  }
  if (roll >=  135)             {
    xd =  0;
    yd = -1;
  }
  if (roll < -45 && roll > -135) {
    xd =  1;
    yd =  0;
  }
  if (roll <= -135)             {
    xd =  0;
    yd = -1;
  }
pitchn = pitch*2; //This line changes the # of pixels = pitch of 1 degree 
  if ((roll != last_roll) || (pitch != last_pitch))
  {
/*xdn = 8 * xd;  //may need this 
    ydn = 8 * yd;//c
     //pitchn = pitch*2; //This line changes the # of pixels = pitch of 1 degree 
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitchn, XC + x0 - xdn, YC + y0 - ydn - pitchn, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitchn, XC + x0 + xdn, YC + y0 + ydn - pitchn, BROWN);
*/


    xdn = 7 * xd;
    ydn = 7 * yd;//c
    // pitchn = pitch*2; //This line changes the # of pixels = pitch of 1 degree 
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitchn, XC + x0 - xdn, YC + y0 - ydn - pitchn, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitchn, XC + x0 + xdn, YC + y0 + ydn - pitchn, BROWN);
    xdn = 6 * xd;
    ydn = 6 * yd;//c
     //pitchn = pitch*2; //This line changes the # of pixels = pitch of 1 degree 
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitchn, XC + x0 - xdn, YC + y0 - ydn - pitchn, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitchn, XC + x0 + xdn, YC + y0 + ydn - pitchn, BROWN);
    xdn = 5 * xd;
    ydn = 5 * yd;
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitchn, XC + x0 - xdn, YC + y0 - ydn - pitchn, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitchn, XC + x0 + xdn, YC + y0 + ydn - pitchn, BROWN);
    xdn = 4 * xd;
    ydn = 4 * yd;
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitchn, XC + x0 - xdn, YC + y0 - ydn - pitchn, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitchn, XC + x0 + xdn, YC + y0 + ydn - pitchn, BROWN);
    
    xdn = 3 * xd;
    ydn = 3 * yd;
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitchn, XC + x0 - xdn, YC + y0 - ydn - pitchn, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitchn, XC + x0 + xdn, YC + y0 + ydn - pitchn, BROWN);
  }
  xdn = 2 * xd;
  ydn = 2 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitchn, XC + x0 - xdn, YC + y0 - ydn - pitchn, SKY_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitchn, XC + x0 + xdn, YC + y0 + ydn - pitchn, BROWN);

  tft.drawLine(XC - x0 - xd, YC - y0 - yd - pitchn, XC + x0 - xd, YC + y0 - yd - pitchn, SKY_BLUE);
  tft.drawLine(XC - x0 + xd, YC - y0 + yd - pitchn, XC + x0 + xd, YC + y0 + yd - pitchn, BROWN);

  tft.drawLine(XC - x0, YC - y0 - pitchn,   XC + x0, YC + y0 - pitchn,   TFT_WHITE);

  last_roll = roll;
  last_pitch = pitch;

}

// #########################################################################
// Draw the information
// #########################################################################

void drawInfo(void)
{
  // Update things near middle of screen first (most likely to get obscured)

  // Level wings graphic
  tft.fillRect(120 - 1, 160 - 1, 3, 3, TFT_RED);
  tft.drawFastHLine(120 - 30,   160, 24, TFT_RED);
  tft.drawFastHLine(120 + 30 - 24, 160, 24, TFT_RED);
  tft.drawFastVLine(120 - 30 + 24, 160, 3, TFT_RED);
  tft.drawFastVLine(120 + 30 - 24, 160, 3, TFT_RED);

  // Pitch scale
  tft.drawFastHLine(120 - 12,   160 - 40, 24, TFT_WHITE);
  tft.drawFastHLine(120 -  6,   160 - 30, 12, TFT_WHITE);
  tft.drawFastHLine(120 - 12,   160 - 20, 24, TFT_WHITE);
  tft.drawFastHLine(120 -  6,   160 - 10, 12, TFT_WHITE);

  tft.drawFastHLine(120 -  6,   160 + 10, 12, TFT_WHITE);
  tft.drawFastHLine(120 - 12,   160 + 20, 24, TFT_WHITE);
  tft.drawFastHLine(120 -  6,   160 + 30, 12, TFT_WHITE);
  tft.drawFastHLine(120 - 12,   160 + 40, 24, TFT_WHITE);

  // Pitch scale values
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(120 - 12 - 13, 160 - 20 - 3);
  tft.print("10");
  tft.setCursor(120 + 12 + 1, 160 - 20 - 3);
  tft.print("10");
  tft.setCursor(120 - 12 - 13, 160 + 20 - 3);
  tft.print("10");
  tft.setCursor(120 + 12 + 1, 160 + 20 - 3);
  tft.print("10");

  tft.setCursor(120 - 12 - 13, 160 - 40 - 3);
  tft.print("20");
  tft.setCursor(120 + 12 + 1, 160 - 40 - 3);
  tft.print("20");
  tft.setCursor(120 - 12 - 13, 160 + 40 - 3);
  tft.print("20");
  tft.setCursor(120 + 12 + 1, 160 + 40 - 3);
  tft.print("20");

  // Display justified roll value near bottom of screen
  tft.setTextColor(TFT_YELLOW, BROWN); // Text with background
  tft.setTextDatum(MC_DATUM);            // Centre middle justified
  tft.setTextPadding(24);                // Padding width to wipe previous number
  tft.drawString("Roll  ",105,270,2);
  tft.drawNumber(last_roll, 128, 270, 2);
  tft.setTextPadding(24);                // Padding width to wipe previous number
  tft.drawString("Pitch  ",105,290,2);
  tft.drawNumber(last_pitch, 128, 290, 2);
// Display the Heading /Yaw 
  tft.setTextColor(TFT_RED, BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
    tft.setTextPadding(50);
  
   
   tft.drawNumber(yaw,120,20,2); 
  tft.setTextSize(1);
  
  tft.setTextColor(TFT_YELLOW);
  tft.setTextDatum(TC_DATUM); 
  tft.setTextPadding(24); 
  tft.drawString("Heading", 120, 50, 2);
  // Draw fixed text
  tft.setTextColor(TFT_YELLOW);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  tft.drawString("Artifical Horizon", 120, 1, 2);
 
}

// #########################################################################
// Function to generate roll angles for testing only
// #########################################################################

int rollGenerator(int maxroll)
{
  // Synthesize a smooth +/- 50 degree roll value for testing
  delta++; if (delta >= 360) test_roll = 0;
  test_roll = (maxroll + 1) * sin((delta) * DEG2RAD);

  // Clip value so we hold roll near peak
  if (test_roll >  maxroll) test_roll =  maxroll;
  if (test_roll < -maxroll) test_roll = -maxroll;

  return test_roll;
}

// #########################################################################
// Function to generate roll angles for testing only
// #########################################################################

void testRoll(void)
{
  tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  tft.drawString("Roll test", 128, 20, 2);

  for (int a = 0; a < 360; a++) {
    //delay(REDRAW_DELAY / 2);
    updateHorizon(rollGenerator(180), 0);
  }
  tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  tft.drawString("         ", 128, 10, 1);
}

// #########################################################################
// Function to generate pitch angles for testing only
// #########################################################################

void testPitch(void)
{
  tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  tft.drawString("Pitch test", 128, 20, 2);

  for (int p = 0; p > -160; p--) {
    delay(REDRAW_DELAY / 2);
    updateHorizon(0, p);
  //delay(2000);
  }

  for (int p = -160; p < 160; p++) {
    delay(REDRAW_DELAY / 2);
    updateHorizon(0, p);
   // delay(2000);
  }


  for (int p = 160; p > 0; p--) {
    delay(REDRAW_DELAY / 2);
    updateHorizon(0, p);
   // delay(2000);
  }

  tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  tft.drawString("          ", 128, 10, 1);
}

//Below are Function calls frorm AHD 9DOF sensor program. Moving the whole chunk from line 194 in AHD to line 340. 
//Their may be some variable conflicts

// vector math
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
//Why is the below Vector statement in the middke of nowhere 
void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

// function to subtract offsets and apply scale/correction matrices to IMU data

void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];
  Gxyz[0] = Gscale * (imu.gx - G_offset[0]);
  Gxyz[1] = Gscale * (imu.gy - G_offset[1]);
  Gxyz[2] = Gscale * (imu.gz - G_offset[2]);

  Axyz[0] = imu.ax;
  Axyz[1] = imu.ay;
  Axyz[2] = imu.az;
  Mxyz[0] = imu.mx;
  Mxyz[1] = imu.my;
  Mxyz[2] = imu.mz;

  //apply accel offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply mag offsets (bias) and scale factors from Magneto

  for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)
// sjr 3/2021
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
    // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;  //observed West horizon vector W = AxM
  float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Measured horizon vector = a x m (in body frame)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  // Normalise horizon vector
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f) return; // Handle div by zero

  norm = 1.0f / norm;
  hx *= norm;
  hy *= norm;
  hz *= norm;

  // Estimated direction of Up reference vector
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;

  // estimated direction of horizon (West) reference vector
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);

  // Error is the summed cross products of estimated and measured directions of the reference vectors
  // It is assumed small, so sin(theta) ~ theta IS the angle required to correct the orientation error.

  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);
 
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback
    gx += Ki * eInt[0];
    gy += Ki * eInt[1];
    gz += Ki * eInt[2];
  }


  // Apply P feedback
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;

 //update quaternion with integrated contribution
 // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
gx = gx * (0.5*deltat); // pre-multiply common factors
gy = gy * (0.5*deltat);
gz = gz * (0.5*deltat);
float qa = q1;
float qb = q2;
float qc = q3;
q1 += (-qb * gx - qc * gy - q4 * gz);
q2 += (qa * gx + qc * gz - q4 * gy);
q3 += (qa * gy - qb * gz + q4 * gx);
q4 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
