/* Include necessary header files */
#include <Wire.h>
#include <MsTimer2.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#include <SoftwareSerial.h>
SoftwareSerial BTserial(10, 13); // RX | TX
// Connect the HC-06 TX to the Arduino RX on pin 10. 
// Connect the HC-06 RX to the Arduino TX on pin 13.

/* TB6612FNG motor driver signal pins */
#define IN1_L 7
#define IN1_R 12
#define PWM_L 5
#define PWM_R 6
#define STBY 8

/* Encoder count signal pins */
#define PinA_left 2
#define PinA_right 4

/* Sensor Reading Variables */
MPU6050 mpu; //Instantiate a MPU6050 object with the object name MPU.
int16_t ax, ay, az, gx, gy, gz; // Variables for IMU readings
volatile long right_encoder = 0;
volatile long left_encoder = 0;
volatile long left_encoder_pos = 0;
/* 
 The volatile long type is used to ensure that the value is valid when the external 
 interrupt pulse count value is used in other functions 
 */
float num_of_encoder_counts_per_rev = 780.0;
float thousand_by_num_of_encoder_counts_per_rev = 1000.0/num_of_encoder_counts_per_rev;

/* IMU Callibration Variables */
long gyroXCalli = -422/131.0, gyroYCalli = -53/131.0, gyroZCalli = 310/131.0; // Obtained by callibrating gyroscope values
long accelXCalli = -97/16384.0, accelYCalli = -57/16384.0, accelZCalli = (18196-16384)/16384.0; // Obtained by callibrating accelerometer values

/*long gyroXCalli = 11377/131.0, gyroYCalli = -847/131.0, gyroZCalli = 168/131.0; // Obtained by callibrating gyroscope values
long accelXCalli = 398/16384.0, accelYCalli = -175/16384.0, accelZCalli = 15827/16384.0; // Obtained by callibrating accelerometer values*/


/* Motor PWM Input Values */
long motor_left = 0;
long motor_right = 0;

/* Time variables */
unsigned long time;
unsigned long prev_time_encoder = 0;
unsigned long startTime_left = 0;
unsigned long startTime_right = 0;
int sampling_rate = 5; // in milliseconds

/* Encoder to speed measurement variables */

int encoder_count_max = 20;
float motor_left_ang_vel = 0;
float motor_right_ang_vel = 0;

/* Robot parameters */

float wheel_rad = 0.0335; // radius of the wheel in metres
float lat_dist = 0.194; // distance between ends of the wheels in metres

/********************Initialization settings********************/
void setup() {
  
  /* TB6612FNGN Motor Driver module control signal initialization */
  pinMode(IN1_L, OUTPUT); //Control the direction of motor 1, 1 is forward, 0 is reverse
  pinMode(IN1_R, OUTPUT); //Control the direction of motor 1, 1 is forward, 0 is reverse
  pinMode(PWM_L, OUTPUT); //PWM of left motor
  pinMode(PWM_R, OUTPUT); //PWM of right motor
  pinMode(STBY, OUTPUT); //enable TB6612FNG
  
  /* Initializing motor drive module */
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN1_R, LOW);
  digitalWrite(STBY, HIGH);
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
  
  /* Initialize I2C bus */
  Wire.begin(); //Add I2C bus sequence
  Serial.begin(57600); //Open the serial port and set the baud rate
  delay(1500);
  mpu.initialize(); //Initialization MPU6050
  delay(2);

  /* 
   Uncomment the next two lines only once and store the callibration values in the 
   global variables defined for callibration
  */
  callibrateGyroValues();
  callibrateAccelValues();

  time = millis();
  startTime_left = millis();
  startTime_right = millis();
  
  /* Interrupt function to count the encoder pulses */
  attachInterrupt(digitalPinToInterrupt(PinA_left), encoder_left, CHANGE);  
  /* 
  Timing interrupt settings, using MsTimer2. Because PWM uses a timer to control the 
  duty cycle, it is important to look at the pin port corresponding to the timer when 
  using timer.
  */
  MsTimer2::set(sampling_rate, mainfunc);
  MsTimer2::start();
  
  Serial.print("Setup Done!\n");

  BTserial.begin(57600);
  randomSeed(analogRead(0));
}

/***************************************************************************************/
/*
 This section contains functions that you can use to build your controllers
*/
/***************************************************************************************/

/* 
encoder_left() counts encoder pulses of the left wheel motor and stores it in a 
global variable 'left_encoder'
*/
void encoder_left(){left_encoder++;
left_encoder_pos++;}
  
/* 
encoder_right() counts encoder pulses of the right wheel motor and stores it in a 
global variable 'right_encoder'
*/
void encoder_right(){right_encoder++;}


/* 
SetLeftWheelSpeed() takes one input which will be set as the PWM input to the left motor.
If the value is outside the range (-255,255), then the input will be saturated and the 
motor PWM will be set. The value is also written to the global variable 'motor_left'
*/
void SetLeftWheelSpeed(double speed_val)
{
  // Saturate the input to the range (-255,255)
  if (speed_val > 255) { speed_val = 255; }
  else if (speed_val < -255) { speed_val = -255; }
  
  motor_left = speed_val;
  
  if (speed_val < 0)
  {
    digitalWrite(IN1_L, 1);
    analogWrite(PWM_L, -speed_val);
  }
  else
  {
    digitalWrite(IN1_L, 0);
    analogWrite(PWM_L, speed_val);
  }
}

/* 
SetRightWheelSpeed() takes one input which will be set as the PWM input to the right motor.
If the value is outside the range (-255,255), then the input will be saturated and the 
motor PWM will be set. The value is also written to the global variable 'motor_right'
*/
void SetRightWheelSpeed(double speed_val)
{
  // Saturate the input to the range (-255,255)
  if (speed_val > 255) { speed_val = 255; }
  else if (speed_val < -255) { speed_val = -255; }
  
  motor_right = speed_val;
  
  if (speed_val < 0)
  {
    digitalWrite(IN1_R, 1);
    analogWrite(PWM_R, -speed_val);
  }
  else
  {
    digitalWrite(IN1_R, 0);
    analogWrite(PWM_R, speed_val);
  }
}

/*
readIMU() creates an MPU6050 class object and calls the function to read the six axis IMU.
The values are stored in the global variables ax,ay,az,gx,gy,gz where ax,ay,az are the 
accelerometer readings and gx,gy,gz are the gyroscope readings. 
*/
void readIMU()
{
  MPU6050 mpu_obj;
  mpu_obj.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
}

/*
readEncoder() takes the encoder pulse counts and calculates the angular velocities of the
wheels and stores it in the global variables 'motor_left_ang_vel' and 'motor_right_ang_vel'
*/
void readEncoder()
{ 
  // Encoder Calculations
  // angular velocity = (encoder_reading/num_of_counts_per_rotation)*(2*pi/sampling_time)
  
  motor_left_ang_vel = (float) 2 * 3.1415 * left_encoder * (float)thousand_by_num_of_encoder_counts_per_rev / (float)(time - startTime_left);  
  if (motor_left < 0){
    motor_left_ang_vel = -motor_left_ang_vel;}
  startTime_left = time;
  left_encoder = 0;  

  motor_right_ang_vel = motor_left_ang_vel;
  
//  motor_right_ang_vel = (float) 2 * 3.1415 * right_encoder * (float)thousand_by_num_of_encoder_counts_per_rev / (float)(time - startTime_right);  
//  if (motor_right < 0){
//    motor_right_ang_vel = -motor_right_ang_vel;}
//  startTime_right = time;
//  right_encoder = 0;  
}

/* 
printIMU() prints the IMU readings to the serial monitor in the following format:
ax,ay,az,gx,gy,gz
*/
void printIMU()
{
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(gx);
  Serial.print(',');
  Serial.print(gy);
  Serial.print(',');
  Serial.print(gz);
  Serial.print('\n');
}

/* 
printEncoder() prints the encoder readings to the serial monitor in the following format:
motor_left_ang_vel, motor_right_ang_vel
*/
void printEncoder()
{
  Serial.print(motor_left_ang_vel);
  Serial.print(',');
  Serial.print(motor_right_ang_vel);
  Serial.print('\n');
}

/* 
printAllData() prints the IMU readings, encoder readings and PWM inputs to the motor to
the serial monitor in the following format:
ax,ay,az,gx,gy,gz,motor_left_ang_vel,motor_right_ang_vel,motor_left,motor_right
*/
void printAllData()
{
  Serial.print(time);
  Serial.print(',');
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(gx);
  Serial.print(',');
  Serial.print(gy);
  Serial.print(',');
  Serial.print(gz);
  Serial.print(',');
  Serial.print(motor_left_ang_vel);
  Serial.print(',');
  Serial.print(motor_right_ang_vel);
  Serial.print(',');
  Serial.print(motor_left);
  Serial.print(',');
  Serial.print(motor_right);
  Serial.print('\n');
}

/*
callibrateGyroValues() gets the gyroscope readings n times and calculates the average of
the values to find the sensor bias. The callibration values are printed as:
gyroXCalli,gyroYCalli,gyroZCalli
*/
void callibrateGyroValues() 
{
    int n = 10000;
    for (int i=0; i < n; i++) 
    {
      readIMU();
      gyroXCalli = gyroXCalli + gx;
      gyroYCalli = gyroYCalli + gy;
      gyroZCalli = gyroZCalli + gz;
    }
    gyroXCalli = gyroXCalli/n;
    gyroYCalli = gyroYCalli/n;
    gyroZCalli = gyroZCalli/n;
    Serial.print(gyroXCalli);
    Serial.print(',');
    Serial.print(gyroYCalli);
    Serial.print(',');
    Serial.print(gyroZCalli);
    Serial.print('\n');
}

/*
callibrateAccelValues() gets the accelerometer readings n times and calculates the average of
the values to find the sensor bias. The callibration values are printed as:
accelXCalli,accelYCalli,accelZCalli
*/
void callibrateAccelValues() 
{
    int n = 10000;
    for (int i=0; i < n; i++) 
    {
      readIMU();
      accelXCalli = accelXCalli + ax;
      accelYCalli = accelYCalli + ay;
      accelZCalli = accelZCalli + az;
    }
    accelXCalli = accelXCalli/n;
    accelYCalli = accelYCalli/n;
    accelZCalli = accelZCalli/n;
    Serial.print(accelXCalli);
    Serial.print(',');
    Serial.print(accelYCalli);
    Serial.print(',');
    Serial.print(accelZCalli);
    Serial.print('\n');
}

/***************************************************************************************/
/***************** Write your custom variables and functions below *********************/
/***************************************************************************************/

#define alpha 0.90
#define lpf 0.85
#define lpf2 0.85
#include <math.h>

double gyroAng, accAng, filteredAng, prevAng = 0;
double accZ, accY, accX, gyZ, gyY, gyX = 0;
double filteredVel, prevFilteredVel = 0;
double wheel_pos, wheel_vel = 0;
double state[4] = {0};

bool start = true;
float prevT = millis();

void stateUpdate()
{
  // Calibration 
  double accZ = (double) az/16384; double accY = (double) ay/16384; double accX = (double) ax/16384;
  double gyZ = (double) gz/131; double gyY = (double) gy/131; double gyX = (double) gx/131;
  accZ -= accelZCalli; accY -= accelYCalli; accX -= accelXCalli;
  gyZ -= gyroZCalli; gyY -= gyroYCalli; gyX -= gyroXCalli;

  // LPF Gyro X
  filteredVel = gyX * 3.1415/180; // radians
  accAng = (double) atan(accY/sqrt(pow(accZ,2)+pow(accY,2)));

  // Complementary filter
  if(start)
  {
    filteredAng = accAng;
    start = !start;
  }
  
  float currT = millis();
  gyroAng = filteredAng + (currT-prevT)*filteredVel/1000;  

  // State estimate
  filteredAng = alpha*gyroAng + (1-alpha)*(accAng);
  filteredAng = lpf*filteredAng + (1-lpf)*prevAng;
  filteredVel = lpf2*filteredVel + (1-lpf2)*prevFilteredVel;
  prevAng = filteredAng;
  prevFilteredVel = filteredVel;
  
  wheel_vel = motor_left_ang_vel * wheel_rad;
  wheel_pos += wheel_vel*((currT-prevT)/1000);

  prevT = currT;
  double gyroAngleX_prev = gyroAngleX;
  double gyroAngleY_prev = gyroAngleY;  
  gyroAngleX = (gyroAngleX_prev + GyroX * elapsedTime); // rad/s * s = rad
  // States
  state[0] = wheel_pos;
  state[1] = wheel_vel;
  state[2] = filteredAng;
  state[3] = filteredVel;
}

// CONTROLLERS
const int Kp = 6000;
const int Kd = 100;

double PD_control()
{
  return Kp*state[2] + Kd*state[3];
}

const float inputGain = 1.52272727273*255/12; // Force --> PWM
const float freq[5] = {0.300, 0.5335, 0.9487, 1.6870, 3.000};
double sin_ang = 0;
double rand_val = 0;

// LQR Gains from MATLAB
//const double K[4] = {-10, -67.8, 2254.7, 38.1};
//const float K[4] = {-100, -136.226, 915.2956, 14.6294};
const float K[4] = {-20,-45.97, 2500, 205};
float ref[4] = {0, 0, 0, 0};

double LQR_control()
{

  double out = 0;
  
  for(int i = 0; i < 4; i++)
  {
    out += K[i]*(state[i] - ref[i]);
  }
  
  return (out);
  
}


double control_total, lqr_control = 0;
double data_MATLAB = 0.0;

void printStateSerial()
{
  Serial.print("* ");
  Serial.print(state[0]);
  Serial.print(' ');
  Serial.print(state[1]);
  Serial.print(' ');
  Serial.print(state[2]);
  Serial.print(' ');
  Serial.print(state[3]);
  Serial.print(' ');
  Serial.print(data_MATLAB);
  Serial.print(" *");
  Serial.print("\n");
}
void printState()
{
  BTserial.print("* ");
  BTserial.print(state[0]);
  BTserial.print(' ');
  BTserial.print(state[1]);
  BTserial.print(' ');
  BTserial.print(state[2]);
  BTserial.print(' ');
  BTserial.print(state[3]);
  BTserial.print(' ');
  BTserial.print(data_MATLAB);
  BTserial.print(" *");
  BTserial.print("\n");
}
const byte buffSize = 32;
char inputSeveral[buffSize]; // space for 31 chars and a terminator

byte maxChars = 12; // a shorter limit to make it easier to see what happens
                           //   if too many chars are entered
bool reset_check = true;

float getdatafromMATLAB() 
{

    // Function credits: https://forum.arduino.cc/index.php?topic=236162.0

    // this function takes the characters from the serial input and converts them
    // to a single floating point value using the function "atof()"
     
    // a similar approach can be used to read an integer value if "atoi()" is used

    // first read severalChars into the array inputSeveral
    inputSeveral[0] = 0; 
    byte charCount = 0;  
    byte ndx = 0;
    
    if (BTserial.available() > 0) {
      if(reset_check)
      {
        ref[0] = 0.25;
        wheel_pos = 0;
        reset_check = !reset_check;
      }
      
      long time = micros();
      while (BTserial.available() > 0) { 
        if (ndx > maxChars - 1) {
          ndx = maxChars;
        } 
        inputSeveral[ndx] = BTserial.read();
        ndx ++;        
        charCount ++;
      }
      if (ndx > maxChars) { 
        ndx = maxChars;
      }
      inputSeveral[ndx] = 0;

       // and then convert the string into a floating point number  
      if(abs(atof(inputSeveral)) <= 1.00) int x=0;
      
      else data_MATLAB = atof(inputSeveral);
     
    } 
}
/*
mainfunc() is the function that is called at your specified sampling rate. The default 
sampling rate is 5ms. This function will be called at every sampling instance.
*/
void mainfunc()
{
  /* Do not modify begins*/
  sei();
  time = millis(); 
  if (time - prev_time_encoder >= encoder_count_max)
  {
    readEncoder();
    prev_time_encoder = time;

  }
  readIMU();
  /* Do not modify ends*/
  /*Write your code below*/
  /***********************/
  
  // Set reference and reset wheel_position for reference tracking

  stateUpdate();
  getdatafromMATLAB();
  printState();
  
  double control = LQR_control();

  lqr_control = control;
  control_total = control + data_MATLAB*255/12;
  SetLeftWheelSpeed(control_total); 
  SetRightWheelSpeed(control_total);
}

void loop()
{

}
