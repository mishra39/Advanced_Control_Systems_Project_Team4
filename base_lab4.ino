/* Include necessary header files */
//#include <Wire.h>
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
/* 
 The volatile long type is used to ensure that the value is valid when the external 
 interrupt pulse count value is used in other functions 
 */
float num_of_encoder_counts_per_rev = 780.0;
float thousand_by_num_of_encoder_counts_per_rev = 1000.0/num_of_encoder_counts_per_rev;

/* IMU Callibration Variables */ 
//long gyroXCalli =-101 , gyroYCalli =-195 , gyroZCalli = 58; // Obtained by callibrating gyroscope values
//long accelXCalli =44 , accelYCalli =-455 , accelZCalli =18116-16384; // Obtained by callibrating accelerometer 

long gyroXCalli =-91 , gyroYCalli =-186 , gyroZCalli = 60; // Obtained by callibrating gyroscope values
long accelXCalli =-214 , accelYCalli =-439 , accelZCalli =18088-16384; // Obtained by callibrating accelerometer 

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
float pos=0,posdot=0,posdotprev=0.0,posprev=0.0;
void setup() {
  pos=0;
  posprev=0.0;
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
  Serial.begin(9600); //Open the serial port and set the baud rate
  delay(1500);
  mpu.initialize(); //Initialization MPU6050
  delay(2);

  /* 
   Uncomment the next two lines only once and store the callibration values in the 
   global variables defined for callibration
  */
  //callibrateGyroValues();
  //callibrateAccelValues();
  
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

  BTserial.begin(9600);
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
void encoder_left(){left_encoder++;}
  
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
int t,prev_time=0,t1,t1p=0;
float th,alpha=0.95,acc,a=0.1;
float th_prev=0;

float y[4],u;
int C[3][4]={{1,0,0,0},{0,0,1,0},{0,0,0,1}};
//float x_est[4]={pos,posdot,th,(gx-gyroXCalli)/131};
float x_est[4];
//double Klqr[4];
double c;
int Kp,Kd;
float data_MATLAB=0.0,prev=0.0,motopwm;

void compfilter()
{
  t=millis();
  acc=(atan((ay-accelYCalli)/sqrt(pow(ax-accelXCalli,2)+pow(az-accelZCalli,2))));
  th=alpha*(th_prev+((gx-gyroXCalli)/131)*(t-prev_time)*0.001*(3.1415/180))+(1-alpha)*acc;
  prev_time=t;
  th_prev=th;
  //BTserial.println(th);
  //Serial.println(th*(180/3.1415));
}

void linearposvel()
{
  t1=millis();
  posdot=(motor_left_ang_vel)*wheel_rad;
  pos=posprev+(posdot+posdotprev)*0.5*(t1-t1p)*0.001;
  t1p=t1;
  posdotprev=posdot;
  posprev=pos;
  //BTserial.print(posdot);
  //BTserial.print(',');
  //BTserial.print(pos,3);
  //BTserial.print('\n');
}

void StateEstimator()
{
  compfilter();
  linearposvel();
  /*x_est[0]=pos;
  x_est[1]=posdot;
  x_est[2]=th;
  x_est[3]=(gx-gyroXCalli)/131;
  for(int i=0;i<3;i++)
  { y[i]=0;
    for(int j=0;j<4;j++)
    {
      y[i]=y[i]+C[i][j]*x_est[j];
    }
  }
  for(int k=0;k<4;k++)
  {
    Serial.print(y[k]);
    Serial.print(',');
  }
  Serial.print('\n');*/
  BTserial.print(millis());
  BTserial.print(',');
  //BTserial.print(pos);
  //BTserial.print(',');
  //BTserial.print(th);
  //BTserial.print(',');
  BTserial.print(((gx-gyroXCalli)/131)*(3.1415/180));
  BTserial.print(',');
  BTserial.print(a*((gx-gyroXCalli)/131)*(3.1415/180));
  BTserial.print('\n');
}

void PDController()
{
  Kp=5500;
  Kd=250;
  compfilter();
  //c=Kp*(th)+Kd*((gx-gyroXCalli)/131);
  c=Kp*(th)+Kd*(a*((gx-gyroXCalli)/131)*(3.1415/180));
  SetLeftWheelSpeed(c);
  SetRightWheelSpeed(c);
  BTserial.print(millis());
  BTserial.print(',');
  BTserial.print(th);
  BTserial.print('\n');
}
void lqr()
{
  double Klqr[4]={-1,-4.83,120.87,71.08},motopwm;
  //double Klqr[4]={-1,-3.6016,70.2064,31.9539};
  u=0;
  compfilter();
  linearposvel();
  x_est[0]=pos;
  x_est[1]=posdot;
  x_est[2]=th;
  x_est[3]=a*((gx-gyroXCalli)/131)*(3.1415/180);
  Serial.print(x_est[0]);
  Serial.print(",");
  Serial.print(x_est[1]);
  Serial.print(",");
  Serial.print(x_est[2]);
  Serial.print(",");
  Serial.print(x_est[3]);
  Serial.print("\n");
  for(int i=0;i<4;i++)
  {
    u=u+Klqr[i]*x_est[i];
  }
  //u=u+20*sin(2*3.1415*3*millis()/1000);
  //randNumber = random(10);
  motopwm=u-data_MATLAB;
  //Serial.println("LQR");
  //Serial.println(u);
  //Serial.println("MATLAB");
  //Serial.println(data_MATLAB);
  motopwm=motopwm*(255/12);
  //Serial.println(motopwm);
  SetLeftWheelSpeed(motopwm);
  SetRightWheelSpeed(motopwm);
}
/*
 The below code parses the data received from MATLAB
*/

const byte buffSize = 32;
char inputSeveral[buffSize]; // space for 31 chars and a terminator

byte maxChars = 12; // a shorter limit to make it easier to see what happens
                           //   if too many chars are entered


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
       
      data_MATLAB = atof(inputSeveral);
      if(data_MATLAB!=0)
      {
        prev=data_MATLAB;
      }
      if(data_MATLAB==0)
      {
        data_MATLAB=prev;
      }
      
       // atof gives 0.0 if the characters are not a valid number
      //Serial.print("Data from MATLAB -- ");
      //Serial.println(data_MATLAB, 3); // the number specifies how many decimal places
      //Serial.println(pos);
      //Serial.println(millis());
     
    } 
}

/*
mainfunc() is the function that is called at your specified sampling rate. The default 
sampling rate is 5ms. This function will be called at every sampling instance.
*/
void mainfunc()
{
  /* Do not modify begins*/
  float control;
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

  // The getdatafromMATLAB() function parses the data from the serial buffer which is sent by MATLAB and then
  // stores it in a float variable 'data_MATLAB'. Process it as desired and set the motor PWM. Note that the 
  // serial buffer might sometimes give an invalid input upon which the 'data_MATLAB' variable will be read
  // as 0.0. If this happens, ignore the zero readings and use the previous readings. Else, increase sampling
  // which will reduce the occurence of such errors 
  
  getdatafromMATLAB();
  lqr();
  //BTserial.println(control);
  BTserial.print("* ");
  BTserial.print(pos);
  BTserial.print(' ');
  BTserial.print(posdot);
  BTserial.print(' ');
  BTserial.print(th*(180.0/3.1415));
  BTserial.print(' ');
  BTserial.print(a*((gx-gyroXCalli)/131)*(3.1415/180));
  BTserial.print(' ');
  BTserial.print(u);

  //BTserial.print(' ');
  
  // The data sent from Arduino to MATLAB is set in a string of the form "* 12.34 45.67 *". The * character at the beginning and 
  // end of the string is set to debug if the string is received correctly by MATLAB and to parse it appropriately. The MATLAB
  // function given along with this script parses this string and stores the numbers in an array which you can use in the MPC
  // function you will write to get the optimal control inputs.
  
  //BTserial.print("* ");
  //BTserial.print(millis());
  //BTserial.print(" ");
  //BTserial.print(millis());
  BTserial.print(" *");
  BTserial.print("\n");
  /***********************/
}

void loop()
{

}
