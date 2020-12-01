/* Include necessary header files */
#include <Wire.h>
#include <MsTimer2.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

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
long gyroXCalli = -422, gyroYCalli = -53, gyroZCalli = 310; // Obtained by callibrating gyroscope values
long accelXCalli = -97, accelYCalli = -57, accelZCalli = 18196-16384; // Obtained by callibrating accelerometer values

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

/* Complementary Filter Parameters */
double theta_prev = 0.0, alpha = 0.7, theta_curr = 0.0;

/*Gyro Low Pass filter parameters*/
double lpAlpha = 0.5; // Filtering coefficient
double posX[] = {0,0}; // Previous and current position values
double posY[] = {0,0};
double psi[] = {0,0};
double omega[] = {0,0};
double kmat[4][3]; // matrix of range-kutta coefficients
const int n = 1; // index variable for accessing gyroFiltered[]
double distance = 0; // distance moved by the Robot
double linVel = 0; // linear Velocity 

/* Accelerometer and Gyroscope measurements */
double theta_acc = 0.0; // Filtering coefficient
double elapsedTime, currentTime, previousTime;
int c = 0;
double deg_to_rad = 0.01745;

double AccX, AccY, AccZ;
double GyroX, GyroY, GyroZ;
double accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
double roll, pitch, yaw;
double AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;

/* PID Parameters */
double setPoint = 0; // desired tilt angle
double Kp = 2500 , Ki = 0, Kd = 20;
double error, error_d, error_i, error_prev,u, pwm_pulse;

/* LQR Controller Parameters*/
//double Kpos = -1, Ktheta = 2500, KthetaDot = 10;
//double Kpos =  -31.62, K_xdot = -145.97, Ktheta = 3310, KthetaDot = 325.4;
double Kpos =  -31.62, K_xdot = -145.97, Ktheta = 2700, KthetaDot = 225.4;
double pos_prev = 0; double pos_curr = 0;
double elapsedTimeLQR, currentTimeLQR, previousTimeLQR, avg_ang_vel;
double deadZoneMag = 0.00;
static int loop_count = 0;
double sin_val = 0;
/*Good gains: Kp, Kd 2500-3000, 0
 * double Kpos = -1, Ktheta = 401, KthetaDot = 5;

*/
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
//  callibrateGyroValues();
//  callibrateAccelValues();
  
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
  if (isnan(motor_left_ang_vel))
  {
    motor_left_ang_vel = 0;
  }
  // Serial.println(motor_left_ang_vel);
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
// Function to calculate filtered angles from accelerometer and gyroscope readings
void calc_filt_ang()
{
  readIMU();
  readEncoder();
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (ax-accelXCalli) / 16384.0; // X-axis value
  AccY = (ay-accelYCalli) / 16384.0; // Y-axis value
  AccZ = (az-accelZCalli) / 16384.0; // Z-axis value
  
  // Calculating Roll and Pitch from the accelerometer data(radians)
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2)))); 
  accAngleY = (atan(-1*AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))));
  
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

  GyroX = (gx - gyroXCalli)*deg_to_rad / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (gy - gyroYCalli)*deg_to_rad / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroZ = (gz - gyroZCalli)*deg_to_rad / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  
  double gyroAngleX_prev = gyroAngleX;
  double gyroAngleY_prev = gyroAngleY;  
  gyroAngleX = (gyroAngleX_prev + GyroX * elapsedTime); // rad/s * s = rad
  gyroAngleY = (gyroAngleY_prev + GyroY * elapsedTime); // rad/s * s = rad
  yaw = yaw + GyroZ*elapsedTime;

// Complementary filter - combine acceleromter and gyro angle values
  pitch = 0.90 * gyroAngleX + 0.10 * accAngleX;  
  
// Calculating position using Runge-Kutta

  avg_ang_vel = (motor_left_ang_vel + motor_right_ang_vel) / 2;
  if (!isinf(avg_ang_vel))
  {
    linVel = avg_ang_vel*wheel_rad; // linear velocity of the Robot
    distance += linVel*elapsedTime; 
  }
  
  Serial.print("Filtered Angle (radians): ");
  Serial.println(pitch);
  Serial.print("Distance ");
  Serial.println(distance);
  Serial.println(linVel);
}

/* Function to implement PID Controller*/
void PID_calc()
{
  calc_filt_ang();
  error = setPoint - pitch;
  error_d = 0 - gyroAngleX;
  error_i = 0;
  u = (Kp*error + Ki*error_i + Kd*error_d);
  constrain(u,-255,255);
  SetLeftWheelSpeed(-u);
  SetRightWheelSpeed(-u); 
  Serial.print("PID PWM: ");
  Serial.println(u);
}

void lp_filter_test()
{
  //readIMU();
  calc_filt_ang();
  alpha = 0.80;
  omega[n] = alpha * GyroX + (1-alpha) * omega[n-1];
  omega[n-1] = omega[n];
  Serial.print(accAngleX);
  Serial.print(',');
  BTserial.println(gyroAngleX);
}

double state[4] = {0.00, 0.00, 0.00, 0.00};

/* LQR Controller Implementation*/
double lqr_u = 0.00;
double mpc_u = 0.00;

double LQR_control()
{
  double freq = 0.96; // frequency in Hz
  double sine_in = 0;//105*1.5*sin((millis()/1000) * freq);
  calc_filt_ang();
  u = Kpos*distance + K_xdot*linVel + Ktheta*(pitch) + KthetaDot*(gyroAngleX) + sine_in;
  Serial.println(millis()/1000);
 /* if (isnan(distance) || isinf(distance)) {u = Ktheta*(setPoint-pitch) + KthetaDot*(0-gyroAngleX);}
  else
  {
    u = Kpos*distance + Ktheta*(setPoint-pitch) + KthetaDot*(0-gyroAngleX);
    Serial.print(distance);
  }*/
  
  constrain(u,-255,255);
  BTserial.print(distance);
  BTserial.print(',');
  BTserial.println(pitch);
  return u;
}




/*
 The below code parses the data received from MATLAB
*/

const byte buffSize = 32;
char inputSeveral[buffSize]; // space for 31 chars and a terminator

byte maxChars = 12; // a shorter limit to make it easier to see what happens
                           //   if too many chars are entered
double data_MATLAB = 0.0;
bool reset_check = true;
float ref[4] = {0,0,0,0};
void printState()
{
  BTserial.print("Pos: ");
  BTserial.print(state[0]);
  BTserial.print(", Vel: ");
  BTserial.print(state[1]);
  BTserial.print(", Pitch");
  BTserial.print(state[2]);
  BTserial.print(", Ang. Vel: ");
  BTserial.print(state[3]);
  BTserial.print(", LQR_u: ");
  BTserial.print(lqr_u);
  BTserial.print(", MPC_u: ");
  BTserial.print(data_MATLAB);
  BTserial.print("\n");
}
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
      if (reset_check)
      {
        distance = 0.00;
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
      /*data_MATLAB = atof(inputSeveral); // atof gives 0.0 if the characters are not a valid number
      
      Serial.print("Data from MATLAB -- ");
      Serial.println(data_MATLAB, 3); // the number specifies how many decimal places*/
    }
}

void mpc_control()
{
  
}
/*
mainfunc() is the function that is called at your specified sampling rate. The default 
sampling rate is 5ms. This function will be called at every sampling instance.
*/
double count = 0;
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
  //Serial.println("Printing through USB Serial Port");
  //BTserial.println("Printing through Bluetooth");
  /***********************/
  //
  //PID_calc();
 //LQR();
  //lp_filter_test();
  getdatafromMATLAB();
  mpc_u = data_MATLAB * (255 /12);   // convert PWM to Voltage
  Serial.print("MPC Input: ");
  Serial.println(mpc_u);
  double lqr_control = LQR_control();
  double total_input = mpc_u + u;
  SetLeftWheelSpeed(total_input);
  SetRightWheelSpeed(total_input);
  Serial.print("LQR Input: ");
  Serial.println(u);
  Serial.print("Total Input: ");
  Serial.println(total_input);
  BTserial.print("MPC Input: ");
  BTserial.println(mpc_u);
  BTserial.print("LQR Input: ");
  BTserial.println(u);
  BTserial.print("Total Input: ");
  BTserial.println(total_input);
}

void loop()
{
}
