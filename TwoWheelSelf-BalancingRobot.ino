 #include "PinChangeInt.h"
#include <Wire.h>
#include "MsTimer2.h"

int IN1M = 7;
int IN2M = 6;
int IN3M = 13;
int IN4M = 12;
int PWMA = 9;
int PWMB = 10;

#define run_car '1'
#define back_car '2'
#define left_car '3'
#define right_car '4'
#define stop_car '0'
enum{
  enSTOP =0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT
} enCarState;
int g_carstate = enSTOP;
String inputString = "";

//Encoder count signal
int PinA_left = 2;  //Interrupt 0
int PinA_right = 4; //Interrupt 1

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll, angle_yaw;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

float dt = 0.005;
float angleoutput;
double speedoutput = 0;
double E=0;
double kp = 45, ki =0 , kd = 3;
double kp_speed =2.5, ki_speed = 0.25, kd_speed = 0;
double kp_turn = 23, ki_turn = 0, kd_turn = 0.3;
double p0 = 0;

float angle0 = 0; 
int pwm1,pwm2,stopl,stopr,pulseleft,pulseright;
float speeds_filterold = 0;
float positions = 0;
volatile long count_right = 0;
volatile long count_left = 0;
int speedcc = 0;
int front = 0, back = 0, turnl = 0, turnr = 0, spinl = 0, spinr = 0, turnOutput = 0;
int turnCount;
int rpluse = 0;
int lpluse = 0;
int sumam;
void resetCarState(){
  front = 0; 
  back = 0;
  turnl = 0;
  turnr = 0;
  spinl = 0;
  spinr = 0;
  turnOutput = 0;
}

void countpluse()
{
  lpluse = count_left;
  rpluse = count_right;
  count_left = 0;
  count_right = 0;
  
  if ((pwm1 < 0) && (pwm2 < 0))//The direction of the trolley motion is judged to be backward (PWM is the motor voltage is negative) and the number of pulses is negative.
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((pwm1 > 0) && (pwm2 > 0))//The direction of the trolley motion is judged to be forward (PWM is the motor voltage is positive) and the number of pulses is negative.
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((pwm1 < 0) && (pwm2 > 0))//The direction of the trolley motion is judged to be forward (PWM is the motor voltage is positive) and the number of pulses is negative.
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((pwm1 > 0) && (pwm2 < 0))//The direction of trolley motion determines that the number of left rotation and right pulse is negative, and the number of left pulses is positive.
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }
  pulseright += rpluse;
  pulseleft += lpluse;
  sumam = pulseright + pulseleft;
}
/********************Angle PID********************/
void angleout()
{
  E += angle_pitch_output;
  angleoutput = kp * angle_pitch_output +ki*E + kd * gyro_x;
  
}

void mpu_6050_calibration(){
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
                            
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000; 
}
/********************Interrupt timing 5ms timing interrupt********************/
void inter()
{
  sei();//enable interrupt
  countpluse();//Pulse superposition subfunction
  angle_data();
  angleout();
  speedcc++;
  if (speedcc >= 10)//50ms enter speed control
  {
    speedoutput = speedpiout(kp_speed, ki_speed, kd_speed, front, back, p0);
    speedcc = 0;
  }
  turnCount++;
  if(turnCount > 4){
    turnOutput = turnSpin(turnl, turnr, spinl, spinr, kp_turn, kd_turn,gyro_z);
    turnCount = 0;
  }
  BalanceCar(speedoutput, turnOutput, angle_pitch_output, front, back, IN1M, IN2M, IN3M, IN4M, PWMA, PWMB);//Total PWM output of car
}

/********************Interrupt timing 5ms timing interrupt********************/

/********************Initialization settings********************/
void setup() {  
  // TB6612FNGN Driver module control signal initialization
  pinMode(IN1M, OUTPUT);//Control the direction of motor 1, 01 is positive, 10 is reverse.
  pinMode(IN2M, OUTPUT);
  pinMode(IN3M, OUTPUT);//Control the direction of motor 2, 01 is positive, 10 is reverse.
  pinMode(IN4M, OUTPUT);
  pinMode(PWMA, OUTPUT);//PWM of left motor
  pinMode(PWMB, OUTPUT);//PWM of right motor
  //Initializing motor drive module
  digitalWrite(IN1M, 0);
  digitalWrite(IN2M, 1);
  digitalWrite(IN3M, 0);
  digitalWrite(IN4M, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  //Speed dial input
  pinMode(PinA_left, INPUT);  
  pinMode(PinA_right, INPUT);
  delay(1500);
  pwm1 = 0;
  pwm2 = 0;
  Serial.begin(9600);
  Wire.begin();
  setup_mpu_6050_registers(); 
  mpu_6050_calibration();
  
  MsTimer2::set(5, inter);
  MsTimer2::start();
}

void loop() 
{
  attachInterrupt(digitalPinToInterrupt(PinA_left), Code_left, CHANGE);
  attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);
  
  while(Serial.available()){
    char c = Serial.read();
    inputString += c;
  }
  if(inputString.length()>0){
    Serial.println(inputString);
    if(inputString == "F"){
      g_carstate = enRUN;
    }
    if(inputString == "B"){
      g_carstate = enBACK;
    }
    if(inputString == "L"){
      g_carstate = enLEFT;
    }
    if(inputString == "R"){
      g_carstate = enRIGHT;
    }
    if(inputString == "S"){
      g_carstate = enSTOP;
    }
    if(inputString == "E"){
      g_carstate = enTLEFT;
    }
    if(inputString == "I"){
      g_carstate = enTRIGHT;
    }
    inputString ="";
  }
    
a:  switch(g_carstate){
    case enSTOP : front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnOutput = 0; break;
    case enRUN : resetCarState(); front = 250; break;
    case enBACK : resetCarState(); back = -250; break;
    case enLEFT : turnl = 1; break;
    case enRIGHT : turnr = 1; break;
    case enTLEFT : spinl = 1; break;
    case enTRIGHT : spinr = 1; break;
    default : front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnOutput = 0; break;
  }
}

void Code_left() 
{
  count_left ++;
} 
void Code_right() 
{
  count_right ++;
}

float turnSpin(int turnl, int turnr, int spinl, int spinr, float kp_turn, float kd_turn,float gyro_z){
  int spinOnce = 0;
  float rotationRatio;
  float turnSpeed;
  float turnOut;
  int turnmax, turnmin;
  if(turnl == 1 || turnr == 1 || spinl || spinr){
    if(spinOnce == 0){
      spinOnce++;
      if(turnl == 1 || spinl == 1){
        turnOut = -5;
      }
      else if(turnr == 1 || spinr == 1){
        turnOut = +5;
      }else{
        turnOut = 0;
      }
    }
  }else{
    turnOut = 0;
  }
  float turnOutput = kp_turn*turnOut;
  return turnOutput;
}

double speedpiout(float kp_speed, float ki_speed, float kd_speed, int front, int back, double p0)
{
  float speeds = (pulseleft + pulseright) * 1.0;
  pulseright = pulseleft = 0;
  float speeds_filter = speeds_filterold*0.7 + speeds * 0.3;
  speeds_filterold = speeds_filter;
  positions += speeds_filter;
  positions += front;
  positions += back;
  positions = constrain(positions, -3550,3550); 
  double output = ki_speed * (p0 - positions) + kp_speed * (p0 - speeds_filter);
  return output;
}

void BalanceCar(double speedoutput, float turnOutput, float angle_pitch_output, int front, int back, int IN1M, int IN2M, int IN3M, int IN4M, int PWMA, int PWMB)
{
  pwm1 = -angleoutput - speedoutput + turnOutput; 
  pwm2 = -angleoutput - speedoutput - turnOutput;
  if(pwm1>0 && pwm2>0){
    pwm1=45+pwm1;
    pwm2=45+pwm2;
  }
  else if(pwm1<0 && pwm2<0){
    pwm1=-45+pwm1;
    pwm2=-45+pwm2;
  }
  else{
    pwm1=0;
    pwm2=0;
  }
  
  if (pwm1 > 255) pwm1 = 255;
  if (pwm1 < -255) pwm1 = -255;
  if (pwm2 > 255) pwm2 = 255;
  if (pwm2 < -255) pwm2 = -255;
 
  if(angle_pitch_output > 45 || angle_pitch_output < -45)
  {
    pwm1 = 0;
    pwm2 = 0;
  }
  
  if (pwm1 >= 0) 
  {
  digitalWrite(IN2M, 0);
  digitalWrite(IN1M, 1);
  analogWrite(PWMA, pwm1);
  } 
  else 
  {
  digitalWrite(IN2M, 1);
  digitalWrite(IN1M, 0);
  analogWrite(PWMA, -pwm1);
  }
 
  if (pwm2 >= 0) 
  {
  digitalWrite(IN4M, 0);
  digitalWrite(IN3M, 1);
  analogWrite(PWMB, pwm2);
  } 
  else
  {
  digitalWrite(IN4M, 1);
  digitalWrite(IN3M, 0);
  analogWrite(PWMB, -pwm2);
  }  
}

void angle_data(){
  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050
  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  gyro_x = gyro_x/65.5;
  gyro_y = gyro_y/65.5;
  gyro_z = gyro_z/65.5;
  //Gyro angle calculations
  angle_pitch += gyro_x * dt;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * dt;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  angle_yaw += gyro_z * dt;                                     //Calculate the traveled yaw angle and add this to the angle_yaw variable
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 1.94;                                              //Accelerometer calibration value for pitch
  angle_roll_acc += 1.67;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

}
void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}
