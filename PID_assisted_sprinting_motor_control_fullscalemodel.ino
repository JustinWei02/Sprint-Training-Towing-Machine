//adapted from curiores github ArduinoTutorials/SpeedControl & 
//teachmemicro Home/Tutorials/Arduino Tutorial/Using Rotary Encoders with Arduino

//initialize constants
#include <util/atomic.h>
//#include <LiquidCrystal.h>
#include <Encoder.h>
#include <Servo.h>
#define PI 3.1415926535897932384626433832795
#define E 2.71828

//LCD stuff
//const int rs=12, en=11, d4=10, d5=9, d6=5, d7=4;
//LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
//bool lcd_cleared=false; //counter for number of times the lcd was cleared



const uint8_t VescOutputPin = 5; //VESC output pin 
const int encoderpinOutA = 2; //OutA on encoder to pin 2 on arduino
const int encoderpinOutB = 3; //OutB on encoder to pin 3 on arduino

Encoder sensor1(encoderpinOutA, encoderpinOutB);
Encoder sensor2(encoderpinOutA, encoderpinOutB);


const int encoderPPR=4*2500; //encoder has 600 pulses per revolution
int interval = 3; //time interval for measurement in ms
float shaftRadius = 0.05845; //shaft radius in meters

volatile long encoder_pulses = 0; //counts encoder pulses

//initialize counter variables for previous and current time in milliseconds
long previousTime = 0; 
long currentTime = 0;

Servo esc;

float velocity = 0; //tangential velocity in m/s 
float target_max_velocity = 8; //target max velocity the user wants to reach (PROCEED WITH CAUTION: DO NOT GO FASTER THAN 11.5 M/S)
float rpm = 0; //angular velocity in revolutions per second
float target_rpm = 0; //target angular velocity in revolutions per second
float accelerationDuration=2; //time to reach top speed in seconds (adjust this time)
float max_velocity_duration=4; //duration of max velocity in seconds
float length=0; //length of the string extended from the spool in meters
int percent_error=0; //calculation for the percent error between the target rpm and actual rpm
float line_length=65; //active line length in meters

//Constants for PI Control
//Updated 5/1/23 after adding the chain on sprockets for power transmission
float kp = 0.0000000015; //proportion constant 
float ki = 0.0000005; //integral constant 
float kd = .000004; //derivative constant 

float curr_error = 0; // initialize current error variable 
float prev_error=0; //initialize previous error variable
float u = 0; //initialize control signal u
float error_derivative = 0;//initialize variable for the derivative of the error
volatile long pulses = 0; //initialize a second pulse counter from the encoder
float error_integral=0; //integral error calculation 



float setAcceleration(float maxV, float accelDuration){
  float accelFactor = maxV/(0.27+5.9*(accelDuration)-1.45*pow(accelDuration,2)+0.126*pow(accelDuration,3)); //scaling factor for a preset velocity(time) equation that will adjust for the inputed target max velocity
  float accelVelocity = accelFactor*(0.27+5.9*(currentTime/1.0e3-4)-1.45*pow(currentTime/1.0e3-4,2)+0.126*pow(currentTime/1.0e3-4,3)); //Calculates the velocity necessary throughout the duration of the acceleration
  float accel_target_rpm = 60*accelVelocity/(2*PI*shaftRadius); //convert accelVelocity above into a target rpm
  return accel_target_rpm;
}
float setDecceleration(float maxV, float accelDuration, float maxVDuration){
  float deccelVelocity=maxV+(-maxV)/(1+500*pow(E,-7*((currentTime/1.0e3-4)-(accelDuration+maxVDuration))));
  float deccel_target_rpm = 60*deccelVelocity/(2*PI*shaftRadius); //convert accelVelocity above into a target rpm
  return deccel_target_rpm;
}

void setup() {
  //Set I/O
  esc.attach(VescOutputPin);
  //set LCD
  //lcd.begin(16, 2); 
  
  //arm VESC
  esc.writeMicroseconds(2000);
  delay(2000);
  esc.writeMicroseconds(1000);
  delay(2000);
    
}


void loop() {
  //prints number of pulses for every interval of time passed
  currentTime = millis();
  if (currentTime - previousTime >= interval){
    previousTime = currentTime; //save the last time 
    rpm=(float)(abs(sensor1.read())*(6e4/interval)/encoderPPR); //rpm
    sensor1.write(0); 
  }
  //set a target speed 
    if (currentTime>=4000){
      if ((currentTime/1.0e3)-4 <= accelerationDuration){ //ramps the user to the target max velocity 
    	    target_rpm = setAcceleration(target_max_velocity, accelerationDuration);
          //Serial.println("in case 1");
  	    }
  	    else if ((accelerationDuration <(currentTime/1.0e3)-4) && ((currentTime/1.0e3)-4<accelerationDuration+max_velocity_duration)){
    	  target_rpm = 60*target_max_velocity/(2*PI*shaftRadius); 
        //Serial.println("in case 2");
  	    }
  
  	    else if (((currentTime/1.0e3)-4>accelerationDuration+max_velocity_duration) && ((currentTime/1.0e3)-4<accelerationDuration+max_velocity_duration+2)){
    	    target_rpm= setDecceleration(target_max_velocity, accelerationDuration, max_velocity_duration);
          //setMotor(0, 0, motorpinENA, motorpinIn1, motorpinIn2);   
          //Serial.println("in case 3");
 	      }
        else {
          esc.writeMicroseconds(1000);  
          //Serial.println("in case 4");    
        }  
   }
  
  //if (currentTime/1.0e3 < accelerationDuration+max_velocity_duration){
    //computing the control signal u
  curr_error= target_rpm-rpm; //error between target velocity and velocity
  error_integral = error_integral+curr_error*(currentTime-previousTime); //calculates the integral of the error
  error_derivative = (curr_error-prev_error)/(currentTime-previousTime); //calculates the derivative of the error
  u =kp*curr_error+ki*error_integral+kd*error_derivative; //control signal u
  prev_error = curr_error; 
    
  
 
  int pwr = (int) fabs(u); //computes the PI control signal u and sets it to the pwm value for the motor speed
  if (pwr > 1000){
    pwr = 1000; 
  }

  //Turns off the motor completely after the repetition is complete
  if (currentTime>=4000){
    if (currentTime/1.0e3-4<=accelerationDuration+max_velocity_duration+2.5) { 
      if ((sensor2.read()/encoderPPR)*2*PI*shaftRadius<line_length){ //only sets control signl for the motor if there is enough line spooled on
          esc.writeMicroseconds(map(pwr, 0, 1000, 1000, 2000)); //sets motor configuration
      }
    }
    else{
      target_rpm=0;
    }
  }
  //Velocity and Error Calculations
  velocity=rpm*(2*PI/60)*shaftRadius;   
  percent_error=100*(target_rpm-rpm)/target_rpm;

  //ENCODER DETACHMENT FAIL SAFE (IMPORTANT, TEST THIS IMMEDIATELY)
  //(If encoder detaches due to vibration, which there is small chance it can happen at close to top speed, motor will turn off to prevent a spike in control effort)
  if (currentTime>=4000){
    if ((0.8*accelerationDuration <(currentTime/1.0e3)-4) && ((currentTime/1.0e3)-4<accelerationDuration+max_velocity_duration+2.5) && (percent_error>60)){ 
      esc.writeMicroseconds(map(pwr, 0, 1000, 1000, 2000)); //sets motor configuration
    }
  }

  while ((currentTime/1.0e3-4<=accelerationDuration+max_velocity_duration+2.5) && (currentTime>=3900)){
    Serial.print(currentTime/1.0e3-4);
    Serial.print(" , ");
    Serial.print(velocity); 
    Serial.println();
  }
  
}