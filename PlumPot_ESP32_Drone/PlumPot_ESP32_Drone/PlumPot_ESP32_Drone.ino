#include <PID_v1.h>

//---------------------------------PID------------------------------------
//Define and import all the PID settings
double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput; 
int actSpeed[4];
//Define the aggressive and conservative Tuning Parameters<br>
double consKp = 1, consKi = 0.05, consKd = 0.25;//We can play with these values to 
PID pitchPID(&rollInput, &rollOutput, &rollSetpoint, consKp, consKi, consKd, DIRECT);
PID rollPID(&pitchInput, &pitchOutput, &pitchSetpoint, consKp, consKi, consKd, DIRECT);
int targetSpeed[4];


//Gyro Variables
float elapsedTime, time1, timePrev;        //Variables for time control
int gyro_error=0;                         //We use this variable to only calculate once the gyro data error
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;       //Here we store the raw data read 
float Gyro_angle_x, Gyro_angle_y;         //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y; //Here we store the initial gyro data error

//Acc Variables
int acc_error=0;                            //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180/3.141592654;         //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;         //Here we store the raw data read 
float Acc_angle_x, Acc_angle_y;             //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y; //Here we store the initial Acc data error



float Total_angle_x, Total_angle_y;
#include <PID_v1.h>

//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B, roll_error, roll_previous_error;
float roll_pid_p=0;
float roll_pid_i=0;
float roll_pid_d=0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp=0.7;//3.55
double roll_ki=0.006;//0.003
double roll_kd=1.2;//2.05
float roll_desired_angle = 0;     //This is the angle in which we whant the

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp=0.72;//3.55
double pitch_ki=0.006;//0.003
double pitch_kd=1.22;//2.05
float pitch_desired_angle = 0;     //This is the angle in which we whant the




//#include <Blynk.h>


//#include <WiFi.h>



//For the Blynk App


#define BLYNK_PRINT Serial

#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>




// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "pc8Xc-3atxxzpFVnV2ooIIO6eIeyBrEP";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "PlumPot";
char pass[] = "Noyoucannot55";

/////////////////////



//setting PWM properties

//For the ESP32 you have to set up the PWM signal, here we choose the frequency of the PWM singnal and Also the resolution of the PWM ESP32 can be up to 16-bits

const int freq = 20000;
const int ledChannel = 0;
const int ledChannel2 = 1;
const int ledChannel3 = 2;
const int ledChannel4 = 3;
const int resolution = 11;

//These values will come from the blynk App

int THROT = 0;
int YAW = 0;
int ROLL = 0;
int PITCH = 0;
int START = 0;

///////////////////////////


//#include "wifi_websocket.h"
#include "imu.h"

//Here we set up the motors pins, so on the PCB front right is connected to pin 12 ect..
const int Back_Left = 12;
const int Front_Left = 14;
const int Back_Right = 16;
const int Front_Right = 18;
/////////////////////////////////////////////////////////////////////
// current yaw pitch roll values in degrees
float* curr_ypr;
float ypr_cal[3];
// current rotational rate in degrees/second
float gyro_pitch, gyro_roll, gyro_yaw;

int controller_active;


//These will be used to MAP my input from the Blynk App from 0 to 2042 to -10 and 10. Dis basicaly makes my raw input into degrees by using the MAP function
float ref_throttle = 0,
      ref_yaw =0,
      ref_pitch=0,
      ref_roll=0;

//PID constants



void updateMotor() {
  if (controller_active) {
    float u_throttle = ref_throttle;

    // do calculation **ROLL**
  
     rollInput = curr_ypr[1] * -180 / M_PI - ypr_cal[1];
     pitchInput = curr_ypr[2] * 180 / M_PI - ypr_cal[2];
     Serial.print("Roll from Acc: ");
     Serial.print(rollInput);
      Serial.print("  Pitch from Acc: ");
     Serial.println(pitchInput);
    // do calculation **PITCH**
   



/*First calculate the error between the desired angle and 
*the real measured angle*/

roll_error = rollInput - ref_roll;
pitch_error = pitchInput - ref_pitch; 
//     Serial.print("Roll error: ");
//     Serial.print(roll_error);
//      Serial.print(" Pitch error: ");
//     Serial.println(pitch_error);


/*Next the proportional value of the PID is just a proportional constant
*multiplied by the error*/
roll_pid_p = roll_kp*roll_error;
pitch_pid_p = pitch_kp*pitch_error;

/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -2 and 2 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/
if(-3 < roll_error <3)
{
  roll_pid_i = roll_pid_i+(roll_ki*roll_error);  
}
if(-3 < pitch_error <3)
{
  pitch_pid_i = pitch_pid_i+(pitch_ki*pitch_error);  
}

/*The last part is the derivate. The derivate acts upon the speed of the error.
As we know the speed is the amount of error that produced in a certain amount of
time divided by that time. For taht we will use a variable called previous_error.
We substract that value from the actual error and divide all by the elapsed time. 
Finnaly we multiply the result by the derivate constant*/
roll_pid_d = roll_kd*((roll_error - roll_previous_error)/elapsedTime);
pitch_pid_d = pitch_kd*((pitch_error - pitch_previous_error)/elapsedTime);
/*The final PID values is the sum of each of this 3 parts*/
roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
/*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
have a value of 2000us the maximum value taht we could substract is 1000 and when
we have a value of 1000us for the PWM signal, the maximum value that we could add is 1000
to reach the maximum 2000us. But we don't want to act over the entire range so -+400 should be enough*/
if(roll_PID < -400){roll_PID=-400;}
if(roll_PID > 400) {roll_PID=400; }
if(pitch_PID < -400){pitch_PID=-400;}
if(pitch_PID > 400) {pitch_PID=400;}


/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
pwm_R_F  = ref_throttle - roll_PID - pitch_PID;
pwm_R_B  = ref_throttle - roll_PID + pitch_PID;
pwm_L_B  = ref_throttle + roll_PID + pitch_PID;
pwm_L_F  = ref_throttle + roll_PID - pitch_PID;

roll_previous_error = roll_error; //Remember to store the previous error.
pitch_previous_error = pitch_error; //Remember to store the previous error.

//     Serial.print("RF PWM ");
//     Serial.print(pwm_R_F);
//     Serial.print(" RB PWM ");
//     Serial.print(pwm_R_B);
//  
//     Serial.print(" LB PWM ");
//     Serial.print(pwm_L_B);
//     Serial.print(" LF PWM ");
//     Serial.println(pwm_L_F);


ledcWrite(ledChannel3, abs(pwm_R_F));
ledcWrite(ledChannel, abs(pwm_R_B));
ledcWrite(ledChannel2, abs(pwm_L_B));
ledcWrite(ledChannel4, abs(pwm_L_F));

  
 
}
  else {
// Serial.println("Disamred. Shutting off motors..");
ledcWrite(ledChannel, 0);
ledcWrite(ledChannel2, 0);
ledcWrite(ledChannel3, 0);
ledcWrite(ledChannel4, 0);

  }

}

void readJoystick() {
  
   ref_throttle = map(THROT, 0, 1023, 0, 2000);
   ref_yaw =  map(YAW, 0, 1023, -10, 10);// this is what I want my Yaw to be
   ref_pitch =  map(PITCH, 0, 1023, -10, 10);// this is what I want my pitch to be and only move from -10 degrees to 10 degrees
   ref_roll = map(ROLL, 0, 1023, -10, 10);// this is what I want my roll to be
   controller_active = START;
}


///BlunkApp Getting values from app

BLYNK_WRITE(V0)
{
  YAW = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
}
BLYNK_WRITE(V1)
{
  THROT = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
}

BLYNK_WRITE(V2)
{
  PITCH = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
}
//////////////////////////
BLYNK_WRITE(V3)
{
  ROLL = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
}
BLYNK_WRITE(V4)
{
  START = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
}

void setup() {
  Serial.begin(115200);

  pitchInput = 0.0;
  rollInput = 0.0;  
  pitchSetpoint = 0.0;
  rollSetpoint = 0.0;
  //turn the PID on
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-20, 20);
  rollPID.SetOutputLimits(-20, 20);
  //-------------------------------------------------------------------
  for (int i = 0; i < 4; i++) {
    targetSpeed[i] = 0;
  } 
 for (int i = 0; i < 3; i++) {
    ypr_cal[i] = 0.0;
  }
 
  pinMode(Back_Left, OUTPUT);
  pinMode(Back_Right, OUTPUT);
  pinMode(Front_Left, OUTPUT);
  pinMode(Front_Right, OUTPUT);
 
   initIMU();
  //Blynk App set up

  Blynk.begin(auth, ssid, pass);

  //Setting up the PWM for motors
  ledcSetup(ledChannel, freq, resolution);
  ledcSetup(ledChannel2, freq, resolution);
  ledcSetup(ledChannel3, freq, resolution);
  ledcSetup(ledChannel4, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(Back_Right, ledChannel);
  ledcAttachPin(Back_Left, ledChannel2);
  ledcAttachPin(Front_Right, ledChannel3);
  ledcAttachPin(Front_Left, ledChannel4);


  
  /////////////////////////////////////////

 

}


void loop() 
{
     /////////////////////////////I M U/////////////////////////////////////
  timePrev = time1;  // the previous time is stored before the actual time read
  time1 = millis();  // actual time read
  elapsedTime = (time1 - timePrev) / 1000; 
   Blynk.run();
  
  curr_ypr = getYPR();
  getRotation(&gyro_pitch, &gyro_roll, &gyro_yaw);
  readJoystick();
  pitchPID.Compute();
  rollPID.Compute();
 
  
  
  updateMotor();
}
