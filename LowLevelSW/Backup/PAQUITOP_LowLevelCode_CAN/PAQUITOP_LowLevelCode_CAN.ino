// PAQUITOP: Low Level Code

//////////////////
//  LIBRARIES   //
//////////////////

#include <math.h> 
#include "SPI.h"
#include "SBUS.h"
#include "Sensors.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

/////////////////
//  CONSTANTS  //
/////////////////

#define pi 3.14159265359
#define g 9.80665a
#define a 0.25
#define b 0.125
#define rw 0.05
#define rc 0.04

/////////////
//  PINS   //
/////////////

// Stepper Motor Driver (ST)
#define ST_ENB 17
#define ST_MS1 16 
#define ST_MS2 15 

#define ST_STEP_1 36 
#define ST_DIR_1  35
#define ST_STEP_2 14 
#define ST_DIR_2  13 

// Encoder (ENC)
#define ENC_SCK  27
#define ENC_MISO 39
#define ENC_MOSI 26
#define ENC_CS_1 38
#define ENC_CS_2 37

// Motor Wheel Driver (MW)
#define MW_ENB   32

#define MW_PWM_1 29
#define MW_DIR_1 31
#define MW_FB_i1 24 
#define MW_FB_w1 25 

#define MW_PWM_2 33
#define MW_DIR_2 34
#define MW_FB_i2 41 
#define MW_FB_w2 40 

///////////////
// VARIABLES //
///////////////

void alarmCallback(uint8_t, uint8_t, uint16_t);



Sensors proxSensors({.yellowThreshold = 110, .redThreshold = 50, .laserThreshold = 15, .alarmTimeout=10}, alarmCallback);  //Sensors.h class element, thresholds and callback function definition
//Sensors proxSensors({.yellowThreshold = 0, .redThreshold = 00, .laserThreshold = 000, .alarmTimeout=10}, alarmCallback);  //Sensors.h class element, thresholds and callback function definition

SBUS joystick(Serial1);

unsigned long t, t_old, t0, t01, t02, t03, dt, dt1, dt2, dt3;                             //time variables (us)
unsigned long tHidle, tHidle_max = 1e6;                                                   //if zero inputs are sent for t = tHidle, the motors are temporarly disabled (us)
unsigned long tSens;

float vx, vy, gammad;                                                                     //linear velocities (m/s) and angular velocity (rad/s) of the platform in body rf {b}
float scale_vx = 1.0, scale_vy = 1.0, scale_gammad = 1.0;
float v_max = 1.0, w_max = pi/2;                                                          //maximum linear and angular velocity of the platform
int   wConf = 1, wConf_old = 1;                                                           //1: omnidirectional motion, 2: differential drive, 3: bicycle 2 steering wheels
float vxArm = 0.0, vyArm = 0.0, vzArm = 0.0, wxArm = 0.0, wyArm = 0.0, wzArm = 0.0;       //linear velocities (m/s) and angular velocity (rad/s) of the EE in rf {0}
float vArm_max = 0.15, wArm_max = pi/6;                                                   //maximum linear and angular velocity of the EE
float tau = 0.3;                                                                          //transmission ratio delta_st/delta_m = z_m/z_st = 18/60;
int   uStep = 2;                                                                          //uStepping ratio (MS1 = 1; MS2 = 0)
int   Nstep = 200*uStep/tau;                                                              //number of steps/rev of the steering joint

// Gain controllo PID:
float kp = 20.0*0.0056753;                                                                //2 without load, 20 with load
float ki = 0.0005;                                                                        //0.00005 without load, 0.0005 with load
float kd = 0.0;                                                                           //0.0 without load, 0.0 with load
float INT, DER;

float delta, delta1 = 0.0, delta2 = 0.0, delta1_REF, delta2_REF;                          //steering angles (rad);
float dr1, dr2, dr_old, dl_old1, dl1, dl2, dl_old;
int   step1, step2, step1_REF, step2_REF;                                                 //steering angles (step);
int   e_step1, e_step2, step1_OLD = 0, step2_OLD = 0;
int   dn = 10;
float k_eStep = 1.0, error_rad;

float thd1, thd2, thd1_REF, thd2_REF ;
float thd1_OLD = 0.0, thd2_OLD = 0.0;
float thdr1, thdr2, thdl1, thdl2;
float i1, i2, i1_REF, i2_REF;
float i1_OLD = 0.0, i2_OLD = 0.0;
float MW_wMax = 400.0, MW_iMax = 7.5;

float    Rx, Ry, Lx;
float    RxvArm, RyvArm, LxvArm, RxwArm, RywArm, LxwArm, LygArm;                          //variabili di lettura degli analogici
uint16_t channels[16];
bool     failSafe, lostFrame;
bool     tabletON = false;

// ROS Variables
bool enb_ROSsub = false;
char str_fb_q[20] = {' '};

void cmd_vel_cb(const geometry_msgs::Twist& cmdVel){
  if (enb_ROSsub){
    vx = cmdVel.linear.y;
    vy = -cmdVel.linear.x;
    gammad = cmdVel.angular.z;}
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);

// Proximity Sensors:
float psx[] = {-0.10391, -0.11563, -0.11563, -0.10391, 0.10391, 0.11563, 0.11563, 0.10391};
float psy[] = {-0.277, -0.21321, 0.21321, 0.277, 0.277, 0.21321, -0.21321, -0.277};
float nsx[] = {0.5661, -0.964, -0.964, -0.5661, 0.5661, 0.964, 0.964, 0.5661};
float nsy[] = {-0.8243, -0.2659, 0.2659, 0.8243, 0.8243, 0.2659, -0.2659, -0.8243};
float po[2];


void setup() {
  
  pinInit();

  commInit();pro

  motorsInit();

  motorsENB();

  t0    = micros();
  t_old = micros();
  t01   = micros();
  t02   = micros();
  t03   = micros();
  tSens = micros();
}

void loop() {

  t   = micros() - t0;
  dt1 = micros() - t01;
  dt2 = micros() - t02; 
  dt3 = micros() - t03;  
  
  if (dt1 > 2e+4){  
    joystickRead(); 
    ikineVel(wConf, scale_vx*vx, scale_vy*vy, scale_gammad*gammad);

    t01 = micros(); 
  }

  step1_REF = delta1_REF*Nstep/2/pi;
  step2_REF = delta2_REF*Nstep/2/pi;

  contrStep1(step1_REF);
  contrStep2(step2_REF);

  error_rad = (abs(delta1_REF-delta1)+abs(delta2_REF-delta2))*Nstep/(pi);
  if (error_rad <=1000 || error_rad>=10500){k_eStep = 1.0;}
  else {k_eStep = 0.10;}

  thd1 = -map(analogRead(MW_FB_w1),0,1023,-MW_wMax,MW_wMax);   //(rmp)  
  thd2 =  map(analogRead(MW_FB_w2),0,1023,-MW_wMax,MW_wMax);   //(rmp)
  i1   =  mapF(analogRead(MW_FB_i1),0,1023,0,MW_iMax);          //(A)
  i2   =  mapF(analogRead(MW_FB_i2),0,1023,0,MW_iMax);          //(A)

  dt = micros() - t_old;
 
  i1_REF = PID_ctrl(dt, i1_OLD, k_eStep*thd1_REF*30/pi, thd1, thd1_OLD, kp, kd, ki, -MW_wMax, MW_wMax);
  i2_REF = PID_ctrl(dt, i2_OLD, k_eStep*thd2_REF*30/pi, thd2, thd2_OLD, kp, kd, ki, -MW_wMax, MW_wMax);

  if (i1_REF > MW_wMax){i1_REF = MW_wMax;}
  else if (i1_REF < -MW_wMax){i1_REF = -MW_wMax;}
  
  if (i2_REF > MW_wMax){i2_REF = MW_wMax;}
  else if (i2_REF < -MW_wMax){i2_REF = -MW_wMax;}

  if (i1_REF >= 0){digitalWrite(MW_DIR_1, LOW); analogWrite(MW_PWM_1, mapF(abs(i1_REF),0,MW_iMax,255*0.1,255*0.9));} 
  else {digitalWrite(MW_DIR_1, HIGH); analogWrite(MW_PWM_1,mapF(abs(i1_REF),0,MW_iMax,255*0.1,255*0.9));}

  if (i2_REF >= 0){digitalWrite(MW_DIR_2, HIGH); analogWrite(MW_PWM_2, mapF(abs(i2_REF),0,MW_iMax,255*0.11,255*0.89));} 
  else {digitalWrite(MW_DIR_2, LOW); analogWrite(MW_PWM_2, mapF(abs(i2_REF),0,MW_iMax,255*0.11,255*0.89));}
  t_old = micros();
  
  thd1_OLD = thd1; 
  thd2_OLD = thd2;
  i2_OLD = i2_REF;
  i1_OLD = i1_REF;
  
  if (dt2 > 1e+5){
    updateStepPos();   
    
    t02 = micros(); 
  } 
  nh.spinOnce();

  proxSensors.update();

/*dist_t distanza = proxSensors.requestDistance(sensor3);
  if(distanza.error) {
    Serial.println("Non è stato possibile rilevare la distanza dal sensore 1.");
  } else {
    Serial.print("Distanze attualmente rilevate dal sensore 1: laser: ");
    Serial.print(distanza.distLaser);
    Serial.print(", sonar: ");
    Serial.println(distanza.distSonar);
  }*/
  if (micros() - tSens >= 1e6){
    scale_vx = 1.0;
    scale_vy = 1.0;
    scale_gammad = 1.0;
  }
}
