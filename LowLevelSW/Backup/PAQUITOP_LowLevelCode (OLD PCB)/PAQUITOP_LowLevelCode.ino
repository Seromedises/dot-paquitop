/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                        FIRMWARE ARDUINO PAQUITOP                                                                                      /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////
// DEFINIZIONE LIBRERIE  //
///////////////////////////

#include "SPI.h"
#include <math.h> 
#include "SBUS.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

///////////////////////////
// DEFINIZIONE COSTANTI  //
///////////////////////////

#define pi 3.14159265359
#define g 9.80665
#define a 0.25
#define b 0.125
#define rw 0.05
#define rc 0.04

///////////////////////////
// ASSEGNAZIONE PORTE //
///////////////////////////

// SPI1
#define SPI1_MOSI_PIN 26
#define SPI1_MISO_PIN 39
#define SPI1_SCK_PIN 27

// Stepper Motor Driver (SMD)
#define SMD_R_STEP_PIN 36 
#define SMD_R_DIR_PIN 12
#define SMD_R_SLEEP_PIN 34

#define SMD_L_STEP_PIN 11 
#define SMD_L_DIR_PIN 10
#define SMD_L_SLEEP_PIN 34

#define SMD_MS1 35 
#define SMD_MS2 9 

// ECframeless ESCON Driver (ECD)
#define ECD_R_DIR_PIN 32 	//PIN 20 ESCON 
#define ECD_R_ENB_PIN 31 	//PIN 21 ESCON 
#define ECD_R_PWM_PIN 29 	//PIN 22 ESCON 
#define ECD_R_VRM_PIN 25 	//PIN 24 ESCON (Velocità reale media)
#define ECD_R_IR_PIN 24 	//PIN 25 ESCON (Corrente reale media)

#define ECD_L_DIR_PIN 30 	//PIN 20 ESCON 
#define ECD_L_ENB_PIN 28 	//PIN 21 ESCON 
#define ECD_L_PWM_PIN 33 	//PIN 22 ESCON 
#define ECD_L_VRM_PIN 40 	//PIN 24 ESCON (Velocità reale media)
#define ECD_L_IL_PIN 41 	//PIN 25 ESCON (Corrente reale media)

// Encoder (ENC)
#define ENC_R_CS1_PIN 38 	// SU SPI1 
#define ENC_L_CS1_PIN 37 	// SU SPI1 


///////////////////////////
// DEFINIZIONE VARIABILI //
///////////////////////////

SBUS padPAQ(Serial1);

// Variabili di tempo:
unsigned long t, t0, t_old, t_old1, t_old2, t_old3, t_old4, t_new, dt, dt1, dt2, dt3, dt4, tHidle;//variabili di tempo globali (microsecondi)

// Variabili velocità spazio operativo:
float vx, vy, gammad, vx_s, vy_s, gammad_s;;                                              //velocità lineari (m/s) e angolare (rad/s) piattaforma rf {c}
float v_max = 1.0, w_max = pi/2;                                                          //velocità massime piattaforma (m/s), (rad/s)
float odom_vx, odom_vy, odom_wz;
int modOp = 1, modOp_old = 1;                                                             //mod 1: moto generico; mod 2: sterzatura differenziale; mod 3: bici 1 ruota; mod 4: bici 2 ruote
float vxArm = 0.0, vyArm = 0.0, vzArm = 0.0, wxArm = 0.0, wyArm = 0.0, wzArm = 0.0;       //velocità lineari (m/s) e angolare (rad/s) braccio robotico rf {??}
float vArm_max = 0.15, wArm_max = pi/6;                                                   //velocità massime braccio robotico (m/s), (rad/s)

// Variabili motori trazione:
float thetadr_REF, thetadl_REF;                                                           //velocità ruote REF (rpm)
float thetadr, thetadl;                                                                   //velocità ruote FB (rpm)
float thetadr_tol = 10.0, thetadl_tol = 10.0;
float thetadr_old, thetadl_old;                                                           //velocità ruote FB_old (rpm)
float thdr1, thdr2, thdl1, thdl2;
float Ir_REF,Il_REF, Itot_REF;                                                                      //correnti motori trazione REF (A)
float Ir_REF_old = 0.0,Il_REF_old = 0.0;                                                  //correnti motori trazione REF_old (A)
float Ir,Il;                                                                              //correnti motori trazione FB (A)
float ecd_r_vmax = 400.0, ecd_l_vmax = 400.0;                                             //velocità massima impostata al driver (rpm)
float ecd_r_imax = 15.0, ecd_l_imax = 15.0;                                               //corrente massima impostata al driver (A)

// Variabili motori sterzo:
float deltar_REF = pi/2, deltal_REF = pi/2, delta;                                                      //variabili posizione sterzo  (rad )
float dr1, dr2, dr_old, dl_old1, dl1, dl2, dl_old;
float deltar_REF_old, deltal_REF_old, countSR = 0, countSL = 0;
float deltar = pi/2, deltal = pi/2;                                                             //variabili posizione sterzo e velocità ruote (rad e rad/s)
float err_deltar, err_deltal, k_eStep;                                                             //errori di posizione angolo di sterzo (rad)
float tau = 0.3;                                                                          //rapporto di trasmizzione delta_sterzo/delta_motore = z_motore/z_sterzo = 18/60;
int mStepping = 2;                                                                        //valore microstepping (vedi enb_SMD parametro MODE)
int Nstep = 200*mStepping/tau;                                                  //numero di step in un giro 
int dnR, dnL, dNsoglia = 100, nstepR_REF, nstepL_REF, nstepR_old, nstepL_old, e_step;            //variabili di riferimento sterzo in numero di step (per OL)

// Gain controllo PID:
float kp = 2.0*0.0056753;                                                              //2 senza carico, 20 con carico
float ki = 0.00005;                                                                      //0.00005 senza carico, 0.0005 con carico
float kd = 0.0;
float INT, DER;

// Variabile di debug:
bool enb_SERIALPRINT = false;

// Variabili joystick:
float Rx, Ry, Lx, RxvArm, RyvArm, LxvArm, RxwArm, RywArm, LxwArm, LygArm;									//variabili di lettura degli analogici
uint16_t channels[16];
bool failSafe;
bool lostFrame;
bool tabletON = false;

// Variabili ROS:
bool enb_ROSsub = false;
char str_fb_q[20] = {' '};

ros::NodeHandle  nh;

// ROS subcriber:
void cmd_vel_cb(const geometry_msgs::Twist& cmdVel){
  if (enb_ROSsub){
    //vx = cmdVel.linear.x;
    //vy = cmdVel.linear.y;
    vx = cmdVel.linear.y;
    vy = -cmdVel.linear.x;
    gammad = cmdVel.angular.z;
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);

// ROS publisher:
std_msgs::String str_msg;
ros::Publisher fb_q("fb_q", &str_msg);

geometry_msgs::Twist twistArm;
ros::Publisher velTwistArm("velTwistArm", &twistArm);

std_msgs::Bool restArm;
ros::Publisher arm_rest_pose("arm_rest_pose", &restArm);

std_msgs::Bool tabletDeliver;
ros::Publisher extract_tablet("extract_tablet", &tabletDeliver);

std_msgs::Bool tabletStore;
ros::Publisher retrain_tablet("retrain_tablet", &tabletStore);

std_msgs::Float32 mvGripper;
ros::Publisher moveGripper("moveGripper", &mvGripper);

std_msgs::Bool stopArm;
ros::Publisher stop_arm("stop_arm", &stopArm);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                        INIZIALIZZAZIONE		                                                                                            /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{  
  // Inizializzazione porte
  pinMode(SMD_R_STEP_PIN, OUTPUT); 
  pinMode(SMD_R_DIR_PIN, OUTPUT); 
  pinMode(SMD_R_SLEEP_PIN, OUTPUT); 
  
  pinMode(SMD_L_STEP_PIN, OUTPUT);
  pinMode(SMD_L_DIR_PIN, OUTPUT);
  pinMode(SMD_L_SLEEP_PIN, OUTPUT);  

  pinMode(SMD_MS1,OUTPUT);
  pinMode(SMD_MS2,OUTPUT);

  pinMode(ECD_R_DIR_PIN, OUTPUT);
  pinMode(ECD_R_ENB_PIN, OUTPUT);
  pinMode(ECD_R_PWM_PIN, OUTPUT);
  pinMode(ECD_R_VRM_PIN, INPUT);
  pinMode(ECD_R_IR_PIN, INPUT);

  pinMode(ECD_L_DIR_PIN, OUTPUT);
  pinMode(ECD_L_ENB_PIN, OUTPUT);
  pinMode(ECD_L_PWM_PIN, OUTPUT); 
  pinMode(ECD_L_VRM_PIN, INPUT);
  pinMode(ECD_L_IL_PIN, INPUT);

  pinMode(ENC_R_CS1_PIN, OUTPUT); 
  pinMode(ENC_L_CS1_PIN, OUTPUT);
  
  // Attivo la Seriale
  Serial.begin(115200);
  if (enb_SERIALPRINT){
    while (!Serial) {;}                                                                       //aspetto che la comunicazione seriale sia stabilita
  }
  delay(1000);
  if (enb_SERIALPRINT){
    Serial.println("PAQUITOP FIRMWARE inizializzazione:"); Serial.println("Comunicazione Seriale STABILITA;");
    }
    
  // Imposto e attivo il BUS SPI1
  SPI1.setMOSI(SPI1_MOSI_PIN);
  SPI1.setMISO(SPI1_MISO_PIN);
  SPI1.setSCK(SPI1_SCK_PIN);

  SPI1.begin(); 
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));                         // v. di comunicazione 1MHz (max ??), MSBFIRST, MODE 1
  if (enb_SERIALPRINT){
    Serial.println("Comunicazione SPI1 STABILITA: f = 1MHz; MODE1; PIN CUSTOM;");
    }
  delay(500); 

  // Inizializzo il Joystick
  if (enb_SERIALPRINT){Serial.print("Inizializzazione Gamepad \t...\t");}
  Serial1.begin(100000, 8E2); // Sbus
  delay(500);
  padPAQ.begin();
  if (enb_SERIALPRINT){Serial.println("Gamepad inizializzato;");}
  delay(100);
  

  // ROS Setup:
  if (!enb_SERIALPRINT){
    nh.initNode();
    nh.advertise(fb_q);
    nh.advertise(velTwistArm);
    nh.advertise(arm_rest_pose);
    nh.advertise(extract_tablet);
    nh.advertise(retrain_tablet);
    nh.advertise(stop_arm);
    nh.advertise(moveGripper);
    nh.subscribe(sub);
    delay(100);
  }
  stopArm.data = false;
  
  // Attivo SMD
  digitalWrite(SMD_R_SLEEP_PIN, HIGH);
  digitalWrite(SMD_L_SLEEP_PIN, HIGH);
  if (enb_SERIALPRINT){Serial.println("Driver STEPPER MOTORS ACCESI;");}

  // Abilito/Disabilito SMD (LOW abilita, HIGH disabilita)
  digitalWrite(SMD_R_SLEEP_PIN, LOW);
  digitalWrite(SMD_L_SLEEP_PIN, LOW);
  digitalWrite(SMD_MS1,HIGH);
  digitalWrite(SMD_MS2,LOW);
  if (enb_SERIALPRINT){Serial.println("Driver STEPPER MOTORS ABILITATI tramite SPI;");}
  
  //Abilito/Disabilito ECD
  analogWriteFrequency(ECD_R_PWM_PIN, 536);
  digitalWrite(ECD_R_ENB_PIN, LOW);
  analogWrite(ECD_R_PWM_PIN, 255*0.10);
  digitalWrite(ECD_R_DIR_PIN, HIGH);
  digitalWrite(ECD_R_ENB_PIN, HIGH);

  //Abilito/Disabilito ECD
  analogWriteFrequency(ECD_L_PWM_PIN, 536);
  digitalWrite(ECD_L_ENB_PIN, LOW);
  analogWrite(ECD_L_PWM_PIN, 255*0.10);
  digitalWrite(ECD_L_DIR_PIN, HIGH);
  digitalWrite(ECD_L_ENB_PIN, HIGH);
  
  // Disabilito i motori
  disb_Motors();

  if (enb_SERIALPRINT){Serial.println("Driver MAXON MOTORS ABILITATI: frequenza PWM settata a 536Hz;");}
  delay(100);

  //Per evitare movimenti all'avvio dovuti a errori nella lettura dei feedback
  thetadr = map(analogRead(ECD_R_VRM_PIN),0,1023,-ecd_r_vmax,ecd_r_vmax); 
  thetadl = map(analogRead(ECD_L_VRM_PIN),0,1023,-ecd_l_vmax,ecd_l_vmax);
  delay(100);
  
  if (enb_SERIALPRINT){Serial.println("MOTORI in Standby in attesa di attivazione");

  Serial.println("PAQUITOP FIRMWARE inizializzazione COMPLETATA."); Serial.println("");}
  
  //Salvo l'istante iniziale e quello attuale
  t0 = micros();
  t_old = micros();
  t_old1 = micros();
  t_old2 = micros();
  t_old3 = micros();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                        MAIN LOOP	  			                                                                                            /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{ 
  t = micros() - t0;
  dt = micros() - t_old;
  dt1 = micros() - t_old1;
  dt2 = micros() - t_old2;
  dt3 = micros() - t_old3;
  dt4 = micros() - t_old4;

  //////////////////////////////////////////////////////////////////
  // Ciclo a 50 Hz ////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////

  if (dt > 20000)
  { 
    
    // Se disponibile leggo il Radio Controller
    if(padPAQ.read(&channels[0], &failSafe, &lostFrame)){
    
    // Leggo i valori di INPUT e Mappa Joystick con map_51:
    if (channels[3]<= 992){Lx = map_51(mapF(channels[3],172,992,w_max,0), w_max, w_max); LxvArm = map_51(mapF(channels[3],172,992,vArm_max,0), vArm_max, vArm_max); LxwArm = map_51(mapF(channels[3],172,992,wArm_max,0), wArm_max, wArm_max);}
    else {Lx = map_51(mapF(channels[3],993,1811,0,-w_max), w_max, w_max); LxvArm = map_51(mapF(channels[3],993,1811,0,-vArm_max), vArm_max, vArm_max); LxwArm = map_51(mapF(channels[3],993,1811,0,-wArm_max), wArm_max, wArm_max);}

    if (channels[1]<= 990){Rx = map_51(mapF(channels[1],172,990,-v_max,0), v_max, v_max); RxvArm = map_51(mapF(channels[1],172,990,-vArm_max,0), vArm_max, vArm_max); RxwArm = map_51(mapF(channels[1],172,990,-wArm_max,0), wArm_max, wArm_max);}
    else {Rx = map_51(mapF(channels[1],991,1811,0,v_max), v_max, v_max); RxvArm = map_51(mapF(channels[1],991,1811,0,vArm_max), vArm_max, vArm_max); RxwArm = map_51(mapF(channels[1],991,1811,0,wArm_max), wArm_max, wArm_max);}

    if (channels[2]<= 997){Ry = map_51(mapF(channels[2],172,997,-v_max,0), v_max, v_max); RyvArm = map_51(mapF(channels[2],172,997,-vArm_max,0), vArm_max, vArm_max); RywArm = map_51(mapF(channels[2],172,997,-wArm_max,0), wArm_max, wArm_max);}
    else {Ry = map_51(mapF(channels[2],998,1811,0,v_max), v_max, v_max); RyvArm = map_51(mapF(channels[2],998,1811,0,vArm_max), vArm_max, vArm_max); RywArm = map_51(mapF(channels[2],998,1811,0,wArm_max), wArm_max, wArm_max);}

    if (channels[5] == 1811){
      
      if (channels[6] == 172){
        enb_ROSsub = true;
        modOp = 1;
      }
      else if (channels[6] == 992){
        enb_ROSsub = false;
        vx = 0.0;
        vy = 0.0;
        gammad = 0.0;

        if (tabletDeliver.data){
          tabletDeliver.data = false;
          extract_tablet.publish( &tabletDeliver );
          nh.spinOnce();
          
          tabletStore.data = true;
          retrain_tablet.publish( &tabletStore );
          nh.spinOnce();
          
          tabletON = false;
        }
      }
      else if (channels[6] == 1811){
        enb_ROSsub = false;
        vx = 0.0;
        vy = 0.0;
        gammad = 0.0;

        if (!tabletDeliver.data){
          tabletDeliver.data = true;
          extract_tablet.publish( &tabletDeliver );
          nh.spinOnce();
          
          tabletStore.data = false;
          retrain_tablet.publish( &tabletStore );
          nh.spinOnce();

          tabletON = true;
        }
      }
    }
    else if (channels[5] == 992){
      vx = 0.0;
      vy = 0.0;
      gammad = 0.0;
      enb_ROSsub = false;
      
      if (channels[6] == 172){
        
        vxArm = RyvArm;
        vyArm = -RxvArm;
        vzArm = -LxvArm;
        wxArm = 0.0;
        wyArm = 0.0;
        wzArm = 0.0;
        
        twistArm.linear.x = vxArm;
        twistArm.linear.y = vyArm;
        twistArm.linear.z = vzArm;
        twistArm.angular.x = wxArm;
        twistArm.angular.y = wyArm;
        twistArm.angular.z = wzArm;
        
        if (!enb_SERIALPRINT){
          velTwistArm.publish( &twistArm );
          nh.spinOnce();

          mvGripper.data = mapF(channels[0],172,1811,0.0,1.0);
          
          moveGripper.publish( &mvGripper );
          nh.spinOnce();
        }
      }
      else if (channels[6] == 992){
        
        if (restArm.data && !enb_SERIALPRINT){
        restArm.data = false;
        arm_rest_pose.publish( &restArm );
        nh.spinOnce();
      }
        vxArm = 0.0;
        vyArm = 0.0;
        vzArm = 0.0;
        wxArm = -RxwArm;
        wyArm = RywArm;
        wzArm = -LxwArm;
        
        twistArm.linear.x = vxArm;
        twistArm.linear.y = vyArm;
        twistArm.linear.z = vzArm;
        twistArm.angular.x = wxArm;
        twistArm.angular.y = wyArm;
        twistArm.angular.z = wzArm;
        
        if (!enb_SERIALPRINT){
          velTwistArm.publish( &twistArm );
          nh.spinOnce();

          mvGripper.data = mapF(channels[0],172,1811,0.0,1.0);

          moveGripper.publish( &mvGripper );
          nh.spinOnce();
        }
      }
      else if ((!restArm.data) &&(channels[6] == 1811) && !enb_SERIALPRINT && !tabletON){
        restArm.data = true;
        arm_rest_pose.publish( &restArm );
        nh.spinOnce();
      }
    }
    else{		
      enb_ROSsub = false;
  		if (channels[6] == 172){
      	modOp = 2;
  			vx = Ry;
        vy = 0.0;
  			gammad = -Rx;
  		}
  		else if (channels[6] == 992){
  		  modOp = 1;
  			vx = -Rx;
        vy = -Ry;
  			gammad = Lx;
  		}
  		else if ((channels[6] == 1811)){
  		  modOp = 5;
  			vx = 0.0;
  			vy = -Ry;
  			gammad = Rx;
  		}
    }

    if ((abs(vx) <= 0.05 && abs(vy) <= 0.05 && abs(gammad) <= 0.05) && (abs(dnR) <= 10*dNsoglia && abs(dnL) <= 10*dNsoglia)){tHidle += dt;}
    else {tHidle = 0;}
    
    if ((channels[4] == 1811) || (tHidle >= 1e6))
    { 
      if (!stopArm.data){
        stopArm.data = true;
        stop_arm.publish( &stopArm );
        nh.spinOnce();
      }
        
      disb_Motors();
      vx = 0.0;
      vy = 0.0;
      gammad = 0.0;
    }
    else if (((channels[4] == 172) && (tHidle < 1e6)) || ((channels[4] == 992) && (tHidle < 1e6)))
    {
      if (stopArm.data){
        stopArm.data = false;
        stop_arm.publish( &stopArm );
        nh.spinOnce();
      }        
      enb_Motors();
    }
   
    if (abs(vx) <= v_max && abs(vy) <= v_max && abs(gammad) <= w_max)
    {
      // Calcolo i valori di riferimento in funzione della modalità operativa: 
      ikineVel(modOp, vx, vy, gammad);  
      
      deltal_REF_old = deltal_REF;
      deltar_REF_old = deltar_REF;      
    }
    
    t_old = micros();
    }
  }

  //////////////////////////////////////////////////////////////////
  // Ciclo a 200 kHz ///////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////
  
  if (dt1 > 100)
  {
       
    // Calcolo riferimento sterzo in numero di step:
    nstepR_REF = deltar_REF*Nstep/(2*pi);
    nstepL_REF = deltal_REF*Nstep/(2*pi); 

    e_step = (abs(deltar_REF-deltar)+abs(deltal_REF-deltal))*Nstep/(pi);
    Serial.println(e_step);
    if (e_step <=1000 || e_step>=10500){
      k_eStep = 1.0;
    }
    else {
      k_eStep = 0.10;
    }

    // OpenLoop angoli sterzo:
    OL_StepperR(nstepR_REF);
    OL_StepperL(nstepL_REF);

    // Leggo i feedback motori di trazione:
    thetadr = -map(analogRead(ECD_R_VRM_PIN),0,1023,-ecd_r_vmax,ecd_r_vmax);  
    thetadl = map(analogRead(ECD_L_VRM_PIN),0,1023,-ecd_l_vmax,ecd_l_vmax); 
    Ir = mapF(analogRead(ECD_R_IR_PIN),0,1023,0,ecd_l_imax); //A
    Il = mapF(analogRead(ECD_L_IL_PIN),0,1023,0,ecd_l_imax); //A
    
    // Calcolo il segnale di controllo per i driver:
    Il_REF = PID_ctrl(dt, Il_REF_old, k_eStep*thetadl_REF*30/pi, thetadl, thetadl_old, kp, kd, ki, -ecd_l_imax, ecd_l_imax);
    Ir_REF = PID_ctrl(dt, Ir_REF_old, k_eStep*thetadr_REF*30/pi, thetadr, thetadr_old, kp, kd, ki, -ecd_r_imax, ecd_r_imax);
    Itot_REF = Ir_REF + Il_REF;
    
    
    // Inserisco un semplicissimo controllo di trazione:
    if ((abs(vx)<=0.05) && (abs(thetadr_REF-thetadl_REF)<=1.0)){
      Ir_REF = Itot_REF/2;
      Il_REF = Itot_REF/2;
    }

    // Introduco la saturazione sul riferimento di coppia:
    if (Il_REF > ecd_l_imax){Il_REF = ecd_l_imax;}
    else if (Il_REF < -ecd_l_imax){Il_REF = -ecd_l_imax;}

    if (Ir_REF > ecd_r_imax){Ir_REF = ecd_r_imax;}
    else if (Ir_REF < -ecd_r_imax){Ir_REF = -ecd_r_imax;}

    // Assegno i segnali di riferimento di coppia ai driver:
    if (Il_REF >= 0){digitalWrite(ECD_L_DIR_PIN, HIGH); analogWrite(ECD_L_PWM_PIN, mapF(abs(Il_REF),0,ecd_l_imax,255*0.1,255*0.9));} 
    else {digitalWrite(ECD_L_DIR_PIN, LOW); analogWrite(ECD_L_PWM_PIN, mapF(abs(Il_REF),0,ecd_l_imax,255*0.1,255*0.9));}

    if (Ir_REF >= 0){digitalWrite(ECD_R_DIR_PIN, LOW); analogWrite(ECD_R_PWM_PIN, mapF(abs(Ir_REF),0,ecd_r_imax,255*0.1,255*0.9));} 
    else {digitalWrite(ECD_R_DIR_PIN, HIGH); analogWrite(ECD_R_PWM_PIN,mapF(abs(Ir_REF),0,ecd_r_imax,255*0.1,255*0.9));}
    
    thetadr_old = thetadr; 
    thetadl_old = thetadl;
    Il_REF_old = Il_REF;
    Ir_REF_old = Ir_REF;
    
    t_old1 = micros();
  }

  
  //////////////////////////////////////////////////////////////////
  // Ciclo a 1 Hz  ////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////

  if (dt2 > 1e6)
  {   
    if (abs(nstepR_old - deltar*Nstep/(2*pi))< 45){;}
    else {nstepR_old = deltar*Nstep/(2*pi);}
    if (abs(nstepL_old - deltar*Nstep/(2*pi))< 45){;}
    else {nstepL_old = deltal*Nstep/(2*pi);}    

    t_old2 = micros();
  }

  //////////////////////////////////////////////////////////////////
  // Ciclo a 10 Hz  ///////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////

  if (dt3 > 100000)
  { 
    
    deltar = read_EMS22A(ENC_R_CS1_PIN, deltar); //rad
    delayMicroseconds(10);
    deltal = read_EMS22A(ENC_L_CS1_PIN, deltal); //rad
    
    fkineVel(deltar, thetadr*pi/30, deltal, thetadl*pi/30);
    //sprintf(str_fb_q, "%f %f %f", odom_vx, odom_vy, odom_wz);
    //sprintf(str_fb_q, "%f %f %f %f", Ir, Il, Ir_REF, Il_REF);
    sprintf(str_fb_q, "%f %f %f %f", deltar, deltal, deltar_REF, deltal_REF);
    //sprintf(str_fb_q, "%f %f", thetadr_REF, thetadl_REF);
    //sprintf(str_fb_q, "%f", mapF(abs(Ir_REF),0,ecd_r_imax,255*0.1,255*0.9));
    
    
    if (!enb_SERIALPRINT){
      str_msg.data = str_fb_q;
      fb_q.publish( &str_msg );
      nh.spinOnce();
    }
    else {Serial.println(String(str_fb_q));
      }
    
    t_old3 = micros();
  }
}
