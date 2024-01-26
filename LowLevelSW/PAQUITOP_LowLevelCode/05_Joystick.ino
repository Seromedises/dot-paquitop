void joystickRead(){

  // If available Read the Joystick:
  if(joystick.read(&channels[0], &failSafe, &lostFrame)){
  
    // Reading and Mapping of the three analog signals Lx, Rx, Ry:
    if (channels[3]<= 992){Lx = map_51(mapF(channels[3],172,992,w_max,0), w_max, w_max); LxvArm = map_51(mapF(channels[3],172,992,vArm_max,0), vArm_max, vArm_max); LxwArm = map_51(mapF(channels[3],172,992,wArm_max,0), wArm_max, wArm_max);}
    else {Lx = map_51(mapF(channels[3],993,1811,0,-w_max), w_max, w_max); LxvArm = map_51(mapF(channels[3],993,1811,0,-vArm_max), vArm_max, vArm_max); LxwArm = map_51(mapF(channels[3],993,1811,0,-wArm_max), wArm_max, wArm_max);}
  
    if (channels[1]<= 990){Rx = map_51(mapF(channels[1],172,990,-v_max,0), v_max, v_max); RxvArm = map_51(mapF(channels[1],172,990,-vArm_max,0), vArm_max, vArm_max); RxwArm = map_51(mapF(channels[1],172,990,-wArm_max,0), wArm_max, wArm_max);}
    else {Rx = map_51(mapF(channels[1],991,1811,0,v_max), v_max, v_max); RxvArm = map_51(mapF(channels[1],991,1811,0,vArm_max), vArm_max, vArm_max); RxwArm = map_51(mapF(channels[1],991,1811,0,wArm_max), wArm_max, wArm_max);}
  
    if (channels[2]<= 997){Ry = map_51(mapF(channels[2],172,997,-v_max,0), v_max, v_max); RyvArm = map_51(mapF(channels[2],172,997,-vArm_max,0), vArm_max, vArm_max); RywArm = map_51(mapF(channels[2],172,997,-wArm_max,0), wArm_max, wArm_max);}
    else {Ry = map_51(mapF(channels[2],998,1811,0,v_max), v_max, v_max); RyvArm = map_51(mapF(channels[2],998,1811,0,vArm_max), vArm_max, vArm_max); RywArm = map_51(mapF(channels[2],998,1811,0,wArm_max), wArm_max, wArm_max);}
  
    // If AUTOMATIC MODE is selected:
    if (channels[5] == 1811){
      enb_ArmManualMode = false;
  
      //ROS Navigation:    
      if (channels[6] == 172){enb_ROSsub = true; wConf = 1;}
      
      //Tablet Store:
      else if (channels[6] == 992){
        enb_ROSsub = false;
        vx = 0.0;
        vy = 0.0;
        gammad = 0.0;
      }
      
      //Tablet Deliver:
      else if (channels[6] == 1811){
        enb_ROSsub = false;
        vx = 0.0;
        vy = 0.0;
        gammad = 0.0;
      }
    }
    
    // If ARM MANUAL MODE is selected:
    else if (channels[5] == 992){
      enb_ArmManualMode = true;
      enb_ROSsub = false;

      // Pure Translation:
      if (channels[6] == 172){
        arm_msg.linear.x = RxvArm;
        arm_msg.linear.y = RyvArm;
        arm_msg.linear.z = LxvArm;
        arm_msg.angular.x = 0.0;
        arm_msg.angular.y = 0.0;
        arm_msg.angular.z = 0.0;
        }
  
      // Pure Rotation:
      else if (channels[6] == 992){
        arm_msg.linear.x = 0.0;
        arm_msg.linear.y = 0.0;
        arm_msg.linear.z = 0.0;
        arm_msg.angular.x = RxvArm;
        arm_msg.angular.y = RyvArm;
        arm_msg.angular.z = LxvArm;
        }
  
      // Arm Rest Pose :
      else if ((channels[6] == 1811)){
        arm_msg.linear.x = 0.0;
        arm_msg.linear.y = 0.0;
        arm_msg.linear.z = 0.0;
        arm_msg.angular.x = 0.0;
        arm_msg.angular.y = 0.0;
        arm_msg.angular.z = 0.0;
        }      
      vx = 0.0;
      vy = 0.0;
      gammad = 0.0;
      enb_ROSsub = false;
    }
  
    // If BASE MANUAL MODE is selected:
    else if (channels[5] == 172){ 
      
      enb_ArmManualMode = false;
      enb_ROSsub = false;
  
      // Working Configuration II:
      if (channels[6] == 172){
        wConf = 2;
        vx = Ry;
        vy = 0.0;
        gammad = -Rx;}
  
      // Working Configuration I:
      else if (channels[6] == 992){
        wConf = 1;
        vx = -Rx;
        vy = -Ry;
        gammad = Lx;}
  
      // Working Configuration III:
      else if ((channels[6] == 1811)){
        wConf = 3;
        vx = 0.0;
        vy = -Ry;
        gammad = Rx;}
    }

    if ((abs(vx) <= 0.05 && abs(vy) <= 0.05 && abs(gammad) <= 0.05)){tHidle += dt1;}
    else {tHidle = 0;}
    
    if ((channels[4] == 1811) || (tHidle >= tHidle_max)){       
      motorsDIS();
      vx = 0.0;
      vy = 0.0;
      gammad = 0.0;
    }
    else if (((channels[4] == 172) && (tHidle < tHidle_max)) || ((channels[4] == 992) && (tHidle < tHidle_max))){          
      motorsENB();
    }
  }
}

float mapF(float x, float in_min, float in_max, float out_min, float out_max)
{ 
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float map_51(float IN,float INmax,float OUTmax)
{
  volatile float INlim = INmax/4;
  volatile float b1 = (0.7*OUTmax)/(INmax-INlim);

  volatile float a3 = (2*(5*OUTmax - 5*b1*INmax + 3*b1*INlim))/pow(INlim,3);
  volatile float a4 = -(15*OUTmax - 15*b1*INmax + 8*b1*INlim)/pow(INlim,4);
  volatile float a5 = (3*(2*OUTmax - 2*b1*INmax + b1*INlim))/pow(INlim,5);
  volatile float b0 = OUTmax - b1*INmax;

  if (abs(IN)>=0 && abs(IN)<=INlim)
  {
    if (IN >= 0){return a5*pow(IN,5) + a4*pow(IN,4) + a3*pow(IN,3);} 
    else{return -(a5*pow(-IN,5) + a4*pow(-IN,4) + a3*pow(-IN,3));} 
  }
  if (abs(IN)>INlim && abs(IN)<=INmax)
  {
    if (IN >= 0){return b1*IN + b0;}  
    else {return b1*IN - b0;}
  }    
}
