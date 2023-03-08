float PID_ctrl(float ts, float last_crtl, float ref, float measure, float last_measure, float kp, float kd, float ki, float outMin, float outMax) {
  ts = ts/1000000;
  float error = ref - measure; // error
  DER = -kd * (measure - last_measure) / (ts); //Note: the derivative term use just the measure and not the value to avoid derivative kick.
  //In particular, we use -d(measure)/dt. Check theroy for more details.
  if ((error * last_crtl) > 0 && abs(last_crtl) >= outMax) { //Anti-Windup: Integral Clamping
    INT += 0; // IF the cmd is beyond the limits and has the same sign of the error, it stops updating the integral term
  }
  else {
    INT += ki * (error) * (ts); //ELSE compute the integral term as usual
  }
  float ctrl = kp * error + INT + DER; // controller output, Parallel PID form
  ctrl = constrain(ctrl , outMin , outMax); // Controller output Saturation
  return ctrl; 
}
