float fkineVel(float dr, float thdr, float dl, float thdl){
  
  // Da aggiungere alle variabili globali: 
  // float odom_vx, odom_vy, odom_wz;
  // float thetadr_tol = 5.0, thetadl_tol = 5.0;

  // Da aggiungere al ciclo di FB:
  // fkineVel(deltar, thetadr, deltal, thetadl);
  // sprintf(str_fb_q, "%f %f %f", odom_vx, odom_vy, odom_wz);
  
  if (abs(thdr) <= thetadr_tol){
    thdr = 0.0
  }
  if (abs(thdl) <= thetadl_tol){
    thdl = 0.0
  }

  odom_vx = rw/2*(thdr*cos(dr) + thdl*cos(dl));
  odom_vy = rw/2*(thdr*sin(dr) + thdl*sin(dl));
  odom_wz = rw/(2*a)*(thdr*cos(dr) - thdl*cos(dl));
  
  return odom_vx;
  return odom_vy;
  return odom_wz;
}
