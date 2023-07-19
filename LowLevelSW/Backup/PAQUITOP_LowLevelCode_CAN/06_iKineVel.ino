void ikineVel(int wConf, float vx, float vy, float gammad){

  switch(wConf){

    // Working Configuration I

    case 1:

    if (abs(vx) <= 0.05 && abs(vy) <= 0.05 && abs(gammad) <= 0.05){
      delta1_REF = dr_old;
      delta2_REF = dl_old;
      thd1_REF = 0;
      thd2_REF = 0;}
      
    else {
      
      thdr1 = 1/rw*sqrt(pow(vx+a*gammad,2)+pow(vy,2));
      thdr2 = -1/rw*sqrt(pow(vx+a*gammad,2)+pow(vy,2));
      
      thdl1 = 1/rw*sqrt(pow(vx-a*gammad,2)+pow(vy,2));
      thdl2 = -1/rw*sqrt(pow(vx-a*gammad,2)+pow(vy,2));

      dr1 = atan2((vy)/(rw*thdr1),(vx+a*gammad)/(rw*thdr1));
      dr2 = atan2((vy)/(rw*thdr2),(vx+a*gammad)/(rw*thdr2));
      
      dl1 = atan2((vy)/(rw*thdl1),(vx-a*gammad)/(rw*thdl1));
      dl2 = atan2((vy)/(rw*thdl2),(vx-a*gammad)/(rw*thdl2));

      // Versione semplificata: (PROBLEMA DA -180 A +180 e viceversa) 
      if (abs(dr1-dr_old) <= abs(dr2-dr_old))
      {
        if (abs(dr2-dr_old) > 2*pi - 0.08){
          delta1_REF = dr2;
          thd1_REF = thdr2;
          dr_old = dr2;}
        else {
          delta1_REF = dr1;
          thd1_REF = thdr1;
          dr_old = dr1;}
      }
      else
      {
        if (abs(dr1-dr_old) > 2*pi - 0.08){
          delta1_REF = dr1;
          thd1_REF = thdr1;
          dr_old = dr1;}
        else {
          delta1_REF = dr2;
          thd1_REF = thdr2;
          dr_old = dr2;}
      }

      if (abs(dl1-dl_old) <= abs(dl2-dl_old))
      {
        if (abs(dl2-dl_old) > 2*pi - 0.08) {
          delta2_REF = dl2;
          thd2_REF = thdl2;
          dl_old = dl2;}
        else {
          delta2_REF = dl1;
          thd2_REF = thdl1;
          dl_old = dl1;}
      }
      else
      {
        if (abs(dl1-dl_old) > 2*pi - 0.08){
          delta2_REF = dl1;
          thd2_REF = thdl1;
          dl_old = dl1;}
        else {
          delta2_REF = dl2;
          thd2_REF = thdl2;
          dl_old = dl2;}
      }
    }    
    
    break;

///////////////////////////////////////////////////////////

    // Working Configuration II

    case 2:
    thd1_REF = vx/(rw)+(a*gammad)/rw;
    thd2_REF = vx/(rw)-(a*gammad)/rw;
    delta1_REF = 0;
    delta2_REF = 0;
    
    break;

///////////////////////////////////////////////////////////

    // Working Configuration III

    case 3:

    delta = -gammad;
    delta2_REF = pi/2 - delta;
    delta1_REF = pi/2 + delta;
    thd1_REF = vy/(rw*cos(delta));
    thd2_REF = thd1_REF;

    break;
  }
}
