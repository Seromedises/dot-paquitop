float ikineVel(int modOp, float vx, float vy, float gammad){

  switch(modOp){

    // MOTO GENERICO I

    case 1:

    if (abs(vx) <= 0.05 && abs(vy) <= 0.05 && abs(gammad) <= 0.05)
    {
      deltar_REF = dr_old;
      deltal_REF = dl_old;
      thetadr_REF = 0;
      thetadl_REF = 0;
    }
    else
    {
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
        if (abs(dr2-dr_old) > 2*pi - 0.08)
        {
          deltar_REF = dr2;
          thetadr_REF = thdr2;
          dr_old = dr2;
        }
        else
        {
          deltar_REF = dr1;
        thetadr_REF = thdr1;
        dr_old = dr1;
        }
      }
      else
      {
        if (abs(dr1-dr_old) > 2*pi - 0.08)
        {
          deltar_REF = dr1;
          thetadr_REF = thdr1;
          dr_old = dr1;
        }
        else
        {
          deltar_REF = dr2;
          thetadr_REF = thdr2;
          dr_old = dr2;
        }
      }

      if (abs(dl1-dl_old) <= abs(dl2-dl_old))
      {
        if (abs(dl2-dl_old) > 2*pi - 0.08)
        {
          deltal_REF = dl2;
          thetadl_REF = thdl2;
          dl_old = dl2;
        }
        else
        {
          deltal_REF = dl1;
          thetadl_REF = thdl1;
          dl_old = dl1;
        }
      }
      else
      {
        if (abs(dl1-dl_old) > 2*pi - 0.08)
        {
          deltal_REF = dl1;
          thetadl_REF = thdl1;
          dl_old = dl1;
        }
        else
        {
          deltal_REF = dl2;
          thetadl_REF = thdl2;
          dl_old = dl2;
        }
      }
    }    

    return deltar_REF;
    return deltal_REF;
    return thetadr_REF;
    return thetadl_REF;
    break;

///////////////////////////////////////////////////////////

    // STERZATURA DIFFERENZIALE II

    case 2:
    thetadr_REF = vx/(rw)+(a*gammad)/rw;
    thetadl_REF = vx/(rw)-(a*gammad)/rw;
    deltar_REF = 0;
    deltal_REF = 0;
    
    return deltar_REF;
    return deltal_REF;
    return thetadr_REF;
    return thetadl_REF;
    break;

///////////////////////////////////////////////////////////

    // BICI (RUOTA ANTERIORE) III.a NON AGGIORNATA

    case 3:

		if (vy >= 0)
    {
        delta = gammad;
				// delta = atan2((2*a*gammad), (vy));
        deltal_REF = pi/2 + delta;
        deltar_REF = pi/2;
        thetadr_REF = vy/(rw);
        thetadl_REF = vy/(rw*cos(delta));
    }
    else
    {
      	delta = gammad;
				// delta = atan2((2*a*gammad), (vy));
        deltal_REF = pi/2;
        deltar_REF = pi/2 + delta;
        thetadr_REF = vy/(rw*cos(delta));
        thetadl_REF = vy/(rw);
    }

    return deltar_REF;
    return deltal_REF;
    return thetadr_REF;
    return thetadl_REF;
    break;

///////////////////////////////////////////////////////////

    // BICI (RUOTA POSTERIORE) III.b NON AGGIORNATA

    case 4:
    if (vy >= 0)
    {
        delta = -gammad;
				// delta = atan2((-2*a*gammad), (vy));
        deltal_REF = pi/2;
        deltar_REF = pi/2 + delta;;
        thetadr_REF = vy/(rw*cos(delta));
        thetadl_REF = vy/(rw);
    }
    else
    {
      	delta = -gammad;
				// delta = atan2((-2*a*gammad), (vy));
        deltal_REF = pi/2 + delta;
        deltar_REF = pi/2;
        thetadr_REF = vy/(rw);
        thetadl_REF = vy/(rw*cos(delta));
    }
    
    
    return deltar_REF;
    return deltal_REF;
    return thetadr_REF;
    return thetadl_REF;
    break;

///////////////////////////////////////////////////////////

    // BICI (ENTRAMBE LE RUOTE) III.c

    case 5:

    delta = -gammad;
    deltal_REF = pi/2 - delta;
    deltar_REF = pi/2 + delta;
    thetadr_REF = vy/(rw*cos(delta));
    thetadl_REF = thetadr_REF;

    /*
    if (vy >= 0)
    {
        delta = -gammad;
        deltal_REF = pi/2 - delta;
        deltar_REF = pi/2 + delta;
        thetadr_REF = vy/(rw*cos(delta));
        thetadl_REF = thetadr_REF;
    }
    else
    {
      
        delta = -gammad;
        deltal_REF = pi/2 + delta;
        deltar_REF = pi/2 - delta;
        thetadr_REF = vy/(rw*cos(delta));
        thetadl_REF = thetadr_REF;
    }*/

    
    return deltar_REF;
    return deltal_REF;
    return thetadr_REF;
    return thetadl_REF;
    break;
  }
}
