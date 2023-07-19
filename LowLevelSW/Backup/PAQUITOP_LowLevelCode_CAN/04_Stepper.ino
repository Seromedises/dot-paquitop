void goStep1(int dir1){
  
  digitalWrite(ST_DIR_1, dir1);
  delayMicroseconds(10);
  digitalWrite(ST_STEP_1, HIGH);
  delayMicroseconds(450);
  digitalWrite(ST_STEP_1,LOW);
  delayMicroseconds(450);
}

void goStep2(int dir2){
  
  digitalWrite(ST_DIR_2, dir2);
  delayMicroseconds(10);
  digitalWrite(ST_STEP_2, HIGH);
  delayMicroseconds(450);
  digitalWrite(ST_STEP_2,LOW);
  delayMicroseconds(450);
}

void contrStep1(int step_REF){
  e_step1 = step_REF - step1_OLD;

  if (abs(e_step1) <= Nstep/2){
    if (e_step1 > 0){goStep1(HIGH); step1_OLD +=1;}
    else if (e_step1 < 0){goStep1(LOW); step1_OLD -=1;}
  }
  else {
    if (e_step1 > 0){goStep1(LOW); step1_OLD = step1_REF - 1;}
    else if (e_step1 < 0){goStep1(HIGH); step1_OLD = step1_REF +1;}
  }
}

void contrStep2(int step_REF){
  
  e_step2 = step_REF - step2_OLD;

  if (abs(e_step2) <= Nstep/2){
    if (e_step2 > 0){goStep2(HIGH); step2_OLD +=1;}
    else if (e_step2 < 0){goStep2(LOW); step2_OLD -=1;}
  }
  else {
    if (e_step2 > 0){goStep2(LOW); step2_OLD = step2_REF - 1;}
    else if (e_step2 < 0){goStep2(HIGH); step2_OLD = step2_REF + 1;}
  }
}

void updateStepPos(){
  
    delta1 = read_EMS22A(ENC_CS_1, delta1);
    delta2 = read_EMS22A(ENC_CS_2, delta2);
  
    step1 = delta1*Nstep/2/pi;
    step2 = delta2*Nstep/2/pi;

    if (abs(step1_OLD - step1) < dn){;}
    else {step1_OLD = step1;}
    if (abs(step2_OLD - step2) < dn){;}
    else {step2_OLD = step2;}
}
