 // Funzioni per il controllo dei motori stepper

///////////////////////////
// O.L. Control          //
///////////////////////////

// Open Loop for stepper motors

void OL_StepperR(int nstepR_REF) 
{	
	dnR = nstepR_REF - nstepR_old;

	if (abs(dnR) <= Nstep/2)
	{
		if (dnR > 0)
		{
			goStepperR(HIGH);
			nstepR_old += 1;
		}
		else if (dnR < 0)
		{
			goStepperR(LOW);
			nstepR_old -= 1;
		}
	}
	else
	{
		if (dnR > 0)
		{
			goStepperR(LOW);
			nstepR_old = deltar_REF*Nstep/(2*pi);
		}
		else if (dnR < 0)
		{
			goStepperR(HIGH);
			nstepR_old = deltar_REF*Nstep/(2*pi);
		}
	}
}

void OL_StepperL(int nstepL_REF) 
{	
	dnL = nstepL_REF - nstepL_old;
  
	if (abs(dnL) <= Nstep/2)
	{
		if (dnL > 0)
		{
			goStepperL(HIGH);
			nstepL_old += 1;
		}
		else if (dnL < 0)
		{
			goStepperL(LOW);
			nstepL_old -= 1;
		}
	}
	else
	{
		if (dnL > 0)
		{
			goStepperL(LOW);
      nstepL_old = deltal_REF*Nstep/(2*pi);
		}
		else if (dnL < 0)
		{
			goStepperL(HIGH);
      nstepL_old = deltal_REF*Nstep/(2*pi);
		}
	}
}

// Funzione goStepper() permette di eseguire uno step in avanti in accordo con le tempistiche del MOTORE e del DRIVER:
//
// INPUT:
// - t_in: tempo iniziale in cui inizio a fare lo step (micros());
// - DIR: direzione in cui voglio andare, 0/1 (anti)orario/anti(orario) ??!!;

void goStepperR(int DIRR)
{
  digitalWrite(SMD_R_DIR_PIN, DIRR);
  delayMicroseconds(5);
  digitalWrite(SMD_R_STEP_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(SMD_R_STEP_PIN,LOW);
  delayMicroseconds(100);
}

void goStepperL(int DIRL)
{
  digitalWrite(SMD_L_DIR_PIN, DIRL);
  delayMicroseconds(5);
  digitalWrite(SMD_L_STEP_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(SMD_L_STEP_PIN,LOW);
  delayMicroseconds(100);
}
