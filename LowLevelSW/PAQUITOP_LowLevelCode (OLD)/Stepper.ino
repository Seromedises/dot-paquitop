 // Funzioni per il controllo dei motori stepper

///////////////////////////
// ABILITA/DISABILITA    //
///////////////////////////

// Funzione che abilita e disabilita il motore collegato in SPI attraverso il Chip Select SCS
// ENB = 0b0/0b1 --> disattivo/attivo il MOTORE;

/////////////////////////////////
/* CTRL register of the DRIVER:

ISGAIN:
GAIN 5 : 00; GAIN 10: 01;
GAIN 20: 10; GAIN 40: 11;

MODE:
Full : 0000;    1/2  : 0001; 
1/4  : 0010;    1/8  : 0011;
1/16 : 0100;    1/32 : 0101; 
1/64 : 0110;    1/128: 0111;
1/256: 1000;

SPI.transfer(RW<<7 | ADD<<4 | DTIME<<2 | ISGAIN);
SPI.transfer(EXTALL<<7 | MODE<<4 | RSTEP<<3 | RDIR<<2 | ENBL);	
*/
/////////////////////////////////

void enb_SMD(int SCS,byte ENBL){
    
  digitalWrite(SCS, HIGH);  //setting SCS_pin high to start transfer
  SPI.transfer(0<<7 | 0b000<<4 | 0b11<<2 | 0b11);
  SPI.transfer(0b0<<7 | 0b0011<<4 | 0b0<<3 | 0b0<<2 | ENBL);
  delayMicroseconds(3);
  digitalWrite(SCS, LOW); // stopping transer 
}

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
			goStepperR(LOW);
			nstepR_old += 1;
		}
		else if (dnR < 0)
		{
			goStepperR(HIGH);
			nstepR_old -= 1;
		}
	}
	else
	{
		if (dnR > 0)
		{
			goStepperR(HIGH);
			nstepR_old = deltar_REF*Nstep/(2*pi);
		}
		else if (dnR < 0)
		{
			goStepperR(LOW);
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
			goStepperL(LOW);
			nstepL_old += 1;
		}
		else if (dnL < 0)
		{
			goStepperL(HIGH);
			nstepL_old -= 1;
		}
	}
	else
	{
		if (dnL > 0)
		{
			goStepperL(HIGH);
      nstepL_old = deltal_REF*Nstep/(2*pi);
		}
		else if (dnL < 0)
		{
			goStepperL(LOW);
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
  delayMicroseconds(1);
  digitalWrite(SMD_R_STEP_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(SMD_R_STEP_PIN, LOW);
  delayMicroseconds(2);
}

void goStepperL(int DIRL)
{
  digitalWrite(SMD_L_DIR_PIN, DIRL);
  delayMicroseconds(1);
  digitalWrite(SMD_L_STEP_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(SMD_L_STEP_PIN, LOW);
  delayMicroseconds(2);
}
