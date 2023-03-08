// Funzione per la lettura dell'Encoder EMS22A

// COLLEGAMENTI EMS22A-Teensy4.1 usando SPI1:
// EMS22A       Teensy4.1
//
// Pin 1 (DI)   Pin 26 (M0SI)
// Pin 2 (CLK)  Pin 27 (SCK)
// Pin 3 (GND)  GND
// Pin 4 (DO)   Pin 39 (MISO)
// Pin 5 (VCC)  3.3V
// Pin 6 (CS)   Pin 38 (da definire a codice)


///////////////////////////////////
///      Lettura in radianti    ///
///////////////////////////////////


float read_EMS22A(int CS_PIN, float old_value)
{ 
  digitalWrite(CS_PIN, LOW);  // metto LOW il CS per iniziare la comunicazione SPI
  int f8bit = SPI1.transfer(0);      // trasferisco 8 zeri per ricevere i primi 8 bit
  int l8bit = SPI1.transfer(0);      // trasferisco 8 zeri per ricevere gli ultimi 8 bit
  delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH); // metto HIGH in CS per concludere la comunicazione SPI
  
  int data = (f8bit<<8) | (l8bit); 

  int ang_bin = data>>6;
  int ang_dec = ang_bin;
  float ang_rad = 2.0*3.1415927/1024 * (ang_dec);

  if (ang_rad > pi)
  {
    ang_rad = ang_rad - 2*pi;
  }
  
  
  if (abs(ang_rad - old_value) < 0.015 || abs(ang_rad - old_value) > 6.265)
  {
    return old_value;
  }
  else
  {
    return ang_rad;
  }
  
  
}
