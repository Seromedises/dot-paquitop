
// The function read_EMS22A() reads the encoder connected to the SPI1 Bus on the CS = CS_PIN.
// The output of the function is the steering angle in the range [-pi,pi].
// The function filters out noise thorugh a dead band of span = 2*span. 
  
float read_EMS22A(int CS_PIN, float old_value){ 
  
  digitalWrite(CS_PIN, LOW);                  //activate SPI communication
  volatile int f8bit = SPI1.transfer(0);      //read the first 8 bits
  volatile int l8bit = SPI1.transfer(0);      //read the last 8 bits
  delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);                 //deactivate SPI communication
  
  volatile int data = (f8bit<<8) | (l8bit); 
  volatile int ang_bin = data>>6;
  volatile int ang_dec = ang_bin;
  volatile float ang_rad = 2*pi/1024 * (ang_dec);
  volatile float span = 0.010;

  if (ang_rad > pi)
  {
    ang_rad = ang_rad - 2*pi;
  }
  
  
  if (abs(ang_rad - old_value) < span || abs(ang_rad - old_value) > (2*pi-span))
  {
    return old_value;
  }
  else
  {
    return ang_rad;
  }
  
  
}
