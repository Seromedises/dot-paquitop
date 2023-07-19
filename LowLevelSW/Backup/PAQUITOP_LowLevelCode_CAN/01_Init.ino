void pinInit(){//pin initialization:

  pinMode(ENC_CS_1,OUTPUT);
  pinMode(ENC_CS_2,OUTPUT);
  
  pinMode(ST_ENB,OUTPUT);
  pinMode(ST_MS1,OUTPUT);
  pinMode(ST_MS2,OUTPUT);
  
  pinMode(ST_STEP_1,OUTPUT);
  pinMode(ST_DIR_1,OUTPUT);
  pinMode(ST_STEP_2,OUTPUT);
  pinMode(ST_DIR_2,OUTPUT);

  pinMode(MW_ENB,OUTPUT);

  pinMode(MW_PWM_1,OUTPUT);
  pinMode(MW_DIR_1,OUTPUT);
  pinMode(MW_FB_i1,INPUT);
  pinMode(MW_FB_w1,INPUT);

  pinMode(MW_PWM_2,OUTPUT);
  pinMode(MW_DIR_2,OUTPUT);
  pinMode(MW_FB_i2,INPUT);
  pinMode(MW_FB_w2,INPUT);
}

void commInit(){//communications ports initialization:
  
  //Serial Port:
  Serial.begin(115200);
  delay(1);

  //Joystick Serial1
  Serial1.begin(100000, 8E2); // Sbus
  delay(1);
  joystick.begin();
  delay(1);

  //ROS:
  nh.initNode();
  nh.subscribe(sub);
  delay(1);

  //SPI1 bus:
  SPI1.setMOSI(ENC_MOSI);
  SPI1.setMISO(ENC_MISO);
  SPI1.setSCK(ENC_SCK);
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); 
  //Frequency 1MHz, MSBFIRST, MODE 1 
  delay(1);

  //Prox sensor communication:
  CANsensorBegin();
}

void motorsInit(){ //motors initialization:

  //Set the uStepping ratio to 2:
  digitalWrite(ST_MS1,HIGH);
  digitalWrite(ST_MS2,LOW);

  //Set PWM frequency and REF = 0:
  analogWriteFrequency(MW_PWM_1, 536);
  analogWriteFrequency(MW_PWM_2, 536);
  analogWrite(MW_PWM_1,255*0.1);
  analogWrite(MW_PWM_2,255*0.1);
}

void CANsensorBegin(){
  // Initialization:
  proxSensors.begin();

  // Check the default thresholds:
  threshold_t checkSoglie = proxSensors.getThreshold();
  Serial.print("Soglie impostate: giallo: ");
  Serial.print(checkSoglie.yellowThreshold);
  Serial.print(", rosso: ");
  Serial.print(checkSoglie.redThreshold);
  Serial.print(", laser: ");
  Serial.println(checkSoglie.laserThreshold);

  // Thresholds set:
  //proxSensors.setThreshold({.yellowThreshold = 1, .redThreshold = 2, .laserThreshold = 27, .alarmTimeout = 2000});

  // Initial distances request:
  dist_t distanza = proxSensors.requestDistance(sensor1);
  if(distanza.error) {
    Serial.println("Non è stato possibile rilevare la distanza dal sensore 1.");
  } else {
    Serial.print("Distanze attualmente rilevate dal sensore 1: laser: ");
    Serial.print(distanza.distLaser);
    Serial.print(", sonar: ");
    Serial.println(distanza.distSonar);
  }
}
