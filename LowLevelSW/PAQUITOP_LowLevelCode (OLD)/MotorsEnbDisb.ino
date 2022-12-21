// Motors Enable/Disable

void enb_Motors(){
  digitalWrite(ECD_R_ENB_PIN, LOW);
  digitalWrite(ECD_L_ENB_PIN, LOW);
  digitalWrite(SMD_R_SLEEP_PIN, HIGH);
  digitalWrite(SMD_L_SLEEP_PIN, HIGH);
  delay(1);
  enb_SMD(SMD_R_SCS_PIN,0b1);
  enb_SMD(SMD_L_SCS_PIN,0b1);
  digitalWrite(ECD_R_ENB_PIN, HIGH);
  digitalWrite(ECD_L_ENB_PIN, HIGH);
}

void disb_Motors(){
  enb_SMD(SMD_R_SCS_PIN,0b0);
  enb_SMD(SMD_L_SCS_PIN,0b0);
  digitalWrite(SMD_R_SLEEP_PIN, LOW);
  digitalWrite(SMD_L_SLEEP_PIN, LOW);
  delay(1);
  digitalWrite(ECD_R_ENB_PIN, LOW);
  digitalWrite(ECD_L_ENB_PIN, LOW);
}
