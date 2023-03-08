// Motors Enable/Disable

void enb_Motors(){
  digitalWrite(ECD_R_ENB_PIN, LOW);
  digitalWrite(ECD_L_ENB_PIN, LOW);
  digitalWrite(SMD_R_SLEEP_PIN, LOW);
  digitalWrite(SMD_L_SLEEP_PIN, LOW);
  delay(5);
  digitalWrite(ECD_R_ENB_PIN, HIGH);
  digitalWrite(ECD_L_ENB_PIN, HIGH);
}

void disb_Motors(){
  digitalWrite(SMD_R_SLEEP_PIN, HIGH);
  digitalWrite(SMD_L_SLEEP_PIN, HIGH);
  delay(5);
  digitalWrite(ECD_R_ENB_PIN, LOW);
  digitalWrite(ECD_L_ENB_PIN, LOW);
}
