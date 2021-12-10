void alarmCallback(uint8_t sensorId, uint8_t thresh, uint16_t dist){
  if(thresh == ALARM_YELLOW) {
//    Serial.print("Ricevuto allarme giallo da sensore: ");
//    Serial.println(sensorId);
//    Serial.print("Distanza: ");
//    Serial.println(dist);
    //scale_vx = 0.5;
    //scale_vy = 0.5;
    //scale_gammad = 0.5;
    tSens = micros();
  }
  else if(thresh == ALARM_RED) {
//    Serial.print("Ricevuto allarme rosso da sensore: ");
//    Serial.println(sensorId);
//    Serial.print("Distanza: ");
//    Serial.println(dist);
    //scale_vx = 0.01;
    //scale_vy = 0.01;
    //scale_gammad = 0.01;
    tSens = micros();
  } else if(thresh  == ALARM_LASER) {
//    Serial.print("Ricevuto allarme laser da sensore: ");
//    Serial.println(sensorId);
    //sul laser non mando la distanza
  }


  if(sensorId == sensor3) {
//    Serial.println("L'allarme è stato ricevuto dal sensore 3");
  }
}
