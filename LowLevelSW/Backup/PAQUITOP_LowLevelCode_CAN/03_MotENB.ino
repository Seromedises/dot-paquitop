void motorsENB(){
  
  digitalWrite(ST_ENB,LOW);
  digitalWrite(MW_ENB,HIGH);
}

void motorsDIS(){
  
  digitalWrite(ST_ENB,HIGH);
  digitalWrite(MW_ENB,LOW);
}
