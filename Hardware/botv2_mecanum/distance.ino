void measureDistanceFiltered() {
  measureDistanceFrontFiltered();
  measureDistanceLeftFiltered();
  measureDistanceRightFiltered();
}

void measureDistanceFrontFiltered() {
  delay(300);
  int _d1, _d2, _d3;
  _d1 = measureDistanceFront();
  delay(150);
  _d2 = measureDistanceFront();
  delay(150);
  _d3 = measureDistanceFront();
  while ((abs(_d1 - _d2) > 2) || (abs(_d2 - _d3) > 2) || (abs(_d1 - _d3) > 2)) {
    _d1 = measureDistanceFront();
    delay(150);
    _d2 = measureDistanceFront();
    delay(150);
    _d3 = measureDistanceFront();
    if (SERIAL_PRINT){Serial.print(" - NOISE - ");}
    delay(150);
  }
  //Serial.print("first value: "); Serial.print(_d1); Serial.print(" second value: "); Serial.print(_d2); Serial.print(" third value: "); Serial.print(_d3); 
  //Serial.print(" AVERAGE FRONT: "); Serial.println((_d1+_d2+_d3)/3);
  frontDistance = (_d1+_d2+_d3)/3;
}

int measureDistanceFront() {
  digitalWrite(F_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(F_TRIG, HIGH);
  delayMicroseconds(10); // triggered by ten or more
  digitalWrite(F_TRIG, LOW);
  const unsigned long durationF = pulseIn(F_ECHO, HIGH);
  return durationF / 29 / 2;
//  return frontDistance;
  //  if (durationF == 0) {
  //    Serial.println("Warning: no pulse from front sensor");
  //  }
}

void measureDistanceLeftFiltered() {
  delay(300);
  int _d1, _d2, _d3;
  _d1 = measureDistanceLeft();
  delay(150);
  _d2 = measureDistanceLeft();
  delay(150);
  _d3 = measureDistanceLeft();
  while ((abs(_d1 - _d2) > 2) || (abs(_d2 - _d3) > 2) || (abs(_d1 - _d3) > 2)) {
    _d1 = measureDistanceLeft();
    delay(150);
    _d2 = measureDistanceLeft();
    delay(150);
    _d3 = measureDistanceLeft();
    if (SERIAL_PRINT){Serial.print(" - NOISE - ");}
    delay(150);
  }
  //Serial.print("first value: "); Serial.print(_d1); Serial.print(" second value: "); Serial.print(_d2); Serial.print(" third value: "); Serial.print(_d3); 
  //Serial.print(" AVERAGE LEFT: "); Serial.println((_d1+_d2+_d3)/3);
  leftDistance = (_d1+_d2+_d3)/3;
}

int measureDistanceLeft() {
  digitalWrite(L_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(L_TRIG, HIGH);
  delayMicroseconds(10); // triggered by ten or more
  digitalWrite(L_TRIG, LOW);
  const unsigned long durationL = pulseIn(L_ECHO, HIGH);
  return durationL / 29 / 2;
//  return leftDistance;
  //  if (durationL == 0) {
  //    Serial.println("Warning: no pulse from left sensor");
  //  }
}

void measureDistanceRightFiltered() {
  delay(300);
  int _d1, _d2, _d3;
  _d1 = measureDistanceRight();
  delay(150);
  _d2 = measureDistanceRight();
  delay(150);
  _d3 = measureDistanceRight();
  while ((abs(_d1 - _d2) > 2) || (abs(_d2 - _d3) > 2) || (abs(_d1 - _d3) > 2)) {
    _d1 = measureDistanceRight();
    delay(150);
    _d2 = measureDistanceRight();
    delay(150);
    _d3 = measureDistanceRight();
    if (SERIAL_PRINT){Serial.print(" - NOISE - ");}
    delay(150);
  }
//  Serial.print("first value: "); Serial.print(_d1); Serial.print(" second value: "); Serial.print(_d2); Serial.print(" third value: "); Serial.print(_d3); 
//  Serial.print(" AVERAGE RIGHT: "); Serial.println((_d1+_d2+_d3)/3);
  rightDistance = (_d1+_d2+_d3)/3;
}

int measureDistanceRight() {
  digitalWrite(R_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(R_TRIG, HIGH);
  delayMicroseconds(20); // triggered by ten or more
  digitalWrite(R_TRIG, LOW);
  const unsigned long durationR = pulseIn(R_ECHO, HIGH);
  return durationR / 29 / 2;
  /*if (durationR == 0) {
    Serial.println("Warning: no pulse from right sensor");
    }
  */
}

void writeData() {
  //  Serial.print(frontDistance);
  //  Serial.print("  ");
  //  Serial.print(leftDistance);
  //  Serial.print("  ");
  //  Serial.print(rightDistance);
  //  Serial.println("  ");
  lcd.setCursor(0, 1);
  String line1 = "F:" + String(frontDistance) + " R:" + String(rightDistance) + " L:" + String(leftDistance) + "   ";
  lcd.print(line1);
  lcd.setCursor(0, 0);
  lcd.print("Status: " + currentMode); // + " " + currentDirection);
}
