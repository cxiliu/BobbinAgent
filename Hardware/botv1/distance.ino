String measureHeading() {
  if (accel.available()) {
    // Acceleration of x, y, and z directions in g units
    //    Serial.print(accel.getCalculatedX(), 3);
    //    Serial.print("\t");
    //    Serial.print(accel.getCalculatedY(), 3);
    //    Serial.print("\t");
    //    Serial.print(accel.getCalculatedZ(), 3);
    //    Serial.println();
    float x = (accel.getCalculatedX() - 0.008) * 1000;
    float y = (accel.getCalculatedY() - 0.135) * 1000;
    //    float angle = atan(y/x)/3.1415*180;
    //    Serial.println(angle);
    //    return (int)angle;
    return "Heading: x" +  String(int(x)) + " y" + String(int(y)) + "     ";
  }
  else {
    Serial.println("not avail");
    return "no data";
  }
}

void measureDistance(){
  measureDistanceFront();
  measureDistanceLeft();
  measureDistanceRight();
}

void measureDistanceFront() {
  digitalWrite(F_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(F_TRIG, HIGH);
  delayMicroseconds(10); // triggered by ten or more
  digitalWrite(F_TRIG, LOW);
  const unsigned long durationF = pulseIn(F_ECHO, HIGH);
  frontDistance = durationF / 29 / 2;
  if (durationF == 0) {
    Serial.println("Warning: no pulse from front sensor");
  }
}

void measureDistanceLeft() {
  digitalWrite(L_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(L_TRIG, HIGH);
  delayMicroseconds(10); // triggered by ten or more
  digitalWrite(L_TRIG, LOW);
  const unsigned long durationL = pulseIn(L_ECHO, HIGH);
  leftDistance = durationL / 29 / 2;
  if (durationL == 0) {
    Serial.println("Warning: no pulse from left sensor");
  }
}

void measureDistanceRight() {
  digitalWrite(R_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(R_TRIG, HIGH);
  delayMicroseconds(10); // triggered by ten or more
  digitalWrite(R_TRIG, LOW);
  const unsigned long durationR = pulseIn(R_ECHO, HIGH);
  rightDistance = durationR / 29 / 2;
  /*if (durationR == 0) {
    Serial.println("Warning: no pulse from right sensor");
  }
  */
}

void writeData() {
  lcd.setCursor(0, 0);
  String line1 = "F:" + String(frontDistance) + " R:" + String(rightDistance) + " L:" + String(leftDistance) + "   ";
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(currentMode + " " + currentDirection);
//  lcd.print(headingData);
}
