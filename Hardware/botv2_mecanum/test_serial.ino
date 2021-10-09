void LocomotionTest(int _receivedByte) {
  if (!PID_DEBUG_POS && !PID_DEBUG_VEL) {
    Serial.print("Received: ");
    Serial.println(_receivedByte, DEC);
  }
  if (_receivedByte == 119) {
    moveForwardDistance(SET_DISTANCE); //w - forward - 8
  }
  else if (_receivedByte == 115) {
    Stop(); //s - stop - 5
  }
  else if (_receivedByte == 97) {
    moveLeftDistance(SET_DISTANCE); //a - left - 4
  }
  else if (_receivedByte == 100) {
    moveRightDistance(SET_DISTANCE); //d - right - 6
  }
  else if (_receivedByte == 120) {
    moveBackwardDistance(SET_DISTANCE); //x - back - 2
  }
  else if (_receivedByte == 113) {
    rotateDegree(SET_ANGLE); //q - turn left
  }
  else if (_receivedByte == 101) {
    rotateDegree(-SET_ANGLE); //e - turn right
  }
  else if (_receivedByte == 122) { //z left gripper adaptive
    pickupLeftAdaptive(true);
  }
  else if (_receivedByte == 99) { //c - right gripper adaptive
    pickupRightAdaptive(true);
  }
  else if (_receivedByte == 103) {
    Serial.println("lifting bobbins");
    liftBobbins();
    Serial.println("done");
    //      actuateLeftGripper(true); //g - lift
    delay(500);
  }
  else if (_receivedByte == 117) {
    Serial.println("lowering bobbins");
    lowerBobbins();
    Serial.println("done");
    //      actuateLeftGripper(false); //u - lower
    delay(500);
  }
  else if (_receivedByte == 104) {
    measureDistanceFiltered();
    String lin = "F:" + String(frontDistance) + " R:" + String(rightDistance) + " L:" + String(leftDistance) + "   ";
    Serial.println(lin);
    setLeftGripper(true);
    setRightGripper(true);
//    rightGripper.write(CLOSE_DEG_R);
//    leftGripper.write(CLOSE_DEG_L);
    //      actuateRightGripper(true); //h - grip
    delay(500);
  }
  else if (_receivedByte == 105) {
    measureDistanceFiltered();
    String lin = "F:" + String(frontDistance) + " R:" + String(rightDistance) + " L:" + String(leftDistance) + "   ";
    Serial.println(lin);
    setLeftGripper(false);
    setRightGripper(false);
//    rightGripper.write(OPEN_DEG_R);
//    leftGripper.write(OPEN_DEG_L);
    //      actuateRightGripper(false); //i - ungrip
    delay(500);
  }
  else {
    return;
  }

}
