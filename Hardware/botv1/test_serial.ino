void LocomotionTest(int _receivedByte) {
  if (!PID_DEBUG_POS && !PID_DEBUG_VEL) {
        Serial.print("Received: ");
        Serial.println(_receivedByte, DEC);
  }
  int step_size = 5; //cm

  if (_receivedByte == 119) {
    moveForwardDistance(step_size*2); //w - forward - 8
  }
  else if (_receivedByte == 115) {
    Stop(); //s - stop - 5
  }
  else if (_receivedByte == 97) {
    moveLeftDistance(step_size*2); //a - left - 4
  }
  else if (_receivedByte == 100) {
    moveRightDistance(step_size*2); //d - right - 6
  }
  else if (_receivedByte == 122) {
    StartLocomotion();
    GoBackward45(step_size*2); //z - back - 2
  }
  else if (_receivedByte == 113) {
    rotateDegree(90); //q - turn left
  }
  else if (_receivedByte == 101) {
    rotateDegree(-90); //e - turn right
  }
  //  else if (_receivedByte == 103) {
  ////        liftBobbins();
  ////        Serial.println("lifting bobbins");
  //    actuateLeftGripper(true); //g - grip
  //    delay(500);
  //  }
  //  else if (_receivedByte == 117) {
  ////        lowerBobbins();
  ////        Serial.println("lowering bobbins");
  //    actuateLeftGripper(false); //u - ungrip
  //    delay(500);
  //  }
  //  else if (_receivedByte == 104) {
  //    actuateRightGripper(true); //h - grip
  //    delay(500);
  //  }
  //  else if (_receivedByte == 105) {
  //    actuateRightGripper(false); //i - ungrip
  //    delay(500);
  //  }
  else if (_receivedByte == 39) {
    moveForwardDistance(step_size);  // ' - forward tiny bit
  }
  else {
    return;
  }

}
