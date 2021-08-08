void CommunicationIntegrationTest(int _receivedByte) {
  //  Serial.print("Received: ");
  //  Serial.println(_receivedByte, DEC);
  if (_receivedByte == 116) {
    twist(); //t - twist
  }
  else if (_receivedByte == 112) {
    pickupRight(); //p - pickup
  }
  else if (_receivedByte == 100) {
    dropoffRight(); //d - dropoff
  }
  else if (_receivedByte == 108) {
    rotateDegree(90); //l - turn left 90
  }
  else if (_receivedByte == 114) {
    rotateDegree(-90); //r - turn right 90
  }
  else if (_receivedByte == 110) {
    moveForwardDistance(10); //n - nudge forward 10 cm
  }
  else {
    return;
  }
}

void LocomotionTest(int _receivedByte) {
  if (!PID_DEBUG_POS && !PID_DEBUG_VEL) {
    //    Serial.print("Received: ");
    //    Serial.println(_receivedByte, DEC);
    Serial.println();
  }
  int step_size = 5; //cm

  if (_receivedByte == 119) {
    StartLocomotion();
    GoForward45(step_size); //w - forward - 8
  }
  else if (_receivedByte == 115) {
    Stop(); //s - stop - 5
  }
  else if (_receivedByte == 97) {
    StartLocomotion();
    GoLeft45(step_size * 2); //a - left - 4
  }
  else if (_receivedByte == 100) {
    StartLocomotion();
    GoRight45(step_size * 2); //d - right - 6
  }
  else if (_receivedByte == 122) {
    StartLocomotion();
    GoBackward45(step_size); //z - back - 2
  }
  else if (_receivedByte == 113) {
    StartLocomotion();
    TurnLeft45(90); //q - turn left
  }
  else if (_receivedByte == 101) {
    StartLocomotion();
    TurnRight45(90); //e - turn right
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
    StartLocomotion();
    GoForward45(step_size); // ' - forward tiny bit
    delay(TINY_STEP);
  }
  else {
    return;
  }

}
