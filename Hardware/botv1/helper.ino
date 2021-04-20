void CommunicationIntegrationTest(int _receivedByte) {
//  Serial.print("Received: ");
//  Serial.println(_receivedByte, DEC);
  if (_receivedByte == 116) {
    twist(); //t - twist
  }
  else if (_receivedByte == 112) {
    pickup(); //p - pickup
  }
  else if (_receivedByte == 100) {
    dropoff(); //d - dropoff
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
//  Serial.print("Received: ");
//  Serial.println(_receivedByte, DEC);
  if (_receivedByte == 119) {
    SetMotors(8); //w - forward - 8
    delay(LARGE_STEP);
  }
  else if (_receivedByte == 115) {
    SetMotors(5); //s - stop - 5
    delay(LARGE_STEP);
  }
  else if (_receivedByte == 97) {
    SetMotors(4); //a - left - 4
    delay(LARGE_STEP);
  }
  else if (_receivedByte == 100) {
    SetMotors(6); //d - right - 6
    delay(LARGE_STEP);
  }
  else if (_receivedByte == 122) {
    SetMotors(2); //z - back - 2
    delay(LARGE_STEP);
  }
  else if (_receivedByte == 113) {
    TurnLeft45(); //q - turn left
    delay(TURN_90_STEP);
  }
  else if (_receivedByte == 101) {
    TurnRight45(); //e - turn right
    delay(TURN_90_STEP);
  }
  else {
    return;
  }
  // LARGE_STEP corresponds to approx 5cm movement with 250ms pulse
  SetMotors(5);
}


// if it is super blocked (too tight to turn), go backward and nudge right
//  if ((frontDistance < COLLISION_THRESHOLD) || (rightDistance < COLLISION_THRESHOLD) || (leftDistance < COLLISION_THRESHOLD)) {
//    GoBackward();
//    delay(LARGE_STEP);
//    GoRight();
//    delay(SMALL_STEP);
//
//    // if the nudge is bad, try the other side
//    measureDistance();
//    if ((frontDistance < COLLISION_THRESHOLD) || (rightDistance < COLLISION_THRESHOLD) || (leftDistance < COLLISION_THRESHOLD)) {
//      GoLeft();
//      delay(LARGE_STEP);
//    } else {
//      PickSideAndTurn(SMALL_STEP);
//      return;
//    }
//
//    // if the nudge is still bad, just stop and wait for human
//    measureDistance();
//    if ((frontDistance < COLLISION_THRESHOLD) || (rightDistance < COLLISION_THRESHOLD) || (leftDistance < COLLISION_THRESHOLD)) {
//      Stop();
//      stuckCounter = STUCK_COUNT;
//      return;
//    } else {
//      PickSideAndTurn(SMALL_STEP);
//      return;
//    }
//  }



//  if (Serial.available() > 0) {
//    // read the incoming byte:
//    incomingByte = Serial.read();
//
//      //s 115
//      //d 100
//      //encode incoming bytes to bot instructions
//
//      if (incomingByte == 115){
//      }
//      // say what you got:
//    Serial.print("I received: ");
//    Serial.println(incomingByte, DEC);
//  }
