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
  //  Serial.print("Received: ");
  //  Serial.println(_receivedByte, DEC);
  ResetEncoders();
  
  if (_receivedByte == 119) {
    LOCOMOTION_ACTIVE = true;
    GoForward45(10); //w - forward - 8
//    delay(LARGE_STEP);
    //    int count = 0;
    //    int last = digitalRead(8);
    //    Serial.println(last);
    //    while (count <= 20) {
    //      int current = digitalRead(8);
    //      Serial.println(current);
    //      if (current != last) {
    //        last = current;
    //        count += 1;
    //        Serial.println(count);
    //      }
    //    }
  }
  else if (_receivedByte == 115) {
    Stop(); //s - stop - 5
//    delay(LARGE_STEP);
  }
  else if (_receivedByte == 97) {
    LOCOMOTION_ACTIVE = true;
    GoLeft45(10); //a - left - 4
//    delay(LARGE_STEP);
  }
  else if (_receivedByte == 100) {
    LOCOMOTION_ACTIVE = true;
    GoRight45(10); //d - right - 6
//    delay(LARGE_STEP);
  }
  else if (_receivedByte == 122) {
    LOCOMOTION_ACTIVE = true;
    GoBackward45(10); //z - back - 2
//    delay(LARGE_STEP);
  }
  else if (_receivedByte == 113) {
    LOCOMOTION_ACTIVE = true;
    TurnLeft45(90); //q - turn left
//    delay(TURN_90_STEP);
  }
  else if (_receivedByte == 101) {
    LOCOMOTION_ACTIVE = true;
    TurnRight45(90); //e - turn right
//    delay(TURN_90_STEP);
  }
  else if (_receivedByte == 103) {
//        liftBobbins();
//        Serial.println("lifting bobbins");
    actuateLeftGripper(true); //g - grip
    delay(500);
  }
  else if (_receivedByte == 117) {
//        lowerBobbins();
//        Serial.println("lowering bobbins");
    actuateLeftGripper(false); //u - ungrip
    delay(500);
  }
  else if (_receivedByte == 104) {
    actuateRightGripper(true); //h - grip
    delay(500);
  }
  else if (_receivedByte == 105) {
    actuateRightGripper(false); //i - ungrip
    delay(500);
  }
  else if (_receivedByte == 39) {
    LOCOMOTION_ACTIVE = true;
    GoForward45(10); // ' - forward tiny bit
    delay(TINY_STEP);
  }
  else {
    return;
  }
  // LARGE_STEP corresponds to approx 5cm movement with 250ms pulse
//  PrintEncoderInfo();
//  SetMotors(5);
  
}
