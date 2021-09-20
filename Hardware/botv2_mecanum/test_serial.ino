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
  else if (_receivedByte == 122) {
    StartLocomotion();
    GoRight45(SET_DISTANCE); //z - go right 45
  }
  else if (_receivedByte == 99) {
    StartLocomotion();
    GoLeft45(SET_DISTANCE); //c - go left 45
  }
    else if (_receivedByte == 103) {
          Serial.println("lifting bobbins");
          liftBobbins();
          Serial.println("done");
//      actuateLeftGripper(true); //g - grip
      delay(500);
    }
    else if (_receivedByte == 117) {
      Serial.println("lowering bobbins");
          lowerBobbins();
          Serial.println("done");
//      actuateLeftGripper(false); //u - ungrip
      delay(500);
    }
    else if (_receivedByte == 104) {
      leftGripper.write(CLOSE_DEG);
      rightGripper.write(180-CLOSE_DEG);
//      actuateRightGripper(true); //h - grip
      delay(500);
    }
    else if (_receivedByte == 105) {
      leftGripper.write(OPEN_DEG);
      rightGripper.write(180-OPEN_DEG);
//      actuateRightGripper(false); //i - ungrip
      delay(500);
    }
  else {
    return;
  }

}
