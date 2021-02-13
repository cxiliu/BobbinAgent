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
