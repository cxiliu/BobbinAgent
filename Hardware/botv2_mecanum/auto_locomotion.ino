//const int STUCK_COUNT = 64;
//
//bool IsStuck() {
//  if (abs(frontDistance - lastFrontDistance) < 3) {
//    stuckCounter += 1;
//  } else {
//    stuckCounter = 0;
//    lastFrontDistance = frontDistance;
//  }
//  if (stuckCounter >= STUCK_COUNT) {
//    return true;
//  } else {
//    return false;
//  }
//}

//void PickSideAndTurn(int count) {
//  if (leftDistance > rightDistance) {
//    TurnLeft();
//    delay(count);
//  } else {
//    TurnRight();
//    delay(count);
//  }
//}

//void Locomote() {
//  // if the bot is stuck, do not move until human moves it
//  if (IsStuck()) {
//    Stop();
//    return;
//  }
//
//  // collision avoidance
//  measureDistanceFiltered();
//
//  // if either one reading is below collision threshold, do something
//  if ((frontDistance < COLLISION_THRESHOLD) ||
//      (rightDistance < COLLISION_THRESHOLD) ||
//      (leftDistance < COLLISION_THRESHOLD)) {
//
//    // if both left and right sides are tight, go back and turn to more open side
//    if ((rightDistance < COLLISION_THRESHOLD)
//        && (leftDistance < COLLISION_THRESHOLD)) {
//      GoBackward();
//      delay(SMALL_STEP);
//      PickSideAndTurn(TINY_STEP);
//    } else {
//      // if only one side is tight, back up a bit and turn to the other side
//      if (rightDistance < COLLISION_THRESHOLD) {
//        GoBackward();
//        delay(TINY_STEP);
//        TurnLeft();
//        delay(SMALL_STEP);
//      }
//      else if (leftDistance < COLLISION_THRESHOLD) {
//        GoBackward();
//        delay(TINY_STEP);
//        TurnRight();
//        delay(SMALL_STEP);
//      }
//      // if only front is blocked
//      // the bot is correctly aligned with a bobbin! approach and grip
//      else {
//        // move forward or backward set distance
//        int distance = frontDistance - PICKING_DISTANCE;
//        if ( distance > 0) {
//          GoForward();
//          delay(TINY_STEP * distance);
//        } else if (distance < 0) {
//          GoBackward();
//          delay(-TINY_STEP * distance);
//        }
//        // if gripping is not successful, go back and turn a random side
//        if (!tryGrip()) {
//          //          GoForward();
//          //          delay(SMALL_STEP);
//          //          if (!tryGrip()) {
//          GoBackward();
//          delay(SMALL_STEP);
//          PickSideAndTurn(SMALL_STEP);
//        }
//      }
//    }
//  } else {
//    // if all clear, go forward
//    GoForward();
//    delay(100);
//  }
//
//
//  Stop();
//  delay(50);
//}
