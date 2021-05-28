// joystick inteFace
// press button once to start/stop
// use joystick to control direction

void SetMotors(int command) {
  ResetEncoders();
  //8 forward, 7, 9
  //2 reverse, 1, 3
  //4 left
  //6 right
  //5 stop
  if (command == 5) {
    Stop();
  }
  else if ((command == 8) || (command == 7) || (command == 9)) {
    GoForward45(10);
  }
  else if ((command == 3) || (command == 2) || (command == 1)) {
    GoBackward45(10);
  }
  else if (command == 4) {
    if (leftGripperClosed && rightGripperClosed) {
      TurnLeft45(90);
    }
    else {
      GoLeft45(10); // parallel movement
    }
  }
  else if (command == 6) {
    if (leftGripperClosed && rightGripperClosed) {
      TurnRight45(90);
    }
    else {
      GoRight45(10); // parallel movement
    }
  }
  else {
    Stop();
    Serial.print("out --- ");
    Serial.println(command);
  }
}

int getJoystickDirection() {
  jXval = map(analogRead(jX), 0, 1023, -2, 2);
  jYval = map(analogRead(jY), 0, 1023, -2, 2);

  int dir = 0;

  // JOYSTICK COMMANDS
  if (jXval > 0 && jYval == 0) { // FORWARD
    dir = 8;
    //dir = 'w';
    DirName = "WARP SPEED, ENGAGE!";
    //buf[2] = 37;
    //Serial.write(buf, 8);
    //releaseKey();

  } else if (jXval < -1 && jYval == -1) { // REVERSE
    //      } else if (jXval <0 && jYval == 0) { // REVERSE
    dir = 2;
    //dir = 's';
    DirName = "REVERSE";
    //buf[2] = 31;
    //Serial.write(buf, 8);
    //releaseKey();

  } else if (jYval < -1 && jXval == 0) { // ROTATE LEFT
    //      } else if (jYval < 0 && jXval == 0) { // ROTATE LEFT
    dir = 4;
    //dir = 'a';
    DirName = "ROTATE LEFT";
    //buf[2] = 33;
    //Serial.write(buf, 8);
    //releaseKey();

  } else if (jYval > 0 && jXval == 0) { // ROTATE RIGHT
    dir = 6;
    //dir = 'd';
    DirName = "ROTATE RIGHT";
    //buf[2] = 35;
    //Serial.write(buf, 8);
    //releaseKey();

  } else if (jYval == -1 && jXval == 0) { // STOP
    //      } else if (jYval == 0 && jXval == 0) { // STOP
    dir = 5;
    //dir = 'x';
    DirName = "STOP";
    //buf[2] = 34;
    //Serial.write(buf, 8);
    //releaseKey();

  } else if (jYval >= 1 && jXval < 0) { // REVERSE RIGHT
    dir = 3;
    //dir = 'c';
    DirName = "REVERSE RIGHT";

    //buf[2] = 32;
    //Serial.write(buf, 8);
    //releaseKey();

  } else if (jYval >= 1 && jXval > 0) {// FORWARD RIGHT
    dir = 9;
    //dir = 'e';
    DirName = "FORWARD RIGHT";

    //buf[2] = 38;
    //Serial.write(buf, 8);
    //releaseKey();

  } else if (jYval < -1 && jXval < -1) {// REVERSE LEFT
    //      } else if (jYval <= -1 && jXval <= -1) {// REVERSE LEFT
    dir = 1;
    //dir = "z";
    DirName = "REVERSE LEFT";
    //buf[2] = 30;
    //Serial.write(buf, 8);
    //releaseKey();

  } else if (jYval <= -1 && jXval >= 1) {// FORWARD LEFT
    dir = 7;
    //dir = 'q';
    DirName = "FORWARD LEFT";
    //buf[2] = 36;
    //Serial.write(buf, 8);
    //releaseKey();
  }
  else {
    dir = 5;
    DirName = "STOP";
  }
  Serial.print(jXval);
  Serial.print(jYval);
  Serial.println(DirName);
  return dir;
}
