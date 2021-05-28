void loop() {

  if (LOCOMOTION_ACTIVE) {
    MotorPID();
  }

  if (DistanceOn) {
    measureDistance();
    //  measureDistanceFront();
  }

  //  Serial.println(analogRead(STALL_DETECTION));

  // listen for joystick pressw
  int toggle = digitalRead(sw);
  if (toggle == 0) {
    pressed = true;
  }
  else {
    if (pressed == true) {
      pressed = false;

      // either use joystick to toggle manual / auto
      autoMode = !autoMode;
      if (autoMode) {
        currentMode = "auto";
      } else {
        Stop();
        currentMode = "manual";
      }
    }
  }

  // locomotion
  //  if (autoMode) {
  //    Locomote();
  //  } else {
  //    // listen for joystick pushes
  //    joystickCommand = getJoystickDirection();
  //    if (joystickCommand != lastJoystickCommand) {
  //      lastJoystickCommand = joystickCommand;
  //      if (joystickCommand != 5) {
  //        int on = 30;
  //        int off = 20;
  //        // six cycles
  //        for (int i = 0; i < 6; i++) {
  //          SetMotors(joystickCommand);
  //          delay(on);
  //          SetMotors(5);
  //          delay(off);
  //        }
  //        delay(500);
  //      }
  //      else {
  //        SetMotors(joystickCommand );
  //      }
  //    }
  //  }

  // COMMUNICATION
  if (Serial.available() > 0) {
    // send data only if you receive data
    if (!COMMUNICATION_ENABLED) {
      incomingByte = Serial.read();
      //    Serial.print("Received: ");
      //    Serial.println(incomingByte, DEC);
      LocomotionTest(incomingByte);
      //    CommunicationIntegrationTest(incomingByte);
    } else {

      //currentByte = Serial.read();      // read the incoming byte
      curString = Serial.readStringUntil('\n'); //process incoming byte as a string

      //check if rot in string. then value is for rotation command
      if (curString.indexOf("rot") >= 0) {
        curString = curString.substring(4);
        rotVal = curString.toInt();
      }

      //else it is a normal bot instruction
      else {
        curVal = curString.toInt(); //convert it to an int
        measureDistance();

        if (curVal == 6) {          // keypress 6 sim for distance request 54
          String dPrintText = "distance/";
          String dPrintVal = dPrintText + frontDistance;
          Serial.print(dPrintVal);
          delay(1000);
        }

        else if (curVal == 1) {          // keypress 1 sim for IDLE 49
          delay(200);
          Serial.print("status/idle");
        }

        else if (curVal == 2) {          // keypress 2 sim for rotate func. 50
          delay(1000);
          Serial.print("status/rotating " + String(rotVal) + "deg");
          rotateDegree(rotVal);
          delay(2000); // performing rotation
          Serial.print("status/idle");
        }

        else if (curVal == 3) {          // keypress 3 sim for move func. 51

          Serial.print("status/approaching");
          moveForwardDistance(5);
        }

        else if (curVal == 4) {          // keypress 4 sim for pickup func. 52
          Serial.print("status/grabbing");
          delay(1000);
          Serial.print("bobbins/1"); // update counter
          pickup();
          //delay(2000); // performing grab action

          delay(2000);
          Serial.print("status/idle");
        }

        else if (curVal == 5) {          // keypress 5 sim for twist func. 53
          Serial.print("status/twisting");
          twist();
          delay(2000);
          Serial.print("status/idle");
        }

        else if (curVal == 7) {          // keypress 7 for drop 55
          Serial.print("status/dropping");
          delay(1000);
          Serial.print("bobbins/2");
          dropoff();
          //delay(2000);

          delay(2000);
          Serial.print("status/idle");
        }

        //else{
        //continue
        //Serial.print("status/NONE" + curString);
        //allOn(redLED, bluLED, greenLED, yellowLED);
        //}
      }
    }
  }


  // show info on lcd
  if (DisplayOn) {
    writeData();
  }

  //  Serial.println(digitalRead(FR_encoder));

  delay(10);

}
