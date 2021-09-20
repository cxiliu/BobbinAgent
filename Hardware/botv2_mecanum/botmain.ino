void loop() {
  // highest priority task 
  LoopPID();
  
  //  if (DistanceOn) {
  //    measureDistance();
  //    //  measureDistanceFront();
  //  }

  // COMMUNICATION
  if (Serial.available() > 0) {
          curString = Serial.readStringUntil('\n'); //process incoming byte as a string
          Serial.println(curString);
          //check if rot in string. then value is for rotation command
          if (curString.indexOf("rot") >= 0) {
            curString = curString.substring(4);
            rotVal = curString.toInt();
          }

          //check if dist in string. then value is for moveForward command
          else if (curString.indexOf("dist") >= 0) {
            curString = curString.substring(4);
            distVal = curString.toInt();
          }
          else if (curString == "w" || curString == "s" || curString == "z" || curString == "a" 
            || curString == "d" || curString == "q" || curString == "e"
            || curString == "x" || curString == "c" || curString == "h" || curString == "i" || curString == "g" || curString == "u"){
            incomingByte = curString.charAt(0);
            LocomotionTest(incomingByte);
          }
          else if (curString.indexOf("c1") >= 0){
            Cross1();
          }
          else if (curString.indexOf("c2") >= 0){
            Cross2();
          }
          else if (curString.indexOf("setd") >=0 ) {
            curString = curString.substring(4);
            SET_DISTANCE = curString.toInt();
            Serial.println("distance set to " +curString);
          }
          else if (curString.indexOf("seta") >=0 ) {
            curString = curString.substring(4);
            SET_ANGLE = curString.toInt();
            Serial.println("angle set to " +curString);
          }
          //else it is a normal bot instruction
          else {
            //Serial.print(curString);
            //Serial.println();
            curVal = curString.toInt(); //convert it to an int
            //Serial.print(curVal);
            //Serial.println();

            measureDistance();
            measureDistanceLeft();
            measureDistanceRight();

            if (curVal == 6) {          // keypress 6 sim for distance request 54
              String dPrintText = "distance/";
              currentMode = "Distance";
              String dPrintVal = dPrintText + frontDistance;
              Serial.print(dPrintVal);
              delay(1000);
            }

            else if (curVal == 12) {          // keypress 12 sim for LEFT GRIPPER distance request
              String dPrintText = "left/";
              String dPrintVal = dPrintText + leftDistance;
              Serial.print(dPrintVal);
              delay(1000);
            }

            else if (curVal == 11) {          // keypress 11 sim for RIGHT GRIPPER distance request
              String dPrintText = "right/";
              String dPrintVal = dPrintText + rightDistance;
              Serial.print(dPrintVal);
              delay(1000);
            }

            else if (curVal == 1) {          // keypress 1 sim for IDLE 49
              currentMode = "Idle";
              delay(200);
              Serial.print("status/idle");
            }

            else if (curVal == 2) {          // keypress 2 sim for rotate func. 50
              delay(1000);
              currentMode = "Rotate";
              Serial.print("status/rotating " + String(rotVal) + "deg");
              rotateDegree(rotVal);
              delay(2000);
              Serial.print("status/idle");

            }

            else if (curVal == 8) {          // keypress 3 sim for move adaptive distance func.
              delay(1000);
              Serial.print("status/approaching");
              //Serial.print("status/approaching " + String(distVal) + "cm");
              moveForwardDistance(15);
            }

            // RIGHT GRIPPER GRAB
            else if (curVal == 4) {          // keypress 4 sim for pickup func. 52
              currentMode = "GrabR";
              Serial.print("status/grabbingRIGHT");
              delay(1000);
              alignRightGripper(rotVal);
              delay(2000);

              approachRightGripper();
              delay(1000);

              pickupRight();
              delay(2000);
              Serial.print("status/idle");

            }

            // LEFT GRIPPER GRAB
            else if (curVal == 13) {
              currentMode = "GrabL";
              Serial.print("status/grabbingLEFT");
              delay(1000);
              alignLeftGripper(rotVal);
              delay(2000);

              approachLeftGripper(frontDistance);
              delay(1000);

              pickupLeft();
              delay(2000);
              Serial.print("status/idle");
            }


            else if (curVal == 5) {          // keypress 5 sim for twist func. 53
              Serial.print("status/twisting");
              currentMode = "Twist";
              twist();
              delay(2000); // performing action
              Serial.print("status/idle");
            }

            // RIGHT GRIPPER DROP
            else if (curVal == 7) {          // keypress 7 for drop 55
              Serial.print("status/dropping");
              currentMode = "Drop Off";
              delay(1000);
              //Serial.print("bobbins/2");
              dropoffRight();
              delay(2000); // performing action
              moveLeftDistance(10);
              Serial.print("status/idle");
            }

            // LEFT GRIPPER DROP
            else if (curVal == 14) {          // keypress 14 for drop 55
              Serial.print("status/dropping");
              delay(1000);
              //Serial.print("bobbins/2");
              dropoffLeft();
              delay(2000); // performing action
              moveRightDistance(10);
              Serial.print("status/idle");
            }

            else if (curVal == 3) {          // keypress 3 for approach preset distance 51
              Serial.print("status/approaching");
              moveForwardDistance(5);
            }
          }
  }


  // show info on lcd
  if (DisplayOn) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > 500) {
      writeData();
      previousMillis = currentMillis;
    }
  }

  //  Serial.println(digitalRead(FR_encoder));

  //delay(10);

}
