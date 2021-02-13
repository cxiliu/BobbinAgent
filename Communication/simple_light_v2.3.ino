// 
// Code to turn light on/off from recevied serial commands
// 

int redLED = 12;
int bluLED = 8;


int currentByte;  // variable to store incoming serial data
String content;

void setup() {
  Serial.begin(115200);
  pinMode(redLED, OUTPUT);
  pinMode(bluLED, OUTPUT);
}

void loop() {
  //on(bluLED);
  //delay(100);
  //off(bluLED);
  //on(redLED);
  //delay(200);
  //off(redLED);
  if (Serial.available()>0){          // send data only if you receive data
    currentByte = Serial.read();      // read the incoming byte



    /*
    // Testing Serial.read v. Serial.write
    // read returns the byte sent as an int
    // write returns the actual msg
    Serial.println();
    Serial.write(currentByte);

    // read the serial data as a string, do something depending on what topic is present
    content = Serial.readStringUntil('\n');
    Serial.print(content);
    if (content.indexOf("red") >= 0){
      Serial.println();
      Serial.println("OKAY");
    }
    */
    
    
    //Serial.print("data received: ");
    //Serial.println(currentByte);

    if (currentByte == 49){           // keypress 1
      on(redLED);
      //Serial.println();
      Serial.print("/red/on");
    }

    if (currentByte == 50){           // keypress 2
      off(redLED);
      //Serial.println();
      Serial.print("/red/off");
    }

    if (currentByte == 51){           // keypress 3
      on(bluLED);
      //Serial.println();
      Serial.print("/blue/on");
    }
    
    if (currentByte == 52){           // keypress 3
      off(bluLED);
      //Serial.println();
      Serial.print("/blue/off");
    }
  }
}

// My functions
// Light On
void on(int pinName){
  digitalWrite(pinName, HIGH);
}
// Light Off
void off(int pinName){
  digitalWrite(pinName, LOW);
}
// Check for topic
//void checkTopic(
