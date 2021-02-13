#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "XXX";
const char* password = "XXX";

//const char* mqttServer = "broker.hivemq.com";
IPAddress broker(192,168,1,249);
const int mqttPort = 1883;

#define MQTT_USER "testuser"
#define MQTT_PASSWORD "test"
#define MQTT_SERIAL_RECEIVER_CH "ESP32/rx"
#define MQTT_SERIAL_PUBLISH_CH "ESP32/pub/red"
#define MQTT_SERIAL_PUBLISH_CH_TWO "ESP32/pub/blue"

//const char* MQTT_USER = "testuser";
//const char* MQTT_PASSWORD = "test";
//const char* MQTT_SERIAL_RECEIVER_CH = "/ESP32/test";
//const char* MQTT_SERIAL_PUBLISH_CH = "/ESP32/pub";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Serial.print("hello");
  Serial.setTimeout(500);
  setup_wifi();

  // change broker to mqttServer if using HiveMQ
  client.setServer(broker, mqttPort);
  client.setCallback(callback);
  reconnect();
  

}

void loop() {
  // put your main code here, to run repeatedly:
  client.loop();
  if(Serial.available()>0){
    char mun[501];          //char buffer to store the bytes
    memset(mun,0,501);
    Serial.readBytesUntil('\n',mun,500);
    // send recieved data from arduino serial to pi broker
    // publishSerialData(mun);

    String content;
    //content = Serial.readStringUntil('\n');
    content = String(mun);
    if (content.indexOf("red") >= 0){
      publishSerialData(MQTT_SERIAL_PUBLISH_CH, mun);
    }
    if (content.indexOf("blue") >= 0){
      publishSerialData(MQTT_SERIAL_PUBLISH_CH_TWO, mun);
    }
  }
}

void setup_wifi(){
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      //Serial.println("connected");
      //print to where the ESP is publishing
      //Serial.println("Publishing to: ");
      //Serial.println(MQTT_SERIAL_PUBLISH_CH);
      
      //Once connected, publish an announcement...
      client.publish(MQTT_SERIAL_PUBLISH_CH, "hello red world");
      client.publish(MQTT_SERIAL_PUBLISH_CH_TWO, "hello blue world");
      //print to where the ESP is subscribed
      //Serial.println("Subscribing to ");
      //Serial.println(MQTT_SERIAL_RECEIVER_CH);
      
      // ... and resubscribe
      client.subscribe(MQTT_SERIAL_RECEIVER_CH);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte *payload, unsigned int length) {
    //Serial.println("-------new message from broker-----");
    //Serial.print("channel:");
    //Serial.println(topic);
    //Serial.print("data:"); 
    //Serial.write(payload, length);
    //Serial.println();

    char response = 0;
    for (int i = 0; i < length; i++){
      response += (char)payload[i];
    }
    //Serial.println("The char response (in DEC) is: ");
    //Serial.print(response, DEC);
    //Serial.println("The char response is: ");
    Serial.print(response); // sends the recieved command to the arduino via serial as a byte
                            // the byte is the sum of the char's received. 
    Serial.println();
    
    
}


void publishSerialData(char *channel, char *serialData){
  if (!client.connected()) {
    reconnect();
  }
  client.publish(channel, serialData);
}
