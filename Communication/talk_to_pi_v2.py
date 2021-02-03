import paho.mqtt.client as mqttc
import logging

print('mqtt imported successfully')

logging.basicConfig(level=logging.DEBUG)


#HOST = "192.168.1.254" # raspberrypi
HOST = "raspberrypi"
PORT = 1883
TOPIC = "ESP32/rx"

def on_connect(client, userdata, flags, rc):
    print("Connected with result code {}".format(rc))

def on_disconnect(client, userdata, rc):
    print("Client was disconnected")

def on_message(client, userdata, message):
    print("Message recevied: "+message.payload.decode())
    #print(type(message))

def on_message_red(client, userdata, message):
    print("RED LIGHT" +message.payload.decode("utf-8"))
    redData = message.payload.decode("utf-8")
    if 'on' in redData:
        print("RED LIGHT ON!")
    elif 'off' in redData:
        print("RED LIGHT OFF!")

def on_message_blue(client, userdata, message):
    print("BLUE LIGHT" +message.payload.decode("utf-8"))
    blueData = message.payload.decode("utf-8")
    if 'on' in blueData:
        print("BLUE LIGHT ON!")
    elif 'off' in blueData:
        print("BLUE LIGHT OFF!")

def main():
    client = mqttc.Client() # pretty sure this only works with the pubsublibrary on MQTTv3
    client.enable_logger(logging.getLogger())
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.connect(HOST, PORT)
    client.loop_start() # now the client keeps checking on subbed msgs
    logging.info("Loop has started...")

    #client.subscribe("ESP32/rx") # #(hashtag) subscribes to all topics
    client.subscribe("ESP32/#")
    client.message_callback_add("ESP32/pub/red", on_message_red)
    client.message_callback_add("ESP32/pub/blue", on_message_blue)

    while True:
        topublish = input() # takes keyboard input for publishing 
        client.publish(TOPIC,topublish, qos=2, retain=False)


# run if the same name
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt: # hit ctrl+c to close the program
        logging.info("closing from keypress")

print("Communication Finished")
