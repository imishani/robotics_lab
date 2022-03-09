import paho.mqtt.client as mqtt

import sys
import threading
from .autogen.messages import Frame_pb2 as FramePb


class MQTTTransport():
    address = ('', 0)
    
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_publish = self._on_publish
        self.client.on_subscribe = self._in_subscribe
        # self.client.on_log = self._on_log
        self.connected = False

    def _on_message(self, client, userdata, msg):
        self.onFrameCallback(msg.payload)

    def _on_connect(self, client, userdata, flags, rc):
        self.client.subscribe("kinovafromserver")
        
    def _in_subscribe(self, client, userdata, flags, rc):
        self.connected = True

    def _on_log(self, mqttc, obj, level, string):
        print(string)

    def connect(self, host, port):
        self.client.connect(host, port, 60)
        
        rxThread = threading.Thread(target=self.client.loop_forever)
        rxThread.start()

        while self.connected is False:
            pass

    def send(self, payload):
        self.client.publish("kinovafromclient", payload, qos=2, retain=False)

    def _on_publish(self, client, userdata, flags):
        print("On publish completed")

    def registerOnFrameCallback(self, callback):
        self.onFrameCallback = callback
