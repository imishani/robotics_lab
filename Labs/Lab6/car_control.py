"""
All right reserved to  Itamar Mishani and Osher Azulay
imishani@gmail.com, osherazulay@mail.tau.ac.il
"""

import numpy as np
import time

# -- for processor functions:
import paho.mqtt.client as mqttClient
import json


class Controller():
    def __init__(self, ID):
        self.broker_address = "localhost"
        self.port = 1883
        self.client = mqttClient.Client("controller", clean_session=True)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.robot_id = ID
        self.command_topic = f"command/{ID}"
        self.setup_topic = f"setup/{ID}"
        self.Connected = False
        self.command_max = 255
        self.command_min = -255
        self.communicate = False

    def connect(self):
        self.client.connect(self.broker_address, port=self.port)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected to broker")
            self.Connected = True  # Signal connection
            print("Subscribing to topic", self.setup_topic, client.subscribe(self.setup_topic, qos=0))
            time.sleep(1)
        else:
            print("Connection failed")


    def send_command(self, command):
        data = json.dumps(command)
        self.client.publish(topic=self.command_topic, payload=data, qos=0, retain=False)

    def on_message(self, client, userdata, msg):
        self.msg_setup(client, msg)

    def msg_setup(self, client, msg):
        id = str(msg.payload.decode("utf-8"))
        print(f'robot connected. ID: {id}')
        self.communicate = True

    def motor_command(self, L, R):
        """
        :param L: Left wheel command: Range=(-1, 1) when abs(L) is no movement and 0 is max
        :param R: Right wheel command: Range=(-1, 1) when abs(R) is no movement and 0 is max
        """
        L *= self.command_max
        R *= self.command_max
        L = np.clip(L, self.command_min, self.command_max)
        R = np.clip(R, self.command_min, self.command_max)
        command = {"L": int(L), "R": int(R)}
        # print(f'Sending command: {command}')
        self.send_command(command)