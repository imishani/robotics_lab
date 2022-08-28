'''
# Aruco detection example https://github.com/leeyunjai/aruco-opencv
# github token: 6bb9b41b1051f7d5ac1beb38e5611aa32bd638d9
for camera internal settings :
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-ctrls
v4l2-ctl --set-ctrl=zoom_absolute=130
v4l2-ctl --set-ctrl=focus_absolute=15

to activate keyboard master with proper permision:
    sudo su --preserve-environment
    activate the environment: source activate
    to run the file: python main.py
'''
import time
# -- for processor functions:
import paho.mqtt.client as mqttClient
import json
import os
import numpy as np
# from MarkersDetection import get_pose



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


    def connect(self):
        self.client.connect(self.broker_address, port=self.port)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected to broker")
            self.Connected = True  # Signal connection
            client.subscribe(self.setup_topic, qos=0)
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
        print(f'Sending command: {command}')
        self.send_command(command)

    def control(self, x, y, theta):
        # car_loc, obstacle = get_pose() # from Aruco detection system

        while cntrlr.Connected and distance_to_goal > tolerance:
            # do every ~0.5 sec:
            # L, R = student_function(x, y, theta)

            cntrlr.motor_command(0.5, 0.5)
            time.sleep(1)


def student_function(x_g_g, y_g_g, theta_g_g):
    pass


if __name__ == "__main__":

    cntrlr = Controller(4) # input car ID
    cntrlr.connect()
    time.sleep(1)
    #cntrlr.motor_command(1., 1.)    # Don't move!
    cntrlr.motor_command(1., 1.)  # Don't move!

    # path = student_function()
    # for point in path:
    #     cntrlr.control(x_g, y_g, theta_g)
    time.sleep(2)
