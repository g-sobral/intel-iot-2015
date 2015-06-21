#!/usr/bin/env python

# Intel IoT Roadshow
# 19, 20 jun 2015 - Sao Paulo
# Author: Gabriel Sobral <gasan.sobral@gmail.com>


import time
import paho.mqtt.client as mqtt

class MqttConnection:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def start(self):
        self.client.connect("52.4.34.8", 1883, 60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

    def on_message(self, client, userdata, msg):
        print(msg.topic+" "+str(msg.payload))

    def on_subscribe(self, client, userdata, mid, granted_qos):
        print("Subscribed with result code "+str(rc))

    def publish(self, topic, payload, qos=0):
        self.client.publish(topic, payload, qos)

    def subscribe(self, topic, qos=0):
        self.client.subscribe(topic, qos)

    def add_callback(self, topic, callback):
        self.client.message_callback_add(topic, callback)


def receive_distance(client, userdata, message):
    print '*****************************************'
    print 'Oh, Thomas almost fall on the ground...'
    print 'distance from table to ground is:', message.payload
    print '*****************************************'

valid_commands = ['go', 'stop']
mqtt_conn = MqttConnection()
mqtt_conn.start()

mqtt_conn.add_callback('iotroadshow/thomas/distance', receive_distance)
mqtt_conn.subscribe('iotroadshow/thomas/distance')

time.sleep(1)

while True:
    command = raw_input('Input:')

    if command in valid_commands:
        print 'publishing command:', command
        mqtt_conn.publish('iotroadshow/thomas/command', command)
    else:
        print 'not a valid command:', command