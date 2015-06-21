#!/usr/bin/env python

# Intel IoT Roadshow
# 19, 20 jun 2015 - Sao Paulo
# Author: Gabriel Sobral <gasan.sobral@gmail.com>


import time
from threading import Thread
import pyupm_grove as grove
import mraa
import signal
import sys
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


class Motors:
    def __init__(self, left_pin, right_pin):
        self.left = mraa.Pwm(left_pin)
        self.left.period_ms(20)
        self.left.enable(True)

        self.right = mraa.Pwm(right_pin)
        self.right.period_ms(20)
        self.right.enable(True)

    def go_backward(self):
        self.left.write(0.125)
        self.right.write(0.025)

    def go_forward(self):
        self.left.write(0.025)
        self.right.write(0.125)

    def stop(self):
        self.left.write(1)
        self.right.write(1)

    def turn_right(self):
        self.left.write(0.025)
        self.right.write(0.025)

    def turn_left(self):
        self.left.write(0.125)
        self.right.write(0.125)


class Hc_sr04:
    def __init__(self, echo_pin, trigger_pin):
        self.echo = mraa.Gpio(echo_pin)
        self.echo.dir(mraa.DIR_IN)

        self.trig = mraa.Gpio(trigger_pin)
        self.trig.dir(mraa.DIR_OUT)

    def get_distance(self):
        self.trig.write(0)
        time.sleep(0.000002)
        self.trig.write(1)
        time.sleep(0.000010)
        self.trig.write(0)

        while not self.echo.read():
            pass
        start_time = time.time()
        while self.echo.read():
            pass

        duration = time.time() - start_time
        distance = 340*100*duration/2
        return int(distance)


class Robot:
    def __init__(self):
        self.motors = Motors(left_pin=5, right_pin=6)
        self.ultrasound = Hc_sr04(echo_pin=12, trigger_pin=13)
        self.touch = grove.GroveButton(7)
        self.buz = mraa.Gpio(4)
        self.buz.dir(mraa.DIR_OUT)

        self.led_red = mraa.Gpio(2)
        self.led_red.dir(mraa.DIR_OUT)

        self.led_blue = mraa.Gpio(3)
        self.led_blue.dir(mraa.DIR_OUT)

        self.blink = False
        self.blink_thread = Thread(target=self.thread_blink_leds, args=(1,))
        self.blink_thread.start()

        self.play = False
        self.play_thread = Thread(target=self.thread_play_melody, args=(2, ))
        self.play_thread.start()

        # inicialmente parado
        self.stop()

    def turn_left(self):
        self.motors.turn_left()

    def turn_right(self):
        self.motors.turn_right()

    def stop(self):
        self.motors.stop()

    def go_forward(self):
        self.motors.go_forward()

    def go_backward(self):
        self.motors.go_backward()

    def get_distance(self):
        return self.ultrasound.get_distance()

    def get_touch(self):
        return self.touch.value()

    def blink_leds(self, state):
        self.blink = state

    def thread_blink_leds(self, thread_name):
        while True:
            if self.blink:
                self.led_blue.write(1)
                self.led_red.write(0)
                time.sleep(0.5)
                self.led_blue.write(0)
                self.led_red.write(1)
                time.sleep(0.5)
            else:
                self.led_blue.write(0)
                self.led_red.write(0)

    def play_melody(self, state):
        self.play = state

    def thread_play_melody(self, thread_name):
        while True:
            if self.play:
                self.buz.write(1)
                time.sleep(0.1)
                self.buz.write(0)
                time.sleep(0.1)
            else:
                self.buz.write(0)

    def go_crazy(self):
        distance = self.get_distance()
        print "distance:", distance

        self.go_forward()
        self.play_melody(True)
        self.blink_leds(True)

        if distance > 10:
            self.stop()
            payload = str(distance) + ' cm'
            mqtt_conn.publish('iotroadshow/thomas/distance', payload)
            self.led_blue.write(1)
            self.led_red.write(1)            
            time.sleep(1)
            self.go_backward()
            time.sleep(2)
            self.turn_right()
            time.sleep(1.5)

            self.led_blue.write(0)
            self.led_red.write(0)

    def stop_all(self):
        self.stop()
        self.play_melody(False)
        self.blink_leds(False)
        self.buz.write(0)
        self.led_red.write(0)
        self.led_blue.write(0)

command = 'stop'
def receive_command(client, userdata, message):
    global command
    command = message.payload
    print 'received command:', message.payload

thomas = Robot()
status = False
flag_touch = False

mqtt_conn = MqttConnection()
mqtt_conn.start()

mqtt_conn.add_callback('iotroadshow/thomas/command', receive_command)
mqtt_conn.subscribe('iotroadshow/thomas/command')

def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    print 'Bye bye Thomas!'
    thomas.stop_all()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

while True:

    if thomas.get_touch():
        if not flag_touch:
            flag_touch = True
            status = not status
    else:
        flag_touch = False

    if command == 'go' or status:
        thomas.go_crazy()
    elif command == 'stop' or not status:
        thomas.stop_all()

    time.sleep(0.2)
