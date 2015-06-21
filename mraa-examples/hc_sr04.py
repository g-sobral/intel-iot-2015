#!/usr/bin/env python

# Author: Gabriel Sobral <gasan.sobral@gmail.com>

import mraa
import time

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



led_red = mraa.Gpio(8)
led_red.dir(mraa.DIR_OUT)

led_blue = mraa.Gpio(7)
led_blue.dir(mraa.DIR_OUT)

sensor = Hc_sr04(echo_pin=2, trigger_pin=3)

while True:
    distance = sensor.get_distance()
    print 'distance:', distance

    if distance > 30:
        led_blue.write(1)
        led_red.write(0)
    else:
        led_blue.write(0)
        led_red.write(1)

    time.sleep(0.1)
