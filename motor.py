import RPi.GPIO as GPIO
from time import sleep
import os
#sleep(10)

DIR = 18
PWM = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(PWM, GPIO.OUT)
speed = GPIO.PWM(PWM, 50)
GPIO.output(DIR, True)
speed.start(0)

"""
while True:
    try:
        inp = input("1: forward, 2: backwards, 3: speed ")
        if inp == "1":
            GPIO.output(DIR, False)
        elif inp == "2":
            GPIO.output(DIR, True)
        else:
            duty = int(input("speed: "))
            speed.ChangeDutyCycle(duty)
    except KeyboardInterrupt:
        break
speed.stop()
GPIO.cleanup()
"""

duty = 0
while duty < 100:
    duty += 10
    speed.ChangeDutyCycle(duty)
    sleep(0.1)
sleep(3)
while duty > 0:
    duty -= 10
    speed.ChangeDutyCycle(duty)
    sleep(0.1)

speed.stop()
GPIO.cleanup()
    
