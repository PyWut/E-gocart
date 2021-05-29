import RPi.GPIO as GPIO
from time import sleep
import os

class Motor:
    def __init__(self, PWM, DIR):
        self.PWM = PWM
        self.DIR = DIR
        GPIO.setup(self.DIR, GPIO.OUT)
        GPIO.setup(self.PWM, GPIO.OUT)
        self.speed = GPIO.PWM(self.PWM, 50)
        self.speed.start(0)
        self.max_speed = 100
    
    def ChangeSpeed(self, area):
        # area to duty
        duty = (lambda x: 0.0000000031 * x**2 - 0.001305*x + 130.7189)(area)
        if duty > self.max_speed:
            duty = self.max_speed
        if duty > 100:
            duty = 100
        elif duty < 10:
            duty = 0
        self.speed.ChangeDutyCycle(duty)
    
    def get_surface_area(self, box):
        # trk.box layout [y_up, x_left, y_down, x_right]
        return abs((box[0] - box[2]) * (box[3] - box[1]))
    
    def update(self, box, pos):
        self.ChangeMaxSpeed(abs(pos))
        self.ChangeSpeed(self.get_surface_area(box))
    
    def ChangeMaxSpeed(self, pos):
        self.max_speed = (lambda x: 516.8484 * pos**(-1.8255))(pos) if pos >= 2.5 else 100
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
if __name__ == "__main__":
    DIR = 12
    PWM = 16

    GPIO.setmode(GPIO.BOARD)
    motor = Motor(PWM, DIR)

    for i in range(6):
        motor.ChangeMaxSpeed(i)
        print(i, motor.max_speed)
    
