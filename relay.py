import RPi.GPIO as GPIO
from time import sleep

class DualRelay:
	def __init__(self, relay1, relay2, max_turning_time, delay):
		self.r1 = relay1
		self.r2 = relay2
		self.max_turning_time = max_turning_time
		self.delay = delay
		GPIO.setup(self.r1, GPIO.OUT, initial=GPIO.HIGH)
		GPIO.setup(self.r2, GPIO.OUT, initial=GPIO.HIGH)
	def left(self, time):
		GPIO.output(self.r1, False)
		sleep(time)
		GPIO.output(self.r1, True)
	def right(self, time):
		GPIO.output(self.r2, False)
		sleep(time)
		GPIO.output(self.r2, True)

if __name__ == "__main__":
	relay1 = 29
	relay2 = 31
	"""
	DIR = 12
	PWM = 16
	"""
	GPIO.setmode(GPIO.BOARD)
	steering = DualRelay(relay1, relay2, None, None)
	"""
	GPIO.setup(DIR, GPIO.OUT)
	GPIO.setup(PWM, GPIO.OUT)
	speed = GPIO.PWM(PWM, 50)
	GPIO.output(DIR, True)
	speed.start(0)
	speed.ChangeDutyCycle(20)
	"""
	#steering.right(6)
	sleep(1)
	#speed.ChangeDutyCycle(50)
	steering.left(2)
	sleep(1)
	#speed.ChangeDutyCycle(0)
	#speed.stop()
	GPIO.cleanup()
