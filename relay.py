import RPi.GPIO as GPIO
import time
from random import randint

class DualRelay:
	"""
	Dual channel relay
	Reverse working relay:
	NC --> HIGH
	NO --> LOW
	"""
	def __init__(self, relay1, relay2, max_turning_time, delay):
		self.r1 = relay1
		self.r2 = relay2
		self.max_left = -max_turning_time
		self.max_right = max_turning_time
		self.max_turning_time = max_turning_time
		self.delay = delay
		self.track_pos = {"current_pos": 0, "last_turned_off": 0, "last_time_updated": 0, "old_time": 0}
		GPIO.setup(self.r1, GPIO.OUT, initial=GPIO.HIGH)
		GPIO.setup(self.r2, GPIO.OUT, initial=GPIO.HIGH)

	def left(self):
		if GPIO.input(self.r2):
			GPIO.output(self.r1, False)

	def right(self):
		if GPIO.input(self.r1):
			GPIO.output(self.r2, False)

	def stop(self):
		GPIO.output(self.r1, True)
		GPIO.output(self.r2, True)
		self.track_pos["last_turned_off"] = time.time()

	def check_pos(self):
		if not GPIO.input(self.r1):
			self.track_pos["current_pos"] = self.track_pos["current_pos"] - (time.time() - self.track_pos["old_time"]) if self.track_pos["old_time"] != 0 else self.track_pos["current_pos"]
			if self.track_pos["current_pos"] < self.max_left:
				self.stop() 
		elif not GPIO.input(self.r2):
			self.track_pos["current_pos"] = self.track_pos["current_pos"] + (time.time() - self.track_pos["old_time"]) if self.track_pos["old_time"] != 0 else self.track_pos["current_pos"]
			if self.track_pos["current_pos"] > self.max_right:
				self.stop()
		self.track_pos["old_time"] = time.time() 

	def to_center(self):
		if self.track_pos["current_pos"] < -0.5:
			self.right()
		elif self.track_pos["current_pos"] > 0.5:
			self.left()

	def update(self, x):
		self.check_pos()
		if 260 <= x <= 380:
			if -0.5 < self.track_pos["current_pos"] < 0.5:
				if not GPIO.input(self.r1) or not GPIO.input(self.r2):
					self.stop()
			else:
				self.to_center()
		elif x > 380 and self.track_pos["current_pos"] < self.max_right:
			if GPIO.input(self.r2):
				if time.time() - self.track_pos["last_turned_off"] > self.delay:
					self.right()
		elif x < 260:
			if GPIO.input(self.r1) and self.track_pos["current_pos"] > self.max_left:
				if time.time() - self.track_pos["last_turned_off"] > self.delay:
					self.left()


if __name__ == "__main__":
	relay1 = 29
	relay2 = 31

	GPIO.setmode(GPIO.BOARD)
	steering = DualRelay(relay1, relay2, max_turning_time=5, delay=1)
	"""
	steering.right()
	time.sleep(4)
	steering.stop()
	time.sleep(2)
	
	steering.left()
	time.sleep(4)
	
	steering.stop()
	time.sleep(2)
	steering.right()
	time.sleep(4)
	steering.stop()
	time.sleep(2)

	steering.left()
	time.sleep(4)
	"""
	steering.right()
	time.sleep(2)	
	steering.stop()
	GPIO.cleanup()
