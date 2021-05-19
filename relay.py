import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)
GPIO.setup(29, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(31, GPIO.OUT, initial=GPIO.HIGH)

if __name__ == "__main__":
	GPIO.output(31, False)
	print("pin is low")
	sleep(1)
	GPIO.output(31, True)
	print("pin is high")
	"""
	sleep(1)
	GPIO.output(29, False)
	print("pin is low")
	sleep(6)
	GPIO.output(29, True)
	print("pin is high")
	sleep(1)
	"""
	GPIO.cleanup()
