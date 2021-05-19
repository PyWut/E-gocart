import RPi.GPIO as GPIO
import subprocess


SHUTDOWN = 37
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SHUTDOWN, GPIO.IN, GPIO.PUD_UP)

def shutdown(_):
	subprocess.call("sudo shutdown now", shell=True)

if __name__ == "__main__":
	GPIO.add_event_detect(SHUTDOWN, GPIO.FALLING, callback=shutdown)
	while True:
		pass
