import pigpio
import time

SERVO = 12
microseconds = 2200
pwm = pigpio.pi()
pwm.set_mode(SERVO, pigpio.OUTPUT)
pwm.set_PWM_frequency(SERVO, 50)

# duty from 28000 to 124000 and center is 75000
def change_duty_cycle(duty_cycle):
    pwm.hardware_PWM(SERVO, 50, duty_cycle)

while True:
    try:
        change_duty_cycle(int(input("Duty Cycle: ")))
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(str(repr(e)))
        break

change_duty_cycle(0)
pwm.stop()
