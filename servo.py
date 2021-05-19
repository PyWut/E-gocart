import pigpio

SERVO = 12
pwm = pigpio.pi()
pwm.set_mode(SERVO, pigpio.OUTPUT)


# duty from 28000 to 122000 and center is 77000
def change_duty(duty):
    pwm.hardware_PWM(SERVO, 50, duty)

if __name__ == "__main__":
    while True:
        try:
            change_duty(round(float((input("Duty Cycle: ")))))
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(str(repr(e)))
            break

    change_duty(0)
    pwm.stop()
