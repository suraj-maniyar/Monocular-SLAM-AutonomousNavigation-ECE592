from RPIO import PWM

class PanTilt:

    def __init__(self, pitch_BCM, roll_BCM):

        self.servo = PWM.Servo()
    
        self.pitch_BCM = pitch_BCM
        self.roll_BCM = roll_BCM

        self.period = 20

        self.pitch_pwm = 10000
        self.roll_pwm = 10000
        
        self.servo.set_servo(pitch_BCM, self.pitch_pwm)
        self.servo.set_servo(roll_BCM, self.roll_pwm)

    def pitch(duty_cycle):
        if(duty_cycle == 0):
            self.servo.set_servo(pitch_BCM, 0)
        else:
            self.servo.set_servo(pitch_BCM, duty_cycle)
    
    def roll(duty_cycle):
        if(duty_cycle == 0):
            self.servo.set_servo(roll_BCM, 0)
        else:
            self.servo.set_servo(roll_BCM, duty_cycle)

"""
import RPi.GPIO as GPIO


class PanTilt:

    def __init__(self, pitch_pwm, roll_pwm):
    
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pitch_pwm, GPIO.OUT)
        GPIO.setup(roll_pwm, GPIO.OUT)
    
        self.pitch_pwm = pitch_pwm
        self.roll_pwm = roll_pwm
        
        self.pitch_servo = GPIO.PWM(pitch_pwm, 50)
        self.roll_servo = GPIO.PWM(roll_pwm, 50)
        
        self.pitch_servo.start(7.5)
        self.roll_servo.start(7.5)

    def pitch(duty_cycle):
        if(duty_cycle == 0):
            self.pitch_servo.stop()
        else:
            self.pitch_servo.ChangeDutyCycle(duty_cycle)
    
    def roll(duty_cycle):
        if(duty_cycle == 0):
            self.roll_servo.stop()
        else:
            self.roll_servo.ChangeDutyCycle(duty_cycle)
"""
    
if __name__ == "__main__":
    s = PanTilt(23, 24)
    try:
        p_val = 10000
        r_val = 10000
        val = '1'
        while val != 'p':
            val = raw_input("Enter key: ")
            if val == 'a':
                p_val += 1000
            elif val == 'q':
                p_val -= 1000
            elif val == 'e':
                r_val += 1000
            elif val == 'd':
                r_val -= 1000
                
            print("Pitch = " + str(p_val))
            print("Roll = " + str (r_val))
    except:
        pass
    finally:
        s.servo.stop_servo(23)
        s.servo.stop_servo(24)
