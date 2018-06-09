import pigpio
from BNO055 import BNO055
import time

servos = pigpio.pi()
bno = BNO055(address = 0x28)
if bno.begin() is not True:
	print "Error initializing device"
	exit()
time.sleep(1)
bno.setExternalCrystalUse(True)

for i in range(1,40):
	vector = bno.getVector(BNO055.VECTOR_EULER)
	time.sleep(0.1)
	print(vector)

print("done!")


try:
	while True:
	
		vector = bno.getVector(BNO055.VECTOR_EULER)
	
		roll = vector[2]
		pitch = vector[1]
		
		servo_pitch_val = int(1600+8.88*(pitch))
		servo_roll_val = int(1250 + 9.44 * roll + 0.0123 * roll**2)
		
		# print("Roll Val: %5d" % servo_roll_val)
		# print("Pitch Val %5d" % servo_pitch_val)
		
		servo_roll_val = min(servo_roll_val, 2250)
		servo_roll_val = max(servo_roll_val, 500)
		
		servo_pitch_val = min(servo_pitch_val, 2500)
		servo_pitch_val = max(servo_pitch_val, 800)
		
		print(vector)
	
		servos.set_servo_pulsewidth(24, servo_roll_val)
		servos.set_servo_pulsewidth(23, servo_pitch_val)
	
		time.sleep(0.05)

except:
	pass

servos.set_servo_pulsewidth(24, 0)
servos.set_servo_pulsewidth(23, 0)
