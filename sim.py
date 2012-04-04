#This will be rough simulation of the quadcopter sensor and thrust systems, no control system modeling at 
#this time
import math
import random

gravity = 9.8 #the usual m/s^2

class ThreeState:
        def __init__(self):
                self.x = 0.0
                self.y = 0.0
                self.y = 0.0

#this is the class that models the acceleromter
#it will model the adxl345
class accel:
        #0-1023 is the range of value
        #2g scale
        #noise on z axis =< 1.5lsb
        #noise on x,y axis =<1lsb

        #Will return a three state with the noise values
        def noise():
                state = ThreeState ()
                state.x = random.gauss(0,1)
                state.y = random.gauss(0,1)
                state.z = random.gauss(0,1.5)
                return [state.x,state.y,state.z]

        #Returns a three state with the raw (0-1023 values)
        def measure(phys_state):
                state = accel.noise()
                state[0] += 0.0
                state[1] += 0.0
                #state[2] += gravity*52.24489 #not sure this is right, the range is +-2g, not 2g,
                state[2] += (gravity*26.122)+512
                return state

class QuadCopter:
	#units are grams
	mass = 1000
	thrust_rpm = .1225 #this is thrust per rpm
	num_motors = 4
	max_motor_speed = 10212

	#since python sin uses radins, these are in radians
	#This is the physical state of the airframe
	class phys_state:
		roll = 0.0
		pitch = 0.0
		yaw = 0.0
		x = 0.0
		y = 0.0
		z = 0.0

        #this is the state of the airframe reported by the imu
	class imu_state:
		roll = 0.0
		pitch = 0.0
		yaw = 0.0
		x = 0.0
		y = 0.0
		z = 0.0
		
	def thrust(motor_percent, self):
		z_thrust = 0.0
		x_thrust = 0.0
		y_thrust = 0.0
		for p in motor_percent:
			z_thrust += p*self.max_motor_speed*self.thrust_rpm

                #Need to check if this the correct math, I'm not sure it is
		x_thrust += (motor_percent[0]*self.max_motor_speed*self.thrust_rpm)*math.sin(self.phys_state.roll)
		x_thrust += (motor_percent[2]*self.max_motor_speed*self.thrust_rpm)*math.sin(self.phys_state.roll)
		y_thrust += (motor_percent[1]*self.max_motor_speed*self.thrust_rpm)*math.sin(self.phys_state.pitch)
		y_thrust += (motor_percent[3]*self.max_motor_speed*self.thrust_rpm)*math.sin(self.phys_state.pitch)
			
		return [z_thrust,x_thrust,y_thrust]


x = [.1,.2,.3,.4,.5,.6,.7,.8,.9,1]

for p in x:
        z = [p,p,p,p]
        print (QuadCopter.thrust(z, QuadCopter))

QuadCopter.phys_state.roll = .1
print('------------------------------------------------')
print(QuadCopter.phys_state.roll)
for p in x:
        z = [p,p,p,p]
        print (QuadCopter.thrust(z, QuadCopter))

print('------------------------------------------------')
for i in range(100):
        print(accel.measure(QuadCopter.phys_state))
