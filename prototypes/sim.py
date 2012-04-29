#This will be rough simulation of the quadcopter sensor and thrust systems, no control system modeling at 
#this time
import math
import random
import pdb

gravity = 9.8 #the usual m/s^2

class ThreeState:
        def __init__(self):
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

class physical_state:
        def __init__(self):
                #These are the absoulte positions
                self.position = ThreeState()
                self.velocity = ThreeState()
                self.acceleration = ThreeState()
                #these are the rotations
                self.angle = ThreeState()
                self.angle_velocity = ThreeState()
                self.angle_accl = ThreeState()
                self.prop_speed = [0,0,0,0] #This is in rads/sec
        

class quad_state:
        def __init__(self):
                self.roll = 0.0
                self.pitch = 0.0
                self.yaw = 0.0
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

#this is the class that models the acceleromter
#it will model the adxl345
class accel_model:
        #0-1023 is the range of value
        #2g scale
        #noise on z axis =< 1.5lsb
        #noise on x,y axis =<1lsb
        def __init__(self):
                self.x_noise = 1
                self.y_noise = 1
                self.z_noise = 1.5
                self.fullscale = 1023

        #Will return a three state with the noise values
        def noise(self):
                #pdb.set_trace()
                state = ThreeState ()
                state.x = random.gauss(0,self.x_noise)
                state.y = random.gauss(0,self.y_noise)
                state.z = random.gauss(0,self.z_noise)
                return state

        #Returns a three state with the raw (0-1023 values)
        def measure2(self, phys_state):
                state = accel.noise(self)
                state.x += ((gravity*math.cos((math.pi/2)-phys_state.roll))*26.122)+512
                state.y += ((gravity*math.cos((math.pi/2)-phys_state.pitch))*26.122)+512
                #state[2] += gravity*52.24489 #not sure this is right, the range is +-2g, not 2g,
                state.z += ((gravity*math.cos(phys_state.roll))*26.122)+512
                return state

        def measure(self, phys_state):
                state = accel.noise(self)
                state.x += (-math.sin(phys_state.x)*gravity)*26.122+512
                state.y += math.cos(phys_state.x)*math.sin(phys_state.y)*gravity*26.122+512
                state.z += math.cos(phys_state.x)*math.cos(phys_state.y)*gravity*26.122+512
                return state

        def scale(self):
                return self.fullscale

#xyz should be changed to psi, phi, theta
class gyro_model:
        def __init__(self):
                self.current = ThreeState()
                self.previous = ThreeState()

        def measure(self):
                return self.current
        
#xyz should be changed to psi, phi, theta, I think, look into convetions
class mag_model():
        def __init__(self):
                self.heading = ThreeState()

        def measure(self):
                return self.heading

class distance_model():
        def __init__(self):
                self.reading = 0

        def measure(self):
                return self.reading

class imu_model:
        def __init__(self):
                self.accel = accel_model()
                self.gyro = gyro_model()
                self.mag = mag_model()
                self.alt = distance_model()

class QuadCopter:
	#units are grams
	mass = 1000
	#thrust_rpm = .1225 #this is thrust per rpm
	num_motors = 4
	max_motor_speed = 10212
	thrust_rpm = 7.6972/max_motor_speed # This is in newtons

	#since python sin uses radins, these are in radians
	#This is the physical state of the airframe
	phys_state = quad_state()

	setpoint = quad_state()

        #this is the state of the airframe reported by the imu
	imu_state = quad_state()
		
	def thrust(motor_percent, self):
		z_thrust = 0.0
		x_thrust = 0.0
		y_thrust = 0.0
		for p in motor_percent:
			z_thrust += p*self.max_motor_speed*self.thrust_rpm

		x_thrust += (motor_percent[0]*self.max_motor_speed*self.thrust_rpm)*math.sin(self.phys_state.roll)
		x_thrust += (motor_percent[2]*self.max_motor_speed*self.thrust_rpm)*math.sin(self.phys_state.roll)
		y_thrust += (motor_percent[1]*self.max_motor_speed*self.thrust_rpm)*math.sin(self.phys_state.pitch)
		y_thrust += (motor_percent[3]*self.max_motor_speed*self.thrust_rpm)*math.sin(self.phys_state.pitch)
			
		return [z_thrust,x_thrust,y_thrust]


x = [.1,.2,.3,.4,.5,.6,.7,.8,.9,1]

for p in x:
        z = [p,p,p,p]
        print (QuadCopter.thrust(z, QuadCopter))

QuadCopter.phys_state.roll = .2
print('------------------------------------------------')
print(QuadCopter.phys_state.roll)
for p in x:
        z = [p,p,p,p]
        print (QuadCopter.thrust(z, QuadCopter))

##print('------------------------------------------------')
##a= accel()
##QuadCopter.phys_state.roll = 0
##for i in range(20):
##        f = a.measure(QuadCopter.phys_state)
##        print (math.sqrt((f.x-512)*(f.x-512)+(f.z-512)*(f.z-512)))
##        print(f.x, f.y, f.z)
