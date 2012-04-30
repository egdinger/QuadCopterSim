from random import *
import numpy, math
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt1
#These are constants that need to be set for each quadcopter
mass = 1.2
CodX = 1.04 #Coefficient of Drag on the x axis
CodY = 1.04 #Coefficient of Drag on the y axis
CodZ = 1.04 #Coefficient of Drag on the z axis
Ax = .25 #Area of quadcopter on the x axis, probably the same as the y axis
Ay = .25 #Area of quadcopter on the y axis
Az = .33 #Area of quadcopter on the z axis
arm_length = .254 # Taken from the Gregs pid.py, may not be correct for the current airframe
Ix = 1 #Inertia on the x axis
Iy = 1 #Inertia on the y axis
Iz = 1 #Inertia on the z axis
motor_max_thrust = 7.6972 #thrust in newtons of one motor
motor_max_rpm = 10212 #The max rpm of the motors, note in this case it's over the max rpm of the props 
motor_num = 4 #The number of motors on the quadcopter

#These are constants that are set for the environment
p = 1.225 #density of air
g = 9.81 #Gravity
delta_t = 1 #in 100th's of a second
inverse_delta = 100
run_time = 5 * inverse_delta

#These are sensor constants
adxl345_mu = 0
adxl345_sigma = 1 #note not right!
sonar_mu = 0
sonar_sigma = 1 #note not right!
itg3200_mu = 0
itg3200_sigma = 1 #not not right!

class ThreeState:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
        def __str__(self):
            return "%s, %s, %s" % (self.x, self.y, self.z)

#physical_state will be used to represent the quadcopter in the world frame
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
        def __str__(self):
            lin = "Linear\npos:" + str(self.position.x) +"," + str(self.position.y) +"," + str(self.position.z) + "\nvel:" + str(self.velocity.x) +"," + str(self.velocity.y) +"," + str(self.velocity.z) + "\nacl:" +str(self.acceleration.x) +"," +str(self.acceleration.y) +"," +str(self.acceleration.z)
            angular = "Angular\npos:" + str(self.angle.x) +"," + str(self.angle.y) +"," + str(self.angle.z) + "\nvel:" + str(self.angle_velocity.x) +"," + str(self.angle_velocity.y) +"," + str(self.angle_velocity.z) + "\nacl:" +str(self.angle_accl.x) +"," +str(self.angle_accl.y) +"," +str(self.angle_accl.z)
            return lin + "\n" + angular
        #\nvel: $s, $s, $s\nacl:%s, %s, %s

#quad_state will be used internaly to the quadcopter model, to represent the
#body state and the setpoint.
class quad_state:
        def __init__(self):
                self.roll = 0.0
                self.pitch = 0.0
                self.yaw = 0.0
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
        def __str__(self):
            return str(self.roll) + ", " + str(self.pitch) + ", " + str(self.yaw) + ", " + str(self.x) + ", " + str(self.y) + ", " + str(self.z)

class sensor:
    """A base class for sensors"""
    def __init__(self, name, mu, sigma):
        self.name_of_sensor = name
        self.noise_mu = mu
        self.noise_sigma = sigma
    def noise(self):
        return gauss(self.noise_mu, self.noise_sigma)
    def update(self, physical_state):
        #Some function stub
        self.name = self.name
    def measure(self):
        #Some function stub
        return self.name
    def name(self):
        return self.name_of_sensor

class sonar(sensor):
    def __init__(self, name, mu, sigma):
        sensor.__init__(self, name, mu, sigma)
        self.distance = 0
    def update(self, physical_state):
        self.distance = physical_state.position.z
    def measure(self):
        return self.noise() + self.distance
        
class accelerometer(sensor):
    def __init__(self, name, mu, sigma):
        sensor.__init__(self, name, mu, sigma)
        self.dcm = numpy.zeros([3,3])
    def compute_dcm(self,phi, theta, psi):
        return  numpy.array(
            [[math.cos(theta)*math.cos(psi), math.sin(phi)*math.sin(theta)*math.cos(psi)-math.cos(phi)*math.sin(psi), math.cos(phi)*math.sin(theta)*math.cos(psi)+math.sin(phi)*math.sin(psi)],
            [math.cos(theta)*math.cos(psi), math.sin(phi)*math.sin(theta)*math.cos(psi)+math.cos(phi)*math.sin(psi), math.cos(phi)*math.sin(theta)*math.cos(psi)-math.sin(phi)*math.sin(psi)],
            [-math.sin(theta), math.sin(phi)*math.cos(theta), math.cos(phi)*math.cos(theta)]]
            )
    def update(self, physical_state):
        self.dcm = self.compute_dcm(physical_state.angle.x,physical_state.angle.y,physical_state.angle.z)
    def measure(self):
        #This is not finished, output is in g's directly we need to scale
        #for the sensor values and add noise
        t_dcm = list(zip(*self.dcm)) * numpy.array([0,0,-g])
        grav_vector = ThreeState()
        grav_vector.x = t_dcm[0][2]
        grav_vector.y = t_dcm[1][2]
        grav_vector.z = t_dcm[2][2]
        return grav_vector

class rate_gyro(sensor):
    def __init__(self, name, mu, sigma):
        sensor.__init__(self, name, mu, sigma)
        self.psi = 0
        self.phi = 0
        self.theta = 0
    def update(self, physical_state):
        self.phi =  physical_state.angle_velocity.x
        self.theta = physical_state.angle_velocity.y
        self.psi = physical_state.angle_velocity.z
    def measure(self):
        #not really not done, no noise!
        return [self.phi, self.theta, self.psi]

class inertial_measurement_unit():
    def __init__(self):
        self.accel = accelerometer('adxl 345', adxl345_mu, adxl345_sigma)
        self.gyro = rate_gyro('itg3200', itg3200_mu, itg3200_sigma)
        self.height = sonar('maxbotixs', sonar_mu, sonar_sigma)
    def update(self, physical_state):
        self.accel.update(physical_state)
        self.gyro.update(physical_state)
        self.height.update(physical_state)
    #returns all the values of the sensors in the imu,
    #The format is an array containg the 3 gyro values, followed by the 3 accel values, followed by the height sensor value
    def measure(self):
        gy = self.gyro.measure()
        acc = self.accel.measure()
        h = self.height.measure()
        return [gy[0],gy[1],gy[2],acc.x,acc.y,acc.z,h]

#These are the various physics calculations that are used by the quadcopter, roll these into the quadcopter class?
def lin_drag(Cd, area, velocity):
    return p * Cd * area * (velocity*velocity)

def lin_accel(force, velocity, area, Cd):
    return (force - lin_drag(Cd, area, velocity))/mass

def rot_drag(Cd, area, velocity):
    #Need to figure this out, talked w/Greg, integral of the velocity over the radius?
    return 0

def torque(force, r):
    return (r * force * 1) #This is the radius * force * the sin of the angle that the force is applied with, in our case this is 90 degrees so we just make it 1

def rot_accel(force, rot_vel, area, Cd, I):
    return (torque(force, arm_length) - rot_drag(Cd, area, rot_vel))/I

def update_physical_state(physical_state):
    #calculate the force on each axis
    #on the x axis caculate the linear and rot accel
    x_accel = lin_accel(10, physical_state.velocity.x, Ax, CodX)
    x_rot_accel = rot_accel(0, physical_state.angle_velocity.x, Ax, CodX, Ix)
    #on the y axis caculate the linear and rot accel
    y_accel = lin_accel(0, physical_state.velocity.y, Ay, CodY)
    y_rot_accel = rot_accel(10, physical_state.angle_velocity.y, Ay, CodY, Iy)
    #on the z axis caculate the linear and rot accel
    z_accel = lin_accel(10, physical_state.velocity.z, Az, CodZ)
    z_rot_accel = rot_accel(10, physical_state.angle_velocity.z, Az, CodZ, Iz)

    #Integrate the velocity and position values, just do dumb integration for now, look into better methods.
    physical_state.velocity.x += x_accel/inverse_delta
    physical_state.velocity.y += y_accel/inverse_delta
    physical_state.velocity.z += z_accel/inverse_delta

    #Should I update this before the velocity?
    physical_state.position.x += physical_state.velocity.x/inverse_delta
    physical_state.position.y += physical_state.velocity.y/inverse_delta
    physical_state.position.z += physical_state.velocity.z/inverse_delta

    #update the rotation values, again dumb integration
    physical_state.angle_velocity.x += x_rot_accel/inverse_delta
    physical_state.angle_velocity.y += y_rot_accel/inverse_delta
    physical_state.angle_velocity.z += z_rot_accel/inverse_delta

    physical_state.angle.x = physical_state.angle_velocity.x/inverse_delta
    physical_state.angle.y = physical_state.angle_velocity.y/inverse_delta
    physical_state.angle.z = physical_state.angle_velocity.z/inverse_delta

def load_command_file(file, delta_conversion):
    command_data = numpy.loadtxt(file, delimiter=',')
    for i in range(len(command_data)):
        command_data[i][0] *= delta_conversion

    return command_data

def command_to_setpoint(command):
    tmp_setpoint = quad_state()
    tmp_setpoint.roll = command[1]
    tmp_setpoint.pitch = command[2]
    tmp_setpoint.yaw = command[3]
    tmp_setpoint.x = command[4]
    tmp_setpoint.y = command[5]
    tmp_setpoint.z = command[6]
    return tmp_setpoint
    

class QuadCopterModel():
    def __init__(self):
        self.imu = inertial_measurement_unit()
        self.motor_thrust = motor_max_thrust #the max thrust each motor is capable of
        self.motor_max_rpm = motor_max_rpm
        self.thrust_per_rpm = self.motor_thrust/motor_max_rpm
        self.setpoint = quad_state()
        self.position = quad_state()
        self.motor_val = [0,0,0,0]
        
    def position(self):
        return self.position
    
    def sensor_filter(imu_state):
        #This is a placeholder currently
        return imu_state
        
    def state_update(self):
        imu_state = self.imu.measure()
        #sensor_filter(imu_state)
        self.position.roll += imu_state[0]/inverse_delta
        self.position.pitch += imu_state[1]/inverse_delta
        self.position.yaw += imu_state[2]/inverse_delta

    def control_update(self):
        motor_state = [0,0,0,0]
        
    def update(self, physical_state):
        self.imu.update(physical_state)
        self.state_update()
        self.control_update()
        
    def set_setpoint(self,new_setpoint):
        self.setpoint = new_setpoint

if __name__ == '__main__':
    #setup stuff for the graphs to display
    mpl.rcParams['legend.fontsize'] = 10

    fig = plt1.figure()
    ax = fig.gca(projection='3d')
    theta = numpy.linspace(-4 * numpy.pi, 4 * numpy.pi, 100)
    z = []
    x = []
    y = []
    ax.legend()

    #Setup stuff for quadcopter and initializing the world
    print ('Start')
    world = physical_state()
    time = 0
    command = 0
    command_array = load_command_file('./prototypes/runs/r1.run',100)
    QC = QuadCopterModel()
    QC.imu.update(world) #Do this once to get good vaules

    #Main loop
    while (time <= run_time):
        if command_array[command][0] <= time:
            QC.set_setpoint(command_to_setpoint(command_array[command]))
            print (QC.setpoint)
            if command < len(command_array-1):
                command += 1

        force = QC.update(world)
        update_physical_state(world)
        #Here is where we add whatever data we want to the graphs
        x.append(world.position.x)
        y.append(world.position.y)
        z.append(world.position.z)

        #Increment the time
        time += delta_t

    ax.plot(x, y, z, label='parametric curve')
    plt1.show()
