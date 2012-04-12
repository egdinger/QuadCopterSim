import math, time
import numpy
#xb = 3*math.pi/4
#yb = math.pi/4
xb = 0#-math.pi/2 #phi roll O|
yb = 0 #theta pitch O-
zb = 0 #psi yaw trident
grav = 1023

x = [[1,2,3],
     [4,5,6]]

#Freescales app note 3461 has a good breakdown of the math needed

#Takes the input in the order roll, pitch, yaw
def compute_dcm(phi, theta, psi):
    return  numpy.array(
            [[math.cos(theta)*math.cos(psi), math.sin(phi)*math.sin(theta)*math.cos(psi)-math.cos(phi)*math.sin(psi), math.cos(phi)*math.sin(theta)*math.cos(psi)+math.sin(phi)*math.sin(psi)],
            [math.cos(theta)*math.cos(psi), math.sin(phi)*math.sin(theta)*math.cos(psi)+math.cos(phi)*math.sin(psi), math.cos(phi)*math.sin(theta)*math.cos(psi)-math.sin(phi)*math.sin(psi)],
            [-math.sin(theta), math.sin(phi)*math.cos(theta), math.cos(phi)*math.cos(theta)]]
            )

#Takes the input in the order roll, pitch, yaw
def compute_t_dcm(phi, theta, psi):
    return list(zip(*compute_dcm(theta, phi, psi)))

def p_dcm():
    dcm = compute_dcm(xb, yb, zb)
    for i in dcm:
        print (i)

def p_t_dcm():
    dcm = compute_t_dcm(xb, yb, zb)
    for i in dcm:
        print (i)


def mag_x():
    dcm = compute_t_dcm(xb, yb, zb)
    return dcm[0][2]
    #return -math.sin(xb)*grav #3461 
    #return math.(xb)*math.sin(yb)*math.cos(zb)

def mag_y():
    dcm = compute_t_dcm(xb, yb, zb)
    return dcm[1][2]
    #return math.cos(xb)*math.sin(yb)*grav #3461
    #return math.cos(xb)*math.sin(yb)
    #return math.sin(xb)*math.sin(yb)*math.cos(zb)

def mag_z():
    dcm = compute_t_dcm(xb, yb, zb)
    return dcm[2][2]
    #return math.cos(xb)*math.cos(yb)*grav #3461
    

def grav_mag():
    Mx = mag_x()
    print('Mx:',Mx)
    My = mag_y()
    print('My:',My)
    Mz = mag_z()
    print('Mz:',Mz)
    
    return math.sqrt((Mx*Mx)+(My*My)+(Mz*Mz))

def pitch():
    Mx = mag_x()
    My = mag_y()
    Mz = mag_z()

    tan = (My/Mz)
    return math.atan2(My,Mz)

def roll():
    Mx = mag_x()
    My = mag_y()
    Mz = mag_z()
    return math.atan((-Mx/math.hypot(My,Mz)))

while (xb <= math.pi*2):
    print ('xb:',xb)
#    print ('yb:',yb)
    print (grav_mag())
    print ('roll:',roll())
#    print ('pitch:',pitch())
    p_t_dcm()
    print('--------------------------------------')
    xb += math.pi/4

print('--------------------------------------')
print('--------------------------------------')
print('--------------------------------------')

while (yb < math.pi):
#    print ('xb:',xb)
    print ('yb:',yb)
    print (grav_mag())
#    print ('roll:',roll())
    print ('pitch:',pitch())
    print('--------------------------------------')
    yb += math.pi/4

print('--------------------------------------')
print('--------------------------------------')
print('--------------------------------------')

xb = math.pi/4
yb = 0

while (yb < math.pi/2):
#    print ('xb:',xb)
    print ('yb:',yb)
    print (grav_mag())
#    print ('roll:',roll())
    print ('pitch:',pitch())
    print('--------------------------------------')
    yb += math.pi/8

print('--------------------------------------')
print('--------------------------------------')
print('--------------------------------------')

while (zb < math.pi):
#    print ('xb:',xb)
    print ('yb:',yb)
    print (grav_mag())
    print ('roll:',roll())
    print ('pitch:',pitch())
    print('--------------------------------------')
    zb += math.pi/4
