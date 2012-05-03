import math

max_motor_rpm = 10212
motor_max_thrust = 7.6972
thrust_per_rpm = motor_max_thrust/max_motor_rpm

def thrust(motor_percent, roll, pitch):
    z_thrust = 0.0
    x_thrust = 0.0
    y_thrust = 0.0
    for p in motor_percent:
        z_thrust += p*max_motor_rpm*thrust_per_rpm

    x_thrust += (motor_percent[0]*max_motor_rpm*thrust_per_rpm)*math.sin(roll)
    x_thrust += (motor_percent[2]*max_motor_rpm*thrust_per_rpm)*math.sin(roll)
    y_thrust += (motor_percent[1]*max_motor_rpm*thrust_per_rpm)*math.sin(pitch)
    y_thrust += (motor_percent[3]*max_motor_rpm*thrust_per_rpm)*math.sin(pitch)
    return [x_thrust, y_thrust, z_thrust]

motor = [50,50,50,50]
xb = -math.pi/2
yb = -math.pi/2

while (xb <= math.pi/2):
    print ('xb:',xb)
#    print ('yb:',yb)
#    print ('pitch:',pitch())
    print(thrust(motor, xb, 0))
    print('--------------------------------------')
    xb += math.pi/4

while (yb <= math.pi/2):
#    print ('xb:',xb)
    print ('yb:',yb)
#    print ('pitch:',pitch())
    print(thrust(motor, 0, yb))
    print('--------------------------------------')
    yb += math.pi/4

yb = -math.pi/2

while (yb <= math.pi/2):
    print ('xb:',xb)
    print ('yb:',yb)
#    print ('pitch:',pitch())
    print(thrust(motor, xb, yb))
    print('--------------------------------------')
    yb += math.pi/4

