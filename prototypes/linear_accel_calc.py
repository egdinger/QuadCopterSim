#acceleration and drag test
import matplotlib.pyplot as plt

K = 1.225*1.04*.25
m = 1 # in kg

def accl(force, velocity):
    return (force - K*(velocity*velocity))/m

delta_t = .01 #seconds

f = 3
v = 0
v_plot = []
a_plot = []

samples = 1000

for i in range(samples):
    a = accl(f,v)
    a_plot.append(a)
    v += a*delta_t
    v_plot.append(v)

plt.plot(list(range(samples)),a_plot)
plt.plot(list(range(samples)),v_plot)

plt.show()
