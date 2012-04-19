#Prototype for the sonar sensor

from random import *

class state:
    def __init__(self):
        self.z = 0

class sonar:
    def __init__(self, mu = 0, sigma = 0):
        self.height = 0.0
        self.noise_mu = mu
        self.noise_sigma = sigma

    def update(self, phys_state):
        self.height = phys_state.z

    def measure(self):
        return (self.height + gauss(self.noise_mu, self.noise_sigma))

test = sonar(0,1)
ts = state()

for j in range(0,10,2):
    print (j)
    ts.z = int(j)
    test.update(ts)
    for k in range(3):
        print(' ',test.measure())
