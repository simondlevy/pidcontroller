#!/usr/bin/env python3

class AltitudePidController(object):

    def __init__(self, target, posP, velP, velI, velD, windupMax=10):

        # In a real PID controller, this would be a set-point
        self.target = target

        # Constants
        self.posP = posP
        self.velP = velP
        self.velI = velI
        self.velD = velD
        self.windupMax = windupMax

        # Values modified in-flight
        self.posTarget      = 0
        self.lastError      = 0
        self.integralError  = 0
        self.altitudeTarget = 0

    def u(self, alt, vel, dt):

        # compute dzdt setpoint and error
        velTarget = (self.target - alt) * self.posP
        velError = velTarget - vel

        # Update error integral and error derivative
        self.integralError +=  velError * dt
        self.integralError = AltitudePidController._constrainAbs(self.integralError + velError * dt, self.windupMax)
        deltaError = (velError - self.lastError) / dt if abs(self.lastError) > 0 else 0
        self.lastError = velError

        # Compute control u
        return self.velP * velError + self.velD * deltaError + self.velI * self.integralError

    def _constrainAbs(x, lim):

        return -lim if x < -lim else (+lim if x > +lim else x) 

if __name__ == '__main__':

    # Can't touch this!
    G = 9.80665

    # Reasonable time constant
    DT = 0.001

    # Constant to experiment with
    ALTITUDE_START  = 100
    ALTITUDE_TARGET = 50

    # initial conditions
    t     = 0
    z     = ALTITUDE_START
    dzdt  = 0
    u     = 0
    zprev = 0

    pid = AltitudePidController(
        ALTITUDE_TARGET,
        5.00,   # P
        1.50,   # Velocity P
        1.00,   # Velocity I
        0.05)   # Velocity D

    while True:

        # If altitude has leveled off, halt
        if abs(z-zprev) < .0000001:
            break

        zprev = z

        dzdt2 = G - u
        
        dzdt -= dzdt2 * DT

        z += dzdt*DT

        u = pid.u(z, dzdt, DT)

        print('%3.3f,%3.3f,%3.3f,%3.3f,%3.3f' % (t, dzdt2, dzdt, z, u))

        t += DT

        
