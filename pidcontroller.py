
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

    from time import time

    # Can't touch this!
    G = 9.80665

    # Constant to experiment with
    ALTITUDE_START  = 100
    ALTITUDE_TARGET = 50

    # initial conditions
    z    = ALTITUDE_START
    dzdt = 0
    t0   = time()
    t1   = 0
    u    = 0

    pid = AltitudePidController(
        ALTITUDE_TARGET,
        5.00,   # P
        1.50,   # Velocity P
        1.00,   # Velocity I
        0.05)   # Velocity D

    while True:

        if z <=0:
            break

        t = time() - t0

        dt = t - t1

        if dt > 0:

            dzdt2 = G - u
            
            dzdt -= dzdt2 * dt

            z += dzdt*dt

            u = pid.u(z, dzdt, dt)

            print('t: %-08.3f: dzdt2: %-10.3f dzdt: %-10.3f z: %-10.3f | u: %-10.3f' %
                  (t, dzdt2, dzdt, z, u))

        t1 = t


    print('********************************************************')
    print("********** I'VE FALLEN, AND I CAN'T GET UP! ************")
    print('********************************************************')

        
