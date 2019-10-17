from time import sleep
from pyal5d import *

arm = RoboticArm()
try:
    arm.open()
except:
    print("Couldn't connect with serial port")

arm.home()
sleep(2)

xo, yo, zo, phio = np.array(input('object: ').split(',')).astype(float)
xt, yt, zt, phit = np.array(input('target: ').split(',')).astype(float)

arm.goto(xo, yo, zo, phio)
sleep(3)
arm.hold(time=1000)
sleep(1)
arm.goto(xt, yt, zt, phit)
sleep(3)
arm.release(time=1000)
sleep(1)
