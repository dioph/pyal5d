from time import sleep

from inputs import get_key
from vpython import *
from vpython.no_notebook import stop_server

from pyal5d import *

N = scene.width
M = scene.height

arm = RoboticArm()
try:
    arm.open()
except:
    print("Couldn't connect with serial port")

arm.home()
sleep(2)

label(pos=vec(0, M, 0), pixel_pos=True, xoffset=100, yoffset=100,
      height=20, line=True, align='left', font='monospace',
      text='CONTROLS:'
           '\na/z - base'
           '\ns/x - shoulder'
           '\nd/c - elbow'
           '\nf/v - wrist'
           '\ng/b - grip'
           '\n<space> - save coords')

L = label(pos=vec(0,0,0), pixel_pos=True, xoffset=75, yoffset=-75, 
          height=20, line=True, align='left', font='monospace',
          text='\u03B8=[%.1f, %.1f, %.1f, %.1f]'
               '\nx=%.3f cm'
               '\ny=%.3f cm'
               '\nz=%.3f cm'
               '\n\u03D5=%.2f' % 
                (*arm.thetas[:GRIP], *arm.coords))

c = curve(retain=100, color=color.blue)

v = 1.
t = 50
coords = []

while True:
    x, y, z, phi = arm.coords
    c.append(pos=vector(x, y, z))

    L.text = '\u03B8=[%.1f, %.1f, %.1f, %.1f]' \
             '\nx=%.3f cm' \
             '\ny=%.3f cm' \
             '\nz=%.3f cm' \
             '\n\u03D5=%.2f' % (*arm.thetas[:GRIP], *arm.coords)

    event, = get_key()
    key = event.code
    # stop program
    if key == 'KEY_ESC':
        arm.close()
        stop_server()
        break
    # movement keys
    if key == 'KEY_A':
        arm.increment(BASE, v, t)
    if key == 'KEY_Z':
        arm.increment(BASE, -v, t)
    if key == 'KEY_S':
        arm.increment(SHOULDER, v, t)
    if key == 'KEY_X':
        arm.increment(SHOULDER, -v, t)
    if key == 'KEY_D':
        arm.increment(ELBOW, v, t)
    if key == 'KEY_C':
        arm.increment(ELBOW, -v, t)
    if key == 'KEY_F':
        arm.increment(WRIST, v, t)
    if key == 'KEY_V':
        arm.increment(WRIST, -v, t)
    # grip keys
    if key == 'KEY_G':
        arm.hold()
        sleep(0.5)
    if key == 'KEY_B':
        arm.release()
        sleep(0.5)
    # save current position and draw lines to all previous ones
    if key == 'KEY_SPACE':
        for coord in coords:
            curve(vec(*coord), vec(x, y, z))
        coords.append((x, y, z))
        sphere(pos=vector(x, y, z), radius=1)
