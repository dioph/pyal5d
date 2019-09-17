from time import sleep

import numpy as np
from inputs import get_key
from vpython import *
from vpython.no_notebook import stop_server

from pyal5d import *

N = 512
M = 512

if __name__ == '__main__':
    
    scene.width = N
    scene.height = M
    scene.resizable = False

    arm = RoboticArm()
    try:
        arm.open()
    except:
        print("Couldn't connect with serial port")

    arm.home()
    sleep(2)
    
    L = label(pos=vec(0,0,0), pixel_pos=True, xoffset=75, yoffset=-75, 
              height=20, line=True, align='left', font='monospace',
              text='\u03B8=[%.1f, %.1f, %.1f, %.1f]'
                   '\nx=%.3f cm'
                   '\ny=%.3f cm'
                   '\nz=%.3f cm'
                   '\n\u03D5=%.2f' % 
                    (*arm.thetas[:GRIP], *arm.coords))

    label(pos=vec(N, M,0), pixel_pos=True, xoffset=-100, yoffset=100,
          height=20, line=True, align='left', font='monospace',
          text='CONTROLS:'
               '\na/z - base'
               '\ns/x - shoulder'
               '\nd/c - elbow'
               '\nf/v - wrist'
               '\ng/b - grip'
               '\n<space> - save coords')

    cvs = canvas(title='', width=N, height=M, align='right', resizable=False)
    c = curve(retain=100, color=color.blue)

    v = 0.5
    t = 10
    coords = []

    while True:
        c.append(pos=vector(*arm.coords[:-1]))

        L.text = '\u03B8=[%.1f, %.1f, %.1f, %.1f]'
                 '\nx=%.3f cm'
                 '\ny=%.3f cm'
                 '\nz=%.3f cm'
                 '\n\u03D5=%.2f' %
                    (*arm.thetas[:GRIP], *arm.coords)

        sleep(t * 1e-3)
        event = get_key()
        key = event[0].code

        if key == 'KEY_ESC':
            arm.close()
            stop_server()
            break
        if key == 'KEY_A':
            arm.increment(0, v, t)
        if key == 'KEY_Z':
            arm.increment(0, -v, t)
        if key == 'KEY_S':
            arm.increment(1, v, t)
        if key == 'KEY_X':
            arm.increment(1, -v, t)
        if key == 'KEY_D':
            arm.increment(2, v, t)
        if key == 'KEY_C':
            arm.increment(2, -v, t)
        if key == 'KEY_F':
            arm.increment(3, v, t)
        if key == 'KEY_V':
            arm.increment(3, -v, t)

        if key == 'KEY_G':
            arm.hold()
            sleep(0.5)
        if key == 'KEY_B':
            arm.release()
            sleep(0.5)

        if key == 'KEY_SPACE':
            for coord in coords:
                curve(vec(*coord), vec(*arm.coords[:-1]))
            coords.append(arm.coords[:-1])
            sphere(pos=vector(*coords[-1]), radius=1)
