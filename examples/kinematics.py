import cv2
import numpy as np
from PIL import Image, ImageFont, ImageDraw
from vpython import *

from pyal5d import *

if __name__ == '__main__':

    cv2.namedWindow('angles')
    fontpath = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font = ImageFont.truetype(fontpath, 32)
    img = np.zeros((480, 640))
    cv2.imshow('angles', img)
    
    arm = RoboticArm()
    try:
        arm.open()
    except:
        print("Couldn't connect with serial port")

    arm.home()
    
    v = 1
    t = 50
    coords = []

    while True:
        img = np.zeros((480, 640))
        img_pil = Image.fromarray(img)
        draw = ImageDraw.Draw(img_pil)
        draw.text((60, 80),
            '\u03B8=[{0:.1f}, {1:.1f}, {2:.1f}, {3:.1f}]\n'
            'x={5:.3f} cm\ny={6:.3f} cm\nz={7:.3f} cm'.format(
                *arm.thetas, *arm.coords), font=font)
        img = np.array(img_pil)
        cv2.imshow('angles', img)

        key = chr(cv2.waitKey())
        print('key = ', key)
        np.set_printoptions(precision=3)

        if key == 'a':
            arm.increment(0, v, t)
        if key == 'z':
            arm.increment(0, -v, t)
        if key == 's':
            arm.increment(1, v, t)
        if key == 'x':
            arm.increment(1, -v, t)
        if key == 'd':
            arm.increment(2, v, t)
        if key == 'c':
            arm.increment(2, -v, t)
        if key == 'f':
            arm.increment(3, v, t)
        if key == 'v':
            arm.increment(3, -v, t)
        if key == 'g':
            arm.increment(4, v, t)
        if key == 'b':
            arm.increment(4, -v, t)
        if key == ' ':
            coords.append(arm.coords)
            print(np.array(coords))
        if key == chr(27):
            arm.close()
            break
