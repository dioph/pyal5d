import cv2
import numpy as np
from pyal5d import *
from time import sleep

def labeling(binary):
    M, N = binary.shape
    image = binary.copy()
    nobjects = 0
    for i in range(M):
        for j in range(N):
            if image[i, j] == 255:
                nobjects += 1
                cv2.floodFill(image, None, (j, i), nobjects)
    return nobjects, image

def pixel2coords(x, y):
    # xt = 522 * x / 3800 - 27.7
    # yt = -2525 * y / 21000 + 50.5
    xt = 0.12 * x - 27.5
    yt = -0.12 * y + 50.
    return xt, yt

def get_children(hierarchy, target):
    child = target[2]
    nchildren = 0
    while child >= 0:
        nchildren += 1
        child = hierarchy[0, child, 0]
    return nchildren

arm = RoboticArm()
try:
    arm.open()
except:
    print("Couldn't connect with serial port")

arm.home()
sleep(3)
arm.goto(x=0, y=0, z=10)
sleep(3)

nblur = 7
nopen = 3
nclose = 3

cap = cv2.VideoCapture(2)
_, frame = cap.read()
frame = frame[100:, 100:-120, :]
M, N = frame.shape[:2]
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, ksize=(nblur,nblur), sigmaX=0, sigmaY=0)
_, img = cv2.threshold(blur, 0, 255, cv2.THRESH_OTSU)
img = 255 - img
for i in range(M):
    for j in [0, N-1]:
        if img[i, j] == 255:
            cv2.floodFill(img, None, (j, i), 0)
for i in [0, M-1]:
    for j in range(N):
        if img[i, j] == 255:
            cv2.floodFill(img, None, (j, i), 0)
element_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (nopen, nopen))
element_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (nclose, nclose))
img = cv2.morphologyEx(img, cv2.MORPH_OPEN, element_open)
img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, element_close)

contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
targets = np.where(hierarchy[0, :, -1] < 0)[0]

xy_targets = np.zeros((3, 2))

clone = np.dstack([img, img, img])
for target in targets:
    c = contours[target]
    nchildren = get_children(hierarchy, hierarchy[0, target, :])
    m = cv2.moments(c)
    cx = int(m["m10"] / m["m00"])
    cy = int(m["m01"] / m["m00"])
    if nchildren == 0:
        cv2.circle(clone, (cx, cy), 20, (0,255,0))
        xo, yo = pixel2coords(cx, cy)
        sleep(3)
    else:
        cv2.putText(clone, f'{nchildren-1}', (cx, cy), fontScale=2,
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, thickness=2,
                    color=(255,0,0), lineType=cv2.LINE_AA)
        xy_targets[nchildren-1] = pixel2coords(cx, cy)
    target = hierarchy[0, target, 0]

cv2.namedWindow('teste')
cv2.imshow('teste', clone)
cv2.waitKey(0)
arm.release(time=1000)
sleep(1)
arm.goto(x=xo, y=yo, z=4, time=3000)
sleep(5)
arm.goto(phi=-60)
sleep(3)
arm.goto(z=1)
sleep(3)
arm.hold(time=1000)
sleep(3)
arm.goto(z=30, phi=90)

key = ''
while key not in [ord('0'), ord('1'), ord('2')]:
    key = cv2.waitKey(0)

key = int(chr(key))
xt, yt = xy_targets[key]

arm.goto(x=xt, y=yt, z=15, phi=0)
sleep(3)
arm.goto(z=1, phi=-90)
sleep(3)
arm.release(time=1000)
sleep(1)
