import serial
import numpy as np

BASE = 0
SHOULDER = 1
ELBOW = 2
WRIST = 3
GRIP = 4


class RoboticArm(object):
    def __init__(self, bas=None, shl=None, elb=None, wri=None, gri=None):
        if bas is None:
            bas = (500, 2400)
        if shl is None:
            shl = (1200, 2000)
        if elb is None:
            elb = (1100, 2000)
        if wri is None:
            wri = (500, 2500)
        if gri is None:
            gri = (1300, 2400)
        self.bounds = np.array([bas, shl, elb, wri, gri])
        self.serial_port = None
        self.thetas = np.array([None, None, None, None, None])

    def open(self, name='/dev/ttyUSB0'):
        self.serial_port = serial.Serial(name,
                                         baudrate=115200,
                                         timeout=1)
        if not self.serial_port.is_open:
            self.serial_port.open()

    def close(self):
        self.serial_port.close()
        self.serial_port = None

    def send(self, cmd):
        self.serial_port.write(cmd)

    def lock(self, channel, pos):
        channel = np.atleast_1d(channel)
        pos = np.atleast_1d(pos)

        bmin, bmax = self.bounds[channel].T

        pos[pos < bmin] = bmin[pos < bmin]
        pos[pos > bmax] = bmax[pos > bmax]
        return pos

    def ang2pos(self, ang, chn):
        ang = np.atleast_1d(ang)
        chn = np.atleast_1d(chn)
        m = chn.size
        if ang.size == 1:
            ang = np.array([ang[0] for _ in range(m)])
        assert ang.size == m, "incompatible sizes ({} channels, {} angles)".format(m, ang.size)

        reverse = lambda x: 180 - x
        delay = lambda x: x + 90
        same = lambda x: x
        funcs = np.array([reverse, same, reverse, delay, same])

        theta = np.array([funcs[chn[i]](ang[i]) for i in range(m)])
        pos = theta / .09 + 500
        return pos

    def pos2ang(self, pos, chn):
        pos = np.atleast_1d(pos)
        chn = np.atleast_1d(chn)
        m = chn.size
        if pos.size == 1:
            pos = np.array([pos[0] for _ in range(m)])
        assert pos.size == m, "incompatible sizes ({} channels, {} pulses)".format(m, pos.size)

        reverse = lambda x: 180 - x
        undelay = lambda x: x - 90
        same = lambda x: x
        funcs = np.array([reverse, same, reverse, undelay, same])

        theta = (pos - 500) * .09
        ang = np.array([funcs[chn[i]](theta[i]) for i in range(m)])
        return ang

    def move(self, channel, theta, time=1500):
        channel = np.atleast_1d(channel)
        theta = np.atleast_1d(theta)
        m = channel.size
        if theta.size == 1:
            theta = np.array([theta[0] for _ in range(m)])
        assert theta.size == m, "incompatible sizes ({} channels, {} angles)".format(m, theta.size)

        channel, mask = np.unique(channel, return_index=True)
        theta = theta[mask]

        pos = self.ang2pos(theta, channel)
        pos = self.lock(channel, pos)
        cmd = ""
        for i in range(m):
            cmd += "#%dP%d" % (channel[i], pos[i])

        theta = self.pos2ang(pos, channel)
        self.thetas[channel] = theta
        cmd = bytes("%sT%d\r" % (cmd, time))
        self.send(cmd)

    def home(self):
        self.move(theta=[90, 90, 90, 0, 90], channel=[0, 1, 2, 3, 4])

    def increment(self, channel, dtheta, time=1500):
        theta = self.thetas[channel] + dtheta
        self.move(channel, theta, time=time)
