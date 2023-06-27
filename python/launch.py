#!/usr/bin/env python3
'''
Simple take-off-and-move-forward script

Copyright (C) 2021 Simon D. Levy

MIT License
'''

try:
    import cv2
except Exception:
    pass

import numpy as np
import argparse
from argparse import ArgumentDefaultsHelpFormatter

from controller import LaunchController
from multicopter_server import MulticopterServer
from mixers import PhantomMixer, IngenuityMixer
from debugging import debug


class LaunchCopter(MulticopterServer):

    def __init__(
            self,
            mixer,
            kp=1.0,
            initial_target=2.5):

        MulticopterServer.__init__(self)

        self.mixer = mixer

        self.time = 0
        self.target = initial_target

        # Create PID controller
        self.ctrl = LaunchController(kp)

    def handleImage(self, image):
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 100, 200)
            cv2.imshow('Edge Detection', edges)
            cv2.waitKey(1)

            nonzero = np.nonzero(edges)[0]

            # Ignore image for first five seconds
            if len(nonzero) > 0 and np.mean(nonzero) > 390 and self.time > 5:
                self.target = 30

        except Exception:
            debug('Failed')
            pass

    def getMotors(self, t, state, _stickDemands):

        # Track current time to share it with handleImage()
        self.time = t

        # Extract altitude and its first derivative from state.  
        z = state[MulticopterServer.STATE_Z]
        dzdt = state[MulticopterServer.STATE_DZ]

        # Get demands U [throttle, roll, pitch, yaw] from PID controller,
        # ignoring stick demands
        u = self.ctrl.getDemands(self.target, z, dzdt)

        # Use mixer to convert demands U into motor values Omega
        omega = self.mixer.getMotors(u)

        # Constrain motor values to [0,1]
        omega[omega > 1] = 1
        omega[omega < 0] = 0

        # Return motor values
        return omega


def main():

    parser = argparse.ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('--vehicle', required=False, default='Phantom',
                        help='Vehicle name')

    args = parser.parse_args()

    d = {'Phantom': PhantomMixer, 'Ingenuity': IngenuityMixer}

    if args.vehicle in d:
        copter = LaunchCopter(d[args.vehicle]())
        copter.start()

    else:
        debug('Unrecognized vehicle: %s' % args.vehicle)


main()
