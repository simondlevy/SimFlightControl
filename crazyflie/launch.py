#!/usr/bin/env python3
'''

  Simple take-off-and-move-forward script

  Copyright (C) 2021 Simon D. Levy

  This file is part of SimFlightControl.

  SimFlightControl is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free
  Software Foundation, either version 3 of the License, or (at your option)
  any later version.

  SimFlightControl is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
  or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  SimFlightControl. If not, see <https://www.gnu.org/licenses/>.

'''

try:
    import cv2
except Exception:
    pass

import numpy as np

from multicopter_server import MulticopterServer
from mixers import PhantomMixer
from debugging import debug

from pid_controller import pid_velocity_fixed_height_controller
from controller import LaunchController


class LaunchCopter(MulticopterServer):

    def __init__(self, kp=1.0, initial_target=2.5):

        MulticopterServer.__init__(self)

        self.mixer = PhantomMixer()

        self.time = 0
        self.target = initial_target

        # Create PID controller
        self.ctrl = LaunchController(kp)

        self.crazyflie_pid = pid_velocity_fixed_height_controller()

    def getMotors(self, t, state, _stickDemands):

        # Extract altitude and its first derivative from state.
        z = state[MulticopterServer.STATE_Z]
        dzdt = state[MulticopterServer.STATE_DZ]

        if self.time > 0:

            motors = self.crazyflie_pid.pid(
                    t - self.time, # dt
                    0, # desired_vx
                    0, # desired_vy
                    0, # desired_yaw_rate
                    self.target, # desired_altitude
                    0, # actual_roll
                    0, # actual_pitch
                    0, # actual_yaw_rate
                    z, # actual_altitude
                    0, # actual_vx
                    0) # actual_vy

            debug(motors)

        # Track current time to share it with handleImage()
        self.time = t

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

    copter = LaunchCopter()
    copter.start()


main()
