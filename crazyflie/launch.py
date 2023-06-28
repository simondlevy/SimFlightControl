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

import numpy as np

from multicopter_server import MulticopterServer
# from debugging import debug

from pid_controller import pid_velocity_fixed_height_controller


class LaunchCopter(MulticopterServer):

    THROTTLE_INCREMENT = 1e-4

    def __init__(self, initial_target=1.0):

        MulticopterServer.__init__(self)

        self.time = 0
        self.target = initial_target

        self.pid_controller = pid_velocity_fixed_height_controller()

    def getMotors(self, t, state, stickDemands):

        '''
        if stickDemands[0] < 0.25:
            self.target -= self.THROTTLE_INCREMENT

        if stickDemands[0] > 0.75:
            self.target += self.THROTTLE_INCREMENT
        '''

        self.target += self._increment(
               stickDemands[0], 0.25, 0.75, self.THROTTLE_INCREMENT)

        motors = np.zeros(4)

        if self.time > 0:

            motors = np.array(self.pid_controller.pid(
                    t - self.time,  # dt
                    0,  # desired_vx
                    0,  # desired_vy
                    0,  # desired_yaw_rate
                    self.target,  # desired_altitude
                    0,  # actual_roll
                    0,  # actual_pitch
                    0,  # actual_yaw_ratev
                    state[MulticopterServer.STATE_Z],  # actual_altitude
                    0,  # actual_vx
                    0)  # actual_vy
                    ) / 100

        # Track current time to share it with handleImage()
        self.time = t

        # Return motor values
        return motors

    def _increment(self, stick, lo_limit, hi_limit, increment):

        return (-increment if stick < lo_limit
                else +increment if stick > hi_limit
                else 0)


def main():

    copter = LaunchCopter()
    copter.start()


main()
