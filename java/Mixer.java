/*
   Java Mixer class for multirotors

   Takes demands U [Throttle, Roll, Pitch, Yaw] and returns motors spins Omega
   or other motor activity (e.g., coaxial servos)

   Copyright(C) 2021 Bennet Ehret, Simon D.Levy

   MIT License
 */

/**
  * This is the mixer for a quadrotor laid out in the ArduPilot configuration:

    Eventually this should be an abstract class allowing us to subclas various configurations
    (including Coaxial)

*/
public class Mixer {

    private static double [][] d = new double[4][4];

    public Mixer()
    {
        //        Th            RR            PF            YR
        d[0][0] = +1; d[0][1] = -1; d[0][2] = -1; d[0][3] = +1; // 1 right front
        d[1][0] = +1; d[1][1] = +1; d[1][2] = +1; d[1][3] = +1; // 2 left rear
        d[2][0] = +1; d[2][1] = +1; d[2][2] = -1; d[2][3] = -1; // 3 left front
        d[3][0] = +1; d[3][1] = -1; d[3][2] = +1; d[3][3] = -1; // 4 right rear
    }

    public double [] getMotors(double [] u)
    {
        double [] omega = new double[4];

        for (int i=0; i<4; ++i) {

            omega[i] = (u[0] * d[i][0] + u[1] * d[i][1] +
                        u[2] * d[i][2] + u[3] * d[i][3]);
        }

        return omega;
    }
}
