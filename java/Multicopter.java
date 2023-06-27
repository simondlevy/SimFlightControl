/*
   Java Multicopter class

   Uses UDP sockets to communicate with MulticopterSim

   Copyright(C) 2019 Simon D.Levy

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

  */

import java.util.Arrays;
import java.lang.Thread;
import java.io.*;
import java.net.DatagramSocket;
import java.net.DatagramPacket;
import java.net.InetAddress;

/**
 * Represents a Multicopter object communicating with MulticopterSim via UDP socket calls.
 */
public class Multicopter {

    // Inner class 
    private class MulticopterThread extends Thread {

        private final int TIMEOUT = 1000;

        public void run()
        {
            while (true) {

                byte [] motorBytes = doublesToBytes(_motorVals);

                DatagramPacket motorPacket = new DatagramPacket(motorBytes, motorBytes.length, _addr, _motorPort);

                try {
                    _motorSocket.send(motorPacket);
                }
                catch (Exception e) {
                    handleException(e);
                }

                byte [] telemetryBytes = new byte [8*_telemetry.length];

                try {
                    DatagramPacket telemetryPacket =
                        new DatagramPacket(telemetryBytes, telemetryBytes.length, _addr, _telemPort);
                    _telemSocket.receive(telemetryPacket);
                }
                catch (Exception e) {
                    handleException(e);
                }

                _telemetry = bytesToDoubles(telemetryBytes);

                if (_telemetry[0] < 0) {
                    break;
                }
            }

            _motorSocket.close();
            _telemSocket.close();

        } // run

        public double [] getTelemetry()
        {
            return  _telemetry;
        }

        private byte[] doublesToBytes(double [] doubles)
        {
            int n = doubles.length;

            byte [] bytes = new byte[8*n];

            for (int i=0; i<n; ++i) {

                long l = Double.doubleToRawLongBits(doubles[i]);

                for (int j=0; j<8; ++j) {
                    bytes[i*8+j] = (byte)((l >> (j*8)) & 0xff);
                }
            }

            return bytes;
        }

        private double[] bytesToDoubles(byte [] bytes)
        {
            int n = bytes.length>>3;

            double [] doubles = new double [n];

            for (int i=0; i<n; ++i) {

                long bits = 0;

                int beg = 8 * i;

                for (int j=0; j<8; ++j) {
                    bits = (bits << 8) | (bytes[beg+8-j-1] & 0xff);
                }

                doubles[i] = Double.longBitsToDouble(bits);
            }

            return doubles;
        }

        public MulticopterThread(String host, int motorPort, int telemetryPort, int motorCount)
        {
            _motorPort = motorPort;
            _telemPort = telemetryPort;

            try {
                _addr = InetAddress.getByName(host);
                _motorSocket = new DatagramSocket();
                _telemSocket = new DatagramSocket(telemetryPort);
                _telemSocket.setSoTimeout(TIMEOUT);
            } 
            catch (Exception e) {
                handleException(e);
            }

            _motorVals = new double [motorCount];

            _telemetry = new double [20]; // Time : State : Demands : Angular velocities
        }

        private int _motorPort;
        private int _telemPort;

        private double [] _motorVals;

        private double [] _telemetry;

        InetAddress _addr;

        private DatagramSocket _motorSocket;
        private DatagramSocket _telemSocket;

        private void handleException(Exception e)
        {
        }

        public void setMotors(double [] motorVals)
        {
            for (int i=0; i<motorVals.length; ++i) {
                _motorVals[i] = motorVals[i];
            }
        }

    } // MulticopterThread

    // ================================================================================

    private MulticopterThread _thread;

    /**
     * Indices for state vector from Bouabdallah (2004)
     */
        public static final int STATE_X = 0;
        public static final int STATE_DX = 1;
        public static final int STATE_Y = 2;
        public static final int STATE_DY = 3;
        public static final int STATE_Z = 4;
        public static final int STATE_DZ = 5;
        public static final int STATE_THETA = 6;
        public static final int STATE_DTHETA = 7;
        public static final int STATE_PHI = 8;
        public static final int STATE_DPHI = 9;
        public static final int STATE_PSI = 10;
        public static final int STATE_DPSI = 11;

    /**
     * Creates a Multicopter object.
     * @param host name of host running MulticopterSim
     * @param motorPort port over which this object will send motor commands to host
     * @param telemeteryPort port over which this object will receive telemetry  from
     * @param motorCount number of motors in vehicle running in simulator on host
     */
    public Multicopter(String host, int motorPort, int telemetryPort, int motorCount)
    {
        _thread = new MulticopterThread(host, motorPort, telemetryPort, motorCount);
    }

    /**
     * Creates a Multicopter object using a default number of motors (4).
     * @param host name of host running MulticopterSim
     * @param motorPort port over which this object will send motor commands to host
     * @param telemeteryPort port over which this object will receive telemetry  from
     */
    public Multicopter(String host, int motorPort, int telemetryPort)
    {
        _thread = new MulticopterThread(host, motorPort, telemetryPort, 4);
    }

    /**
     * Creates a Multicopter object using default parameters.
     */
    public Multicopter()
    {
        _thread = new MulticopterThread("127.0.0.1", 5000, 5001, 4);
    }

    /**
     * Begins communication with simulator running on host.
     */
    public void start()
    {
        System.out.println("Hit the Play button ...");
        _thread.start();
    }

    /**
     * Returns current time.
     * @return current time
     */
    public double getTime()
    {
        return _thread.getTelemetry()[0];
    }

    /**
     * Returns current vehicle state as an array of the form [x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi].
     * @return vehicle state 
     */
    public double [] getVehicleState()
    {
        return Arrays.copyOfRange(_thread.getTelemetry(), 1, 13);
    }

    /**
     * Sets motor values.
     * @param motorVals array of values between 0 and 1
     */
    public void setMotors(double [] motorVals)
    {
        _thread.setMotors(motorVals);
    }
}
