/*------------------------------------------------------------------------

 Author      : Sérgio Patrão
 Date        : 11 Fevereiro 2015
 Version     : 1.0
 Description : Sensor Fusion Library

------------------------------------------------------------------------*/	

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MatrixLibrary;

    public class SensorFusion
    {
        #region Conversão Degrees/Rad
        /// <summary>
        /// Degrees to Radians
        /// </summary>
        /// <param name="angle">Angle or angular velocit in degrees</param> 
        /// <returns></returns>
        public static double ConvertToRadians(double angle)
        {
            return (Math.PI / 180) * angle;
        }

        /// <summary>
        /// Radians to Degrees
        /// </summary>
        /// <param name="angle">Angle or angular velocity in Radians</param> 
        /// <returns></returns>
        public static double ConvertToDegrees(double angle)
        {
            return (180 / Math.PI) * angle;
        }
        #endregion

        #region Quaternion Math       
        /// <summary>
        /// Quaternion multiplication between two quaternions
        /// </summary>
        /// <param name="p"> First Quaternion </param> 
        /// <param name="q"> Second Quaternion </param>
        /// <returns> Quaternion </returns>
        private static double[,] QuaternionMultiply(double[,] p, double[,] q)
        {
            double[,] r = new double[4, 1];

            r[0, 0] = p[0, 0] * q[0, 0] - p[1, 0] * q[1, 0] - p[2, 0] * q[2, 0] - p[3, 0] * q[3, 0];
            r[1, 0] = p[0, 0] * q[1, 0] + p[1, 0] * q[0, 0] + p[2, 0] * q[3, 0] - p[3, 0] * q[2, 0];
            r[2, 0] = p[0, 0] * q[2, 0] - p[1, 0] * q[3, 0] + p[2, 0] * q[0, 0] + p[3, 0] * q[1, 0];
            r[3, 0] = p[0, 0] * q[3, 0] + p[1, 0] * q[2, 0] - p[2, 0] * q[1, 0] + p[3, 0] * q[0, 0];

            return r;
        }

        /// <summary>
        /// Add two quaternions
        /// </summary>
        /// <param name="p"> First quaternion </param>
        /// <param name="q"> Second quaternion </param>
        /// <returns>Quaternion</returns>
        public static double[,] QuaternionAdd(double[,] p, double[,] q)
        {
            double[,] r = new double[4, 1];

            r[0, 0] = p[0, 0] + q[0, 0];
            r[1, 0] = p[1, 0] + q[1, 0];
            r[2, 0] = p[2, 0] + q[2, 0];
            r[3, 0] = p[3, 0] + q[3, 0];

            return r;
        }

        /// <summary>
        /// Subtract two quaternions
        /// </summary>
        /// <param name="p"> First quaternion </param>
        /// <param name="q"> Second quaternion </param>
        /// <returns> Quaternion </returns>
        private static double[,] QuaternionSub(double[,] p, double[,] q)
        {
            double[,] r = new double[4, 1];
            r[0, 0] = p[0, 0] - q[0, 0];
            r[1, 0] = p[1, 0] - q[1, 0];
            r[2, 0] = p[2, 0] - q[2, 0];
            r[3, 0] = p[3, 0] - q[3, 0];
            return r;
        }

        /// <summary>
        /// Normalize a Quaternion
        /// </summary>
        /// <param name="q"> Quaternion to normalize</param>
        /// <returns> Quaternion </returns>
        public static double[,] QuaternionNormalize(double[,] q)
        {
            double norm;

            norm = Math.Sqrt(q[0, 0] * q[0, 0] + q[1, 0] * q[1, 0] + q[2, 0] * q[2, 0] + q[3, 0] * q[3, 0]);

            double[,] r = new double[4, 1];

            r[0, 0] = q[0, 0] / norm;
            r[1, 0] = q[1, 0] / norm;
            r[2, 0] = q[2, 0] / norm;
            r[3, 0] = q[3, 0] / norm;

            return r;
        }

        /// <summary>
        /// Find the conjugate of a quaternion
        /// </summary>
        /// <param name="q"> Quaternion to find its conjugate </param>
        /// <returns> Quaternion </returns>
        private static double[,] QuaternionConjugate(double[,] q)
        {
            double[,] r = new double[4, 1];

            r[0, 0] = q[0, 0];
            r[1, 0] = -q[1, 0];
            r[2, 0] = -q[2, 0];
            r[3, 0] = -q[3, 0];

            return r;
        }

        /// <summary>
        /// Find the Euler angles from quaternion
        /// </summary>
        /// <param name="q"> Quaternion </param>
        /// <returns> Euler angles in radians - r[0,0] = roll, r[1,0] = pitch, r[2,0] = yaw </returns>
        public static double[,] QuaternionToEuler(double[,] q)
        {
            double q0 = q[0, 0];
            double q1 = q[1, 0];
            double q2 = q[2, 0];
            double q3 = q[3, 0];
            double[,] result = new double[3, 1];

            result[0, 0] = Math.Atan2((2 * (q2 * q3 + q0 * q1)), (1 - 2 * (q1 * q1 + q2 * q2)));
            result[1, 0] = Math.Asin(2 * (q0 * q2 - q1 * q3));
            result[2, 0] = Math.Atan2((2 * (q1 * q2 + q0 * q3)), (1 - 2 * (q2 * q2 + q3 * q3)));

            return result;
        }

        /// <summary>
        /// Fin the quaternion from Euler angles
        /// </summary>
        /// <param name="roll"> Euler angle roll in Radians </param>
        /// <param name="pitch"> Euler angle pitch in Radians </param>
        /// <param name="yaw"> Euler angle yaw in Radians </param>
        /// <returns> Quaternion </returns>
        public static double[,] EulerToQuaternion(double roll, double pitch, double yaw)
        {
            double cosX = Math.Cos(roll / 2.0f);
            double sinX = Math.Sin(roll / 2.0f);
            double cosY = Math.Cos(pitch / 2.0f);
            double sinY = Math.Sin(pitch / 2.0f);
            double cosZ = Math.Cos(yaw / 2.0f);
            double sinZ = Math.Sin(yaw / 2.0f);
            double[,] q = new double[4, 1]; ;

            q[0, 0] = cosX * cosY * cosZ + sinX * sinY * sinZ;
            q[1, 0] = sinX * cosY * cosZ - cosX * sinY * sinZ;
            q[2, 0] = cosX * sinY * cosZ + sinX * cosY * sinZ;
            q[3, 0] = cosX * cosY * sinZ - sinX * sinY * cosZ;

            q = QuaternionNormalize(q);

            return q;
        }
        #endregion

        #region Data directly from Sensors
        /// <summary>
        /// Find Euler angles (Roll and Pitch) from accelerometer data
        /// </summary>
        /// <param name="accx"> X-axis accelerometer data in g </param>
        /// <param name="accy"> Y-axis accelerometer data in g </param>
        /// <param name="accz"> Z-axis accelerometer data in g </param>
        /// <param name="roll"> Out param - Roll in Degrees </param>
        /// <param name="pitch"> Out param - Pitch in Degrees </param>
        public static void EulerFromAcc(double accx, double accy, double accz, out double roll, out double pitch)
        {
            roll = ConvertToDegrees(Math.Atan2(accy, Math.Sign(accz) * Math.Sqrt(accz * accz + 0.01 * accx * accx)));
            pitch = ConvertToDegrees(Math.Atan(-accx / Math.Sqrt(accy * accy + accz * accz)));
        }

        /// <summary>
        /// Get heading (yaw angle) from magnetometer.
        /// </summary>
        /// <param name="magx"> X-axis magnetometer data (uT or gauss) </param>
        /// <param name="magy"> Y-axis magnetometer data (uT or gauss) </param>
        /// <param name="heading"> Out param - Heading or Yaw angle in Degrees </param>
        public static void HeadingFromCompass(double magx, double magy, out double heading)
        {
            heading = ConvertToDegrees(Math.Atan2(-magy, magx));
        }

        /// <summary>
        /// Get Euler angles from gyroscope data
        /// </summary>
        /// <param name="gyrx"> X-axis gyroscope data in degrees/s </param>
        /// <param name="gyry"> Y-axis gyroscope data in degrees/s </param>
        /// <param name="gyrz"> Z-axis gyroscope data in degrees/s </param>
        /// <param name="roll"> Euler angle Roll from a previous iteration - degrees </param>
        /// <param name="pitch"> Euler angle Pitch from a previous iteration - degrees </param>
        /// <param name="yaw"> Euler angle Yaw from a previous iteration - degrees </param>
        /// <param name="rollGyro"> Out param - Euler angle Roll from gyroscope data - degrees </param>
        /// <param name="pitchGyro"> Out param - Euler angle Pitch from gyroscope data - degrees </param>
        /// <param name="yawGyro"> Out param - Euler angle Yaw from gyroscope data - degrees </param>
        /// <param name="dt"> Time elapsed between iterations </param>
        public static void EulerFromGyro(double gyrx, double gyry, double gyrz, double roll, double pitch, double yaw, out double rollGyro, out double pitchGyro, out double yawGyro, double dt)
        {
            gyrx = ConvertToRadians(gyrx);
            gyry = ConvertToRadians(gyry);
            gyrz = ConvertToRadians(gyrz);
            roll = ConvertToRadians(roll);
            pitch = ConvertToRadians(pitch);
            yaw = ConvertToRadians(yaw);

            rollGyro = roll + dt * (gyrx + gyry * Math.Cos(roll) * Math.Tan(pitch) + gyrz * Math.Sin(roll) * Math.Tan(pitch));
            pitchGyro = pitch + dt * (gyry * Math.Cos(roll) - gyrz * Math.Sin(roll));
            yawGyro = yaw + dt * (gyry * Math.Sin(roll) / Math.Cos(pitch) + gyrz * Math.Cos(roll) / Math.Cos(pitch));

            rollGyro = ConvertToDegrees(rollGyro);
            pitchGyro = ConvertToDegrees(pitchGyro);
            yawGyro = ConvertToDegrees(yawGyro);
        }

        /// <summary>
        /// Remove gravity components from accelerometer measurements with Euler angles and returns Sensor frame linear accelerations
        /// </summary>
        /// <param name="accx"> X-axis accelerometer data - g </param>
        /// <param name="accy"> Y-axis accelerometer data - g </param>
        /// <param name="accz"> Z-axis accelerometer data - g </param>
        /// <param name="roll"> Euler angle Roll - degrees </param>
        /// <param name="pitch"> Euler angle Pitch - degrees </param>
        /// <param name="yaw"> Euler angle Yaw - degrees </param>
        /// <param name="RealAccx"> Out param - Sensor frame X-axis linear acceleration - g </param>
        /// <param name="RealAccy"> Out param - Sensor frame Y-axis linear acceleration - g </param>
        /// <param name="RealAccz"> Out param - Sensor frame Z-axis linear acceleration - g </param>
        public static void RemoveGravitySensorFrame(double accx, double accy, double accz, double roll, double pitch, double yaw, out double RealAccx, out double RealAccy, out double RealAccz)
        {
            #region Declaração de Variáveis Necessárias
            double[,] R1 = new double[3, 3] { {Math.Cos(ConvertToRadians(-yaw)), Math.Sin(ConvertToRadians(-yaw)),0},
                                              {-Math.Sin(ConvertToRadians(-yaw)),Math.Cos(ConvertToRadians(-yaw)),0},
                                              {0,0,1 } };

            double[,] R2 = new double[3, 3] { {Math.Cos(ConvertToRadians(-pitch)),0, -Math.Sin(ConvertToRadians(-pitch))},
                                              {0,1,0},
                                              {Math.Sin(ConvertToRadians(-pitch)),0,Math.Cos(ConvertToRadians(-pitch))}};

            double[,] R3 = new double[3, 3] { {1,0,0},
                                              {0,Math.Cos(ConvertToRadians(-roll)),Math.Sin(ConvertToRadians(-roll))},
                                              {0,-Math.Sin(ConvertToRadians(-roll)), Math.Cos(ConvertToRadians(-roll))}};

            double[,] R4 = new double[3, 3] { {Math.Cos(ConvertToRadians(yaw)), Math.Sin(ConvertToRadians(yaw)),0},
                                              {-Math.Sin(ConvertToRadians(yaw)),Math.Cos(ConvertToRadians(yaw)),0},
                                              {0,0,1 } };

            double[,] R5 = new double[3, 3] { {Math.Cos(ConvertToRadians(pitch)),0, -Math.Sin(ConvertToRadians(pitch))},
                                              {0,1,0},
                                              {Math.Sin(ConvertToRadians(pitch)),0,Math.Cos(ConvertToRadians(pitch))}};

            double[,] R6 = new double[3, 3] { {1,0,0},
                                              {0,Math.Cos(ConvertToRadians(roll)),Math.Sin(ConvertToRadians(roll))},
                                              {0,-Math.Sin(ConvertToRadians(roll)), Math.Cos(ConvertToRadians(roll))}};

            double[,] AccTotal = new double[3, 1] { { accx }, { accy }, { accz } };
            double[,] Gravity = new double[3, 1] { { 0.0f }, { 0.0f }, { 1.0f } };
            #endregion

            double[,] AccInertial = Matrix.Multiply(Matrix.Multiply(Matrix.Multiply(R1, R2), R3), AccTotal);
            AccInertial = Matrix.Subtract(AccInertial, Gravity);
            double[,] RealAcc = Matrix.Multiply(Matrix.Multiply(Matrix.Multiply(R6, R5), R4), AccInertial);
            RealAccx = RealAcc[0, 0];
            RealAccy = RealAcc[1, 0];
            RealAccz = RealAcc[2, 0];
        }

        /// <summary>
        /// Remove gravity components from accelerometer measurements with Euler angles and returns Earth frame linear accelerations
        /// </summary>
        /// <param name="accx"> X-axis accelerometer data - g </param>
        /// <param name="accy"> Y-axis accelerometer data - g </param>
        /// <param name="accz"> Z-axis accelerometer data - g </param>
        /// <param name="roll"> Euler angle Roll - degrees </param>
        /// <param name="pitch"> Euler angle Pitch - degrees </param>
        /// <param name="yaw"> Euler angle Yaw - degrees </param>
        /// <param name="RealAccx"> Out param - Earth frame X-axis linear acceleration - g </param>
        /// <param name="RealAccy"> Out param - Earth frame Y-axis linear acceleration - g </param>
        /// <param name="RealAccz"> Out param - Earth frame Z-axis linear acceleration - g </param>
        public static void RemoveGravityEarthFrame(double accx, double accy, double accz, double roll, double pitch, double yaw, out double RealAccx, out double RealAccy, out double RealAccz)
        {
            #region Declaração de Variáveis Necessárias
            double[,] R1 = new double[3, 3] { {Math.Cos(ConvertToRadians(-yaw)), Math.Sin(ConvertToRadians(-yaw)),0},
                                              {-Math.Sin(ConvertToRadians(-yaw)),Math.Cos(ConvertToRadians(-yaw)),0},
                                              {0,0,1 } };

            double[,] R2 = new double[3, 3] { {Math.Cos(ConvertToRadians(-pitch)),0, -Math.Sin(ConvertToRadians(-pitch))},
                                              {0,1,0},
                                              {Math.Sin(ConvertToRadians(-pitch)),0,Math.Cos(ConvertToRadians(-pitch))}};

            double[,] R3 = new double[3, 3] { {1,0,0},
                                              {0,Math.Cos(ConvertToRadians(-roll)),Math.Sin(ConvertToRadians(-roll))},
                                              {0,-Math.Sin(ConvertToRadians(-roll)), Math.Cos(ConvertToRadians(-roll))}};

            double[,] AccTotal = new double[3, 1] { { accx }, { accy }, { accz } };
            double[,] Gravity = new double[3, 1] { { 0.0f }, { 0.0f }, { 1.0f } };
            #endregion

            double[,] AccInertial = Matrix.Multiply(Matrix.Multiply(Matrix.Multiply(R1, R2), R3), AccTotal);
            AccInertial = Matrix.Subtract(AccInertial, Gravity);
            RealAccx = AccInertial[0, 0];
            RealAccy = AccInertial[1, 0];
            RealAccz = AccInertial[2, 0];
        }


        /// <summary>
        /// Remove gravity components from accelerometer measurements with Quaternions and returns Sensor frame linear accelerations
        /// </summary>
        /// <param name="accx"> X-axis accelerometer data -g </param>
        /// <param name="accy"> Y-axis accelerometer data -g </param>
        /// <param name="accz"> Z-axis accelerometer data -g </param>
        /// <param name="q"> Quaternion </param>
        /// <param name="RealAccx"> Out param - Sensor frame X-axis linear acceleration - g </param>
        /// <param name="RealAccy"> Out param - Sensor frame Y-axis linear acceleration - g </param>
        /// <param name="RealAccz"> Out param - Sensor frame Z-axis linear acceleration - g </param>
        public static  void RemoveGravityQuatSensorFrame(double accx, double accy, double accz, double[,] q, out double RealAccx, out double RealAccy, out double RealAccz)
        {
            double[,] qGravityF;
            double[,] AccTotal = new double[4, 1] { { 0.0f }, { accx }, { accy }, { accz } };
            double[,] Gravity = new double[4, 1] { { 0.0f }, { 0.0f }, { 0.0f }, { 1.0f } };
            double[,] quatConj, temp1;

            quatConj = QuaternionConjugate(q);
            temp1 = QuaternionMultiply(AccTotal, quatConj);
            qGravityF = QuaternionMultiply(q, temp1);
            qGravityF = QuaternionSub(qGravityF, Gravity);

            RealAccx = qGravityF[1, 0];
            RealAccy = qGravityF[2, 0];
            RealAccz = qGravityF[3, 0];
        }

        /// <summary>
        /// Remove gravity components from accelerometer measurements with Quaternions and returns Earth frame linear accelerations
        /// </summary>
        /// <param name="accx"> X-axis accelerometer data - g </param>
        /// <param name="accy"> Y-axis accelerometer data - g </param>
        /// <param name="accz"> Z-axis accelerometer data - g </param>
        /// <param name="q"> Quaternion </param>
        /// <param name="RealAccx"> Out param - Earth frame X-axis linear acceleration - g </param>
        /// <param name="RealAccy"> Out param - Earth frame Y-axis linear acceleration - g </param>
        /// <param name="RealAccz"> Out param - Earth frame Z-axis linear acceleration - g </param>
        public static void RemoveGravityQuatEarthFrame(double accx, double accy, double accz, double[,] q, out double RealAccx, out double RealAccy, out double RealAccz)
        {
            double[,] qGravityF;
            double[,] AccTotal = new double[4, 1] { { 0.0f }, { accx }, { accy }, { accz } };
            double[,] Gravity = new double[4, 1] { { 0.0f }, { 0.0f }, { 0.0f }, { 1.0f } };
            double[,] quatConj, temp1, temp2;

            quatConj = QuaternionConjugate(q);

            temp1 = QuaternionMultiply(AccTotal, quatConj);
            qGravityF = QuaternionMultiply(q, temp1);
            qGravityF = QuaternionSub(qGravityF, Gravity);

            temp2 = QuaternionMultiply(qGravityF, q);
            qGravityF = QuaternionMultiply(quatConj, temp2);

            RealAccx = qGravityF[1, 0];
            RealAccy = qGravityF[2, 0];
            RealAccz = qGravityF[3, 0];
        }

        /// <summary>
        /// Tilt compensation of magnetometer data
        /// </summary>
        /// <param name="qmag"> Pure quaternion constructed from magnetometer data </param>
        /// <param name="quat"> Quaternion (only considering roll and pitch) </param>
        /// <param name="qmagF"> Pure quaterion with vectorial part equal to the new magnetometer vector data </param>
        public static void TiltCompensationCompass(double[,] qmag, double[,] quat, out double[,] qmagF)
        {
            double[,] quatConj, temp1;

            quatConj = QuaternionConjugate(quat);
            temp1 = QuaternionMultiply(qmag, quatConj);
            qmagF = QuaternionMultiply(quat, temp1);
        }
        #endregion

        #region Filtro de Kalman Estendido - IMU
        /// <summary>
        /// Extended Kalman filter for IMUs (accelerometer and gyroscope data only).
        /// </summary>
        /// <param name="q"> Quaternion from previous iteration </param>
        /// <param name="p"> Covariance matrix from previous iteration </param>
        /// <param name="Q"></param>
        /// <param name="R1"></param>
        /// <param name="dt"> Time elapsed between iterations </param>
        /// <param name="accx"> X-axis accelerometer data - g </param>
        /// <param name="accy"> Y-axis accelerometer data - g </param>
        /// <param name="accz"> Z-axis accelerometer data - g </param>
        /// <param name="gyrx"> X-axis gyroscope data - degrees/s </param>
        /// <param name="gyry"> Y-axis gyroscope data - degrees/s </param>
        /// <param name="gyrz"> Z-axis gyroscope data - degrees/s </param>
        /// <param name="qEKF"> Out param - Estimated Quaternion </param>
        /// <param name="pEKF"> Out param - New Covariance Matrix </param>
        public static void EKFimu(double[,] q, double[,] p, double[,] Q, double[,] R1, double dt, double accx, double accy, double accz, double gyrx, double gyry, double gyrz, out double[,] qEKF, out double[,] pEKF)
        {
            gyrx = ConvertToRadians(gyrx);
            gyry = ConvertToRadians(gyry);
            gyrz = ConvertToRadians(gyrz);


            double[,] V = new double[3, 3] {{1.0f,0.0f,0.0f},
                                          {0.0f,1.0f,0.0f},
                                          {0.0f,0.0f,1.0f}};

            double[,] I = new double[4, 4] {{1.0f,0.0f,0.0f,0.0f},
                                          {0.0f,1.0f,0.0f,0.0f},
                                          {0.0f,0.0f,1.0f,0.0f},
                                          {0.0f,0.0f,0.0f,1.0f}};
            double[,] gyr = new double[4, 4] { { 0, -gyrx, -gyry, -gyrz }, { gyrx, 0, gyrz, -gyry }, { gyry, -gyrz, 0, gyrx }, { gyrz, gyry, -gyrx, 0 } };

            double[,] Ak = Matrix.Add(I, Matrix.ScalarMultiply(0.5f * dt, gyr));
            double[,] q_e = Matrix.Multiply(Ak, q);
            q_e = QuaternionNormalize(q_e);
            double[,] p_e = Matrix.Add(Matrix.Multiply(Matrix.Multiply(Ak, p), Matrix.Transpose(Ak)), Q);


            double[,] z1 = new double[3, 1] { { accx }, { accy }, { accz } };
            double[,] h_1 = new double[3, 1] {{2*q_e[1,0]*q_e[3,0]-2*q_e[0,0]*q_e[2,0]},
                                              {2*q_e[0,0]*q_e[1,0]+2*q_e[2,0]*q_e[3,0]},
                                              {2*(-0.5f+q_e[0,0]*q_e[0,0]+q_e[3,0]*q_e[3,0])}};
            double[,] H1 = new double[3, 4] {{-2*q_e[2,0] , 2*q_e[3,0] , -2*q_e[0,0] , 2*q_e[1,0]},
                                             {2*q_e[1,0]  , 2*q_e[0,0] , 2*q_e[3,0]  , 2*q_e[2,0]},
                                             {4*q_e[0,0]  , 0          , 0           , 4*q_e[3,0]}};

            double[,] Inv1 = Matrix.Inverse(Matrix.Add(Matrix.Multiply(Matrix.Multiply(H1, p_e), Matrix.Transpose(H1)), Matrix.Multiply(Matrix.Multiply(V, R1), Matrix.Transpose(V))));
            double[,] K1 = Matrix.Multiply(Matrix.Multiply(p_e, Matrix.Transpose(H1)), Inv1);
            double[,] qe1 = Matrix.Multiply(K1, Matrix.Subtract(z1, h_1));
            qEKF = QuaternionAdd(q_e, qe1);
            qEKF = QuaternionNormalize(qEKF);
            pEKF = Matrix.Multiply(Matrix.Subtract(I, Matrix.Multiply(K1, H1)), p_e);

        }
        #endregion

        #region Extended Kalman Filter - Double Stage
        /// <summary>
        /// Extended Kalman filter for AHRS (accelerometer, gyroscope and magnetometer)
        /// </summary>
        /// <param name="p"> Covariance matrix from previous iterations </param>
        /// <param name="q"> Quaternion from previous iteration </param>
        /// <param name="Q"></param>
        /// <param name="R1"></param>
        /// <param name="R2"></param>
        /// <param name="dt"> Time elapsed between iterations </param>
        /// <param name="gyrx"> X-axis gyroscope data - degrees/s </param>
        /// <param name="gyry"> Y-axis gyroscope data - degrees/s </param>
        /// <param name="gyrz"> Z-axis gyroscope data - degrees/s </param>
        /// <param name="accx"> X-axis accelerometer data - g </param>
        /// <param name="accy"> Y-axis accelerometer data - g </param>
        /// <param name="accz"> Z-axis accelerometer data - g </param>
        /// <param name="magx"> X-axis magnetometer data - uT </param>
        /// <param name="magy"> Y-axis magnetometer data - uT </param>
        /// <param name="magz"> Z-axis magnetometer data - uT </param>
        /// <param name="qEKF"> Out param - Estimated quaternion </param>
        /// <param name="pEKF"> Out param - New Covariance matrix </param>
        public static void DoubleStageEKF(double[,] p, double[,] q, double[,] Q, double[,] R1, double[,] R2, double dt, double gyrx, double gyry, double gyrz, double accx, double accy, double accz, double magx, double magy, double magz, out double[,] qEKF, out double[,] pEKF)
        {
            gyrx = ConvertToRadians(gyrx);
            gyry = ConvertToRadians(gyry);
            gyrz = ConvertToRadians(gyrz);
            #region Declaração de Variáveis

            double[,] V = new double[3, 3] {{1.0f,0   ,0   },
                                            {0   ,1.0f,0   },
                                            {0   ,0   ,1.0f}};
            double[,] I = new double[4, 4] {{1.0f,0   ,0   ,0   },
                                            {0   ,1.0f,0   ,0   },
                                            {0   ,0   ,1.0f,0   },
                                            {0   ,0   ,0   ,1.0f}};




            double[,] gyr = new double[4, 4] { { 0   ,-gyrx,-gyry,-gyrz },
                                               { gyrx,0    ,gyrz ,-gyry },
                                               { gyry,-gyrz,0    ,gyrx  },
                                               { gyrz,gyry ,-gyrx,0     }};
            #endregion

            //Estágio 0
            double[,] Ak = Matrix.Add(I, Matrix.ScalarMultiply(0.5f * dt, gyr));
            double[,] q_e = Matrix.Multiply(Ak, q);
            double[,] p_e = Matrix.Add(Matrix.Multiply(Matrix.Multiply(Ak, p), Matrix.Transpose(Ak)), Q);

            //Estágio 1
            double[,] z1 = new double[3, 1] { { accx }, { accy }, { accz } };

            double[,] h_1 = new double[3, 1] {{2*q_e[1,0]*q_e[3,0]-2*q_e[0,0]*q_e[2,0]},
                                              {2*q_e[0,0]*q_e[1,0]+2*q_e[2,0]*q_e[3,0]},
                                              {q_e[0,0]*q_e[0,0]-q_e[1,0]*q_e[1,0]-q_e[2,0]*q_e[2,0]+q_e[3,0]*q_e[3,0]}};
            double[,] H1 = new double[3, 4] {{-2*q_e[2,0] , 2*q_e[3,0] , -2*q_e[0,0] , 2*q_e[1,0]},
                                             {2*q_e[1,0]  , 2*q_e[0,0] , 2*q_e[3,0]  , 2*q_e[2,0]},
                                             {2*q_e[0,0]  , -2*q_e[1,0]          , -2*q_e[2,0]           , 2*q_e[3,0]}};




            double[,] Inv1 = Matrix.Inverse(Matrix.Add(Matrix.Multiply(Matrix.Multiply(H1, p_e), Matrix.Transpose(H1)), Matrix.Multiply(Matrix.Multiply(V, R1), Matrix.Transpose(V))));
            double[,] K1 = Matrix.Multiply(Matrix.Multiply(p_e, Matrix.Transpose(H1)), Inv1);
            double[,] qe1 = Matrix.Multiply(K1, Matrix.Subtract(z1, h_1));
            double[,] qk1 = (QuaternionAdd(q_e, qe1));
            double[,] p1 = Matrix.Multiply(Matrix.Subtract(I, Matrix.Multiply(K1, H1)), p_e);

            //Estágio 2
            double[,] z2 = new double[3, 1] { { magx }, { magy }, { magz } };
            double norm = Math.Sqrt(magx * magx + magy * magy + magz * magz);
            z2 = Matrix.ScalarDivide(norm, z2);


            double[,] quatmag = new double[4, 1] { { 0 }, { z2[0, 0] }, { z2[1, 0] }, { z2[2, 0] } };
            double[,] h = QuaternionMultiply(q_e, QuaternionMultiply(quatmag, QuaternionConjugate(q_e)));
            double[,] b = new double[4, 1] { { 0 }, { Math.Sqrt(h[1, 0] * h[1, 0] + h[2, 0] * h[2, 0]) }, { 0 }, { h[3, 0] } };
            b = QuaternionNormalize(b);

            double[,] h_2 = new double[3, 1]{  {2*b[1,0]*(0.5-q_e[2,0]*q_e[2,0]-q_e[3,0]*q_e[3,0])+2*b[3,0]*(q_e[1,0]*q_e[3,0]-q_e[0,0]*q_e[2,0])-q_e[0,0]*q_e[2,0]},
                                             {2*b[1,0]*(q_e[1,0]*q_e[2,0]-q_e[0,0]*q_e[3,0])+2*b[3,0]*(q_e[0,0]*q_e[1,0]+q_e[2,0]*q_e[3,0])},
                                             {2*b[1,0]*(q_e[0,0]*q_e[2,0]+q_e[1,0]*q_e[3,0])+2*b[3,0]*(0.5-q_e[1,0]*q_e[1,0]-q_e[2,0]*q_e[2,0])}};
            double[,] H2 = new double[3, 4]{{-2*b[3,0]*q_e[2,0], 2*b[3,0]*q_e[3,0] , -4*b[1,0]*q_e[2,0]-2*b[3,0]*q_e[0,0] , -4*b[1,0]*q_e[3,0]+2*b[3,0]*q_e[1,0] },
                                          { -2*b[1,0]*q_e[3,0]+2*b[3,0]*q_e[1,0], 2*b[1,0]*q_e[2,0]+2*b[3,0]*q_e[0,0] , 2*b[1,0]*q_e[1,0]+2*b[3,0]*q_e[3,0] ,-2*b[1,0]*q_e[0,0]+2*b[3,0]*q_e[2,0] },
                                          { 2*b[1,0]*q_e[2,0], 2*b[1,0]*q_e[3,0]-4*b[3,0]*q_e[1,0] ,2*b[1,0]*q_e[0,0]-4*b[3,0]*q_e[2,0] ,2*b[1,0]*q_e[1,0] }};



            double[,] Inv2 = Matrix.Inverse(Matrix.Add(Matrix.Multiply(Matrix.Multiply(H2, p_e), Matrix.Transpose(H2)), Matrix.Multiply(Matrix.Multiply(V, R2), Matrix.Transpose(V))));
            double[,] K2 = Matrix.Multiply(Matrix.Multiply(p_e, Matrix.Transpose(H2)), Inv2);
            double[,] qe2 = Matrix.Multiply(K2, Matrix.Subtract(z2, h_2));
            qEKF = QuaternionAdd(qk1, qe2);
            qEKF = QuaternionNormalize(qEKF);
            pEKF = Matrix.Multiply(Matrix.Subtract(I, Matrix.Multiply(K2, H2)), p1);

        }
        #endregion

        #region Gauss-Newton
        /// <summary>
        /// Complementary Filter for Gauss-Newton method
        /// </summary>
        /// <param name="qFilt"> Quaternion from previous Complementary Filter </param>
        /// <param name="qObs"> Observed quaternion from Gauss-Newton method </param>
        /// <param name="gain"> Complementary Filter Gain </param>
        /// <param name="dt"> Time elapsed between iterations </param>
        /// <param name="accx"> X-axis accelerometer data - g </param>
        /// <param name="accy"> Y-axis accelerometer data - g </param>
        /// <param name="accz"> Z-axis accelerometer data - g </param>
        /// <param name="magx"> X-axis magnetometer data - gauss </param>
        /// <param name="magy"> Y-axis magnetometer data - gauss </param>
        /// <param name="magz"> Z-axis magnetometer data - gauss </param>
        /// <param name="gyrx"> X-axis gyroscope data - degrees/s </param>
        /// <param name="gyry"> Y-axis gyroscope data - degrees/s </param>
        /// <param name="gyrz"> Z-axis gyroscope data - degrees/s </param>
        /// <param name="qComplementar"> Out param - Estimated quaternion </param>
        /// <param name="qOsserv"> Out param - New observation Quaternion from Gauss-Newton method </param>
        public static void GaussNewtonComplementar(double[,] qFilt, double[,] qObs, double gain, double dt, double accx, double accy, double accz, double magx, double magy, double magz, double gyrx, double gyry, double gyrz, out double[,] qComplementar, out double[,] qOsserv)
        {
            gyrx = ConvertToRadians(gyrx);
            gyry = ConvertToRadians(gyry);
            gyrz = ConvertToRadians(gyrz);

            double[,] qGyro = { { 0 }, { gyrx }, { gyry }, { gyrz } };
            double[,] dq = Matrix.ScalarMultiply(0.5f, QuaternionMultiply(qFilt, qGyro));
            double[,] qGyroFilt = QuaternionAdd(qFilt, Matrix.ScalarMultiply(dt, dq));
            qGyroFilt = QuaternionNormalize(qGyroFilt);

            GaussNewton(qObs, accx, accy, accz, magx, magy, magz, out qOsserv);
            qOsserv = QuaternionNormalize(qOsserv);
            qComplementar = QuaternionAdd(Matrix.ScalarMultiply(gain, qGyroFilt), Matrix.ScalarMultiply(1 - gain, qOsserv));
            qComplementar = QuaternionNormalize(qComplementar);
        }

        /// <summary>
        /// Kalman filter for Gauss-Newton method
        /// </summary>
        /// <param name="qUpdate"> Quaternion from previous iteration </param>
        /// <param name="pUpdate"> Covariance matrix from previous iteration </param>
        /// <param name="Q"></param>
        /// <param name="R"></param>
        /// <param name="dt"> Time elapsed between iteration </param>
        /// <param name="accx"> X-axis accelerometer data - g </param>
        /// <param name="accy"> Y-axis accelerometer data - g </param>
        /// <param name="accz"> Z-axis accelerometer data - g </param>
        /// <param name="magx"> X-axis magnetometer data - gauss </param>
        /// <param name="magy"> Y-axis magnetometer data - gauss </param>
        /// <param name="magz"> Z-axis magnetometer data - gauss </param>
        /// <param name="gyrx"> X-axis gyroscope data - degrees/s </param>
        /// <param name="gyry"> Y-axis gyroscope data - degrees/s </param>
        /// <param name="gyrz"> Z-axis gyroscope data - degrees/s </param>
        /// <param name="qGNKF"> Estimated quaternion </param>
        /// <param name="pGNKF"> New covariance matrix </param>
        public static void GaussNewtonKalmanFilter(double[,] qUpdate, double[,] pUpdate, double[,] Q, double[,] R, double dt, double accx, double accy, double accz, double magx, double magy, double magz, double gyrx, double gyry, double gyrz, out double[,] qGNKF, out double[,] pGNKF)
        {
            gyrx = ConvertToRadians(gyrx);
            gyry = ConvertToRadians(gyry);
            gyrz = ConvertToRadians(gyrz);
            #region DefiniçãoVariáveis

            double[,] I = new double[4, 4] {{1.0f,0   ,0   ,0   },
                                            {0   ,1.0f,0   ,0   },
                                            {0   ,0   ,1.0f,0   },
                                            {0   ,0   ,0   ,1.0f}};

            double[,] H = new double[4, 4] {{1.0f,0   ,0   ,0   },
                                            {0   ,1.0f,0   ,0   },
                                            {0   ,0   ,1.0f,0   },
                                            {0   ,0   ,0   ,1.0f}};
            double[,] gyr = new double[4, 4] { { 0   ,-gyrx,-gyry,-gyrz },
                                               { gyrx,0    ,gyrz ,-gyry },
                                               { gyry,-gyrz,0    ,gyrx  },
                                               { gyrz,gyry ,-gyrx,0     }};
            #endregion
            double[,] Ak = Matrix.Add(I, Matrix.ScalarMultiply(0.5f * dt, gyr));
            double[,] qPredicted = Matrix.Multiply(Ak, qUpdate);
            double[,] qOsserv;

            GaussNewton(qUpdate, accx, accy, accz, magx, magy, magz, out qOsserv);
            qOsserv = QuaternionNormalize(qOsserv);

            double[,] p_e = Matrix.Add(Matrix.Multiply(Matrix.Multiply(Ak, pUpdate), Matrix.Transpose(Ak)), Q);

            double[,] Inv1 = Matrix.Inverse(Matrix.Add(Matrix.Multiply(Matrix.Multiply(H, p_e), Matrix.Transpose(H)), R));
            double[,] K = Matrix.Multiply(Matrix.Multiply(p_e, Matrix.Transpose(H)), Inv1);
            qGNKF = Matrix.Add(qPredicted, (Matrix.Multiply(K, Matrix.Subtract(qOsserv, qPredicted))));
            pGNKF = Matrix.Multiply(Matrix.Subtract(I, Matrix.Multiply(K, H)), p_e);
            qGNKF = QuaternionNormalize(qGNKF);
        }

        /// <summary>
        /// Gauss-Newton method 
        /// </summary>
        /// <param name="q"> Observed quaternion from previous iteration </param>
        /// <param name="accx"> X-axis accelerometer data - g </param>
        /// <param name="accy"> Y-axis accelerometer data - g </param>
        /// <param name="accz"> Z-axis accelerometer data - g </param>
        /// <param name="magx"> X-axis magnetometer data - gauss </param>
        /// <param name="magy"> Y-axis magnetometer data - gauss </param>
        /// <param name="magz"> Z-axis magnetometer data - gauss </param>
        /// <param name="n"> Observation quaternion </param>
        private static void GaussNewton(double[,] q, double accx, double accy, double accz, double magx, double magy, double magz, out double[,] n)
        {
            double Mx = magx, My = magy, Mz = magz, Ax = accx, Ay = accy, Az = accz;
            double a = q[1, 0], b = q[2, 0], c = q[3, 0], d = q[0, 0];
            double norm = 1 / Math.Sqrt(Ax * Ax + Ay * Ay + Az * Az);
            Ax *= norm;
            Ay *= norm;
            Az *= norm;

            norm = 1 / Math.Sqrt(Mx * Mx + My * My + Mz * Mz);
            Mx *= norm;
            My *= norm;
            Mz *= norm;
            int indice = 1;
            n = null;

            while (indice <= 3)
            {
                double[,] n_k = new double[4, 1] { { a }, { b }, { c }, { d } };//Nomenclatura deste método considera qw em último 

                double[,] qconj = QuaternionConjugate(q);
                double[,] qmag = new double[4, 1] { { 0 }, { Mx }, { My }, { Mz } };
                double[,] h;
                //Compensação Mag
                h = QuaternionMultiply(q, QuaternionMultiply(qmag, qconj)); //Tilt Comp
                double[,] bmagn = new double[4, 1] { { 0 }, { Math.Sqrt(h[1, 0] * h[1, 0] + h[2, 0] * h[2, 0]) }, { 0 }, { h[3, 0] } };
                double bmagnNorm = Math.Sqrt(bmagn[0, 0] * bmagn[0, 0] + bmagn[1, 0] * bmagn[1, 0] + bmagn[2, 0] * bmagn[2, 0] + bmagn[3, 0] * bmagn[3, 0]);
                bmagn = Matrix.ScalarDivide(bmagnNorm, bmagn);

                double J11 = (2 * a * Ax + 2 * b * Ay + 2 * c * Az);
                double J12 = (-2 * b * Ax + 2 * a * Ay + 2 * d * Az);
                double J13 = (-2 * c * Ax - 2 * d * Ay + 2 * a * Az);
                double J14 = (2 * d * Ax - 2 * c * Ay + 2 * b * Az);

                double J21 = (2 * b * Ax - 2 * a * Ay - 2 * d * Az);
                double J22 = (2 * a * Ax + 2 * b * Ay + 2 * c * Az);
                double J23 = (2 * d * Ax - 2 * c * Ay + 2 * b * Az);
                double J24 = (2 * c * Ax + 2 * d * Ay - 2 * a * Az);

                double J31 = (2 * c * Ax + 2 * d * Ay - 2 * a * Az);
                double J32 = (-2 * d * Ax + 2 * c * Ay - 2 * b * Az);
                double J33 = (2 * a * Ax + 2 * b * Ay + 2 * c * Az);
                double J34 = (-2 * b * Ax + 2 * a * Ay + 2 * d * Az);

                double J41 = (2 * a * Mx + 2 * b * My + 2 * c * Mz);
                double J42 = (-2 * b * Mx + 2 * a * My + 2 * Mz * d);
                double J43 = (-2 * c * Mx - 2 * d * My + 2 * a * Mz);
                double J44 = (2 * d * Mx - 2 * c * My + 2 * b * Mz);

                double J51 = (2 * b * Mx - 2 * a * My - 2 * d * Mz);
                double J52 = (2 * a * Mx + 2 * b * My + 2 * c * Mz);
                double J53 = (2 * d * Mx - 2 * c * My + 2 * b * Mz);
                double J54 = (2 * c * Mx + 2 * d * My - 2 * a * Mz);

                double J61 = (2 * c * Mx + 2 * d * My - 2 * a * Mz);
                double J62 = (-2 * d * Mx + 2 * c * My - 2 * b * Mz);
                double J63 = (2 * a * Mx + 2 * b * My + 2 * c * Mz);
                double J64 = (-2 * b * Mx + 2 * a * My + 2 * d * Mz);

                double[,] J = new double[6, 4] { { J11, J12, J13, J14 }, { J21, J22, J23, J24 }, { J31, J32, J33, J34 }, { J41, J42, J43, J44 }, { J51, J52, J53, J54 }, { J61, J62, J63, J64 } };
                J = Matrix.ScalarMultiply(-1, J);

                double R11 = d * d + a * a - b * b - c * c;
                double R12 = 2 * (a * b - c * d);
                double R13 = 2 * (a * c + b * d);
                double R21 = 2 * (a * b + c * d);
                double R22 = d * d + b * b - a * a - c * c;
                double R23 = 2 * (b * c - a * d);
                double R31 = 2 * (a * c - b * d);
                double R32 = 2 * (b * c + a * d);
                double R33 = d * d + c * c - b * b - a * a;

                double[,] R = new double[3, 3] { { R11, R12, R13 }, { R21, R22, R23 }, { R31, R32, R33 } };

                double[,] M = new double[6, 6] { { R11, R12, R13, 0, 0, 0 }, { R21, R22, R23, 0, 0, 0 }, { R31, R32, R33, 0, 0, 0 }, { 0, 0, 0, R11, R12, R13 }, { 0, 0, 0, R21, R22, R23 }, { 0, 0, 0, R31, R32, R33 } };

                double[,] y_e = new double[6, 1] { { 0 }, { 0 }, { 1 }, { bmagn[1, 0] }, { bmagn[0, 0] }, { bmagn[3, 0] } };
                double[,] y_b = new double[6, 1] { { accx }, { accy }, { accz }, { magx }, { magy }, { magz } };

                //Gauss Newton Step
                double[,] inv = Matrix.Inverse(Matrix.Multiply(Matrix.Transpose(J), J));
                double[,] sub = Matrix.Subtract(y_e, Matrix.Multiply(M, y_b));
                n = Matrix.Subtract(n_k, Matrix.Multiply(Matrix.Multiply(inv, Matrix.Transpose(J)), sub));

                n = QuaternionNormalize(n);
                n_k = n;
                a = n[0, 0];
                b = n[1, 0];
                c = n[2, 0];
                d = n[3, 0];
                q = new double[4, 1] { { d }, { a }, { b }, { c } };
                indice += 1;
            }
            n = q;
        }
        #endregion

        #region Madgwick
        #region Varáveis necessárias ao filtro de Madgwick
        private static double gcx = 0, gcy = 0, gcz = 0;
        #endregion

        /// <summary>
        /// Seb Madgwick Sensor Fusion algorithm based on the Gradient Descent method
        /// </summary>
        /// <param name="q"> Quaternion from previous iteration </param>
        /// <param name="beta"> Beta gain of the filter </param>
        /// <param name="zeta"> Zeta gain of the filter </param>
        /// <param name="deltat"> Time elapsed between iterations </param>
        /// <param name="ax"> X-axis accelerometer data - g </param>
        /// <param name="ay"> Y-axis accelerometer data - g </param>
        /// <param name="az"> Z-axis accelerometer data - g </param>
        /// <param name="gx"> X-axis gyroscope data - degrees/s </param>
        /// <param name="gy"> Y-axis gyroscope data - degrees/s </param>
        /// <param name="gz"> Z-axis gyroscope data - degrees/s </param>
        /// <param name="mx"> X-axis magnetometer data - uT </param>
        /// <param name="my"> Y-axis magnetometer data - uT </param>
        /// <param name="mz"> Z-axis magnetometer data - uT </param>
        /// <param name="qout"> Out-param - Estimated Quaternion </param>
        public static void Madgwick(double[,] q, double beta, double zeta, double deltat, double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz, out double[,] qout)
        {

            gx = ConvertToRadians(gx);
            gy = ConvertToRadians(gy);
            gz = ConvertToRadians(gz);


            double q1 = q[0, 0], q2 = q[1, 0], q3 = q[2, 0], q4 = q[3, 0];
            double norm;
            double hx, hy, _2bx, _2bz;
            double s1, s2, s3, s4;
            double qDot1, qDot2, qDot3, qDot4;
            double gerrx, gerry, gerrz;

            double _2q1mx;
            double _2q1my;
            double _2q1mz;
            double _2q2mx;
            double _4bx;
            double _4bz;
            double _2q1 = 2.0f * q1;
            double _2q2 = 2.0f * q2;
            double _2q3 = 2.0f * q3;
            double _2q4 = 2.0f * q4;
            double _2q1q3 = 2.0f * q1 * q3;
            double _2q3q4 = 2.0f * q3 * q4;
            double q1q1 = q1 * q1;
            double q1q2 = q1 * q2;
            double q1q3 = q1 * q3;
            double q1q4 = q1 * q4;
            double q2q2 = q2 * q2;
            double q2q3 = q2 * q3;
            double q2q4 = q2 * q4;
            double q3q3 = q3 * q3;
            double q3q4 = q3 * q4;
            double q4q4 = q4 * q4;

            norm = Math.Sqrt(ax * ax + ay * ay + az * az);
            norm = 1.0f / norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            norm = Math.Sqrt(mx * mx + my * my + mz * mz);
            norm = 1.0f / norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = 2.0f * (Math.Sqrt(hx * hx + hy * hy));
            _2bz = 2.0f * (-_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4);
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = Math.Sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
            norm = 1.0f / norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            gerrx = _2q1 * s2 - _2q2 * s1 - _2q3 * s4 + _2q4 * s3;
            gerry = _2q1 * s3 + _2q2 * s4 - _2q3 * s1 - _2q4 * s2;
            gerrz = _2q1 * s4 - _2q2 * s3 + _2q3 * s2 - _2q4 * s1;

            gcx += gerrx * deltat * zeta;
            gcy += gerry * deltat * zeta;
            gcz += gerrz * deltat * zeta;

            gx -= gcx;
            gy -= gcy;
            gz -= gcz;


            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = Math.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            qout = new double[4, 1];
            qout[0, 0] = q1 * norm;
            qout[1, 0] = q2 * norm;
            qout[2, 0] = q3 * norm;
            qout[3, 0] = q4 * norm;

        }
        #endregion

        #region Mahony
        #region Variáveis necessárias ao filtro de Mahony
        private static double[] eIntIMU = new double[3] { 0.0f, 0.0f, 0.0f };
        private static double[] eIntAHRS = new double[3] { 0.0f, 0.0f, 0.0f };
        #endregion

        /// <summary>
        /// Robert Mahony Explicit Complementary filter for AHRS
        /// </summary>
        /// <param name="q"> Quaternion from previous iteration</param>
        /// <param name="Kp"> Proportional gain of the filter </param>
        /// <param name="Ki"> Integral gain of the filter </param>
        /// <param name="deltat"> Time elapsed between iterations </param>
        /// <param name="ax"> X-axis accelerometer data - g </param>
        /// <param name="ay"> Y-axis accelerometer data - g </param>
        /// <param name="az"> Z-axis accelerometer data - g </param>
        /// <param name="gx"> X-axis gyroscope data - degrees/s </param>
        /// <param name="gy"> Y-axis gyroscope data - degrees/s </param>
        /// <param name="gz"> Z-axis gyroscope data - degrees/s </param>
        /// <param name="mx"> X-axis magnetometer data - uT </param>
        /// <param name="my"> Y-axis magnetometer data - uT </param>
        /// <param name="mz"> Z-axis magnetometer data - uT </param>
        /// <param name="qout"> Out param - Estimated quaternion </param>
        public static void MahonyQuaternionAHRS(double[,] q, double Kp, double Ki, double deltat, double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz, out double[,] qout)
        {
            gx = ConvertToRadians(gx);
            gy = ConvertToRadians(gy);
            gz = ConvertToRadians(gz);
            double q1 = q[0, 0], q2 = q[1, 0], q3 = q[2, 0], q4 = q[3, 0];
            double norm;
            double hx, hy, bx, bz;
            double vx, vy, vz, wx, wy, wz;
            double ex, ey, ez;
            double pa, pb, pc;


            double q1q1 = q1 * q1;
            double q1q2 = q1 * q2;
            double q1q3 = q1 * q3;
            double q1q4 = q1 * q4;
            double q2q2 = q2 * q2;
            double q2q3 = q2 * q3;
            double q2q4 = q2 * q4;
            double q3q3 = q3 * q3;
            double q3q4 = q3 * q4;
            double q4q4 = q4 * q4;

            norm = Math.Sqrt(ax * ax + ay * ay + az * az);
            norm = 1.0f / norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            norm = Math.Sqrt(mx * mx + my * my + mz * mz);
            norm = 1.0f / norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
            bx = Math.Sqrt((hx * hx) + (hy * hy));
            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (Ki > 0.0f)
            {
                eIntAHRS[0] += ex;
                eIntAHRS[1] += ey;
                eIntAHRS[2] += ez;
            }
            else
            {
                eIntAHRS[0] = 0.0f;
                eIntAHRS[1] = 0.0f;
                eIntAHRS[2] = 0.0f;
            }

            gx = gx + Kp * ex + Ki * eIntAHRS[0];
            gy = gy + Kp * ey + Ki * eIntAHRS[1];
            gz = gz + Kp * ez + Ki * eIntAHRS[2];

            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

            norm = Math.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            qout = new double[4, 1];
            qout[0, 0] = q1 * norm;
            qout[1, 0] = q2 * norm;
            qout[2, 0] = q3 * norm;
            qout[3, 0] = q4 * norm;

        }

        /// <summary>
        /// Robert Mahony Explicit Complementary filter for IMUs
        /// </summary>
        /// <param name="q"> Quaternion from previous iterations </param>
        /// <param name="Kp"> Proportional gain of the filter </param>
        /// <param name="Ki"> Integral gain of the filter </param>
        /// <param name="deltat"> Time elapsed between iterations </param>
        /// <param name="ax"> X-axis accelerometer data - g </param>
        /// <param name="ay"> Y-axis accelerometer data - g </param>
        /// <param name="az"> Z-axis accelerometer data - g </param>
        /// <param name="gx"> X-axis gyroscope data - degrees/s </param>
        /// <param name="gy"> Y-axis gyroscope data - degrees/s </param>
        /// <param name="gz"> Z-axis gyroscope data - degrees/s </param>
        /// <param name="qout"> Out param -  Estimated quaternion </param>
        public static void MahonyQuaternionUpdateIMU(double[,] q, double Kp, double Ki, double deltat, double ax, double ay, double az, double gx, double gy, double gz, out double[,] qout)
        {
            gx = ConvertToRadians(gx);
            gy = ConvertToRadians(gy);
            gz = ConvertToRadians(gz);
            double q1 = q[0, 0], q2 = q[1, 0], q3 = q[2, 0], q4 = q[3, 0];
            double norm;
            double vx, vy, vz;
            double ex, ey, ez;
            double pa, pb, pc;


            double q1q1 = q1 * q1;
            double q1q2 = q1 * q2;
            double q1q3 = q1 * q3;
            double q1q4 = q1 * q4;
            double q2q2 = q2 * q2;
            double q2q3 = q2 * q3;
            double q2q4 = q2 * q4;
            double q3q3 = q3 * q3;
            double q3q4 = q3 * q4;
            double q4q4 = q4 * q4;

            norm = Math.Sqrt(ax * ax + ay * ay + az * az);
            norm = 1.0f / norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;


            ex = (ay * vz - az * vy);
            ey = (az * vx - ax * vz);
            ez = (ax * vy - ay * vx);
            if (Ki > 0.0f)
            {
                eIntIMU[0] += ex;
                eIntIMU[1] += ey;
                eIntIMU[2] += ez;
            }
            else
            {
                eIntIMU[0] = 0.0f;
                eIntIMU[1] = 0.0f;
                eIntIMU[2] = 0.0f;
            }

            gx = gx + Kp * ex + Ki * eIntIMU[0];
            gy = gy + Kp * ey + Ki * eIntIMU[1];
            gz = gz + Kp * ez + Ki * eIntIMU[2];

            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

            norm = Math.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            qout = new double[4, 1];
            qout[0, 0] = q1 * norm;
            qout[1, 0] = q2 * norm;
            qout[2, 0] = q3 * norm;
            qout[3, 0] = q4 * norm;

        }
        #endregion

        #region Gaussian Evaluation
        /// <summary>
        /// Determine if the sensors are moving or if the fluctuations are due to noise.
        /// </summary>
        /// <param name="datax"> X-axis of a sensor </param>
        /// <param name="datay"> Y-axis of a sensor </param>
        /// <param name="dataz"> Z-axis of a sensor </param>
        /// <param name="N"> Number of samples </param>
        /// <param name="th"> Threshold to consider movement </param>
        /// <param name="motionx"> Out param - Integer that indicates movement or not on the X-axis </param>
        /// <param name="motiony"> Out param - Integer that indicates movement or not on the Y-axis </param>
        /// <param name="motionz"> Out param - Integer that indicates movement or not on the Z-axis </param>
        public static void GaussianEval(double[] datax, double[] datay, double[] dataz, int N, double th, out int motionx, out int motiony, out int motionz)
        {
            double T1x, T2x, T3x, V3x, T1y, T2y, T3y, V3y, T1z, T2z, T3z, V3z, T4x, T4y, T4z, V4x, V4y, V4z;
            double Xn = 0, Xn2 = 0, Xn3 = 0, Xn4 = 0, Yn = 0, Yn2 = 0, Yn3 = 0, Yn4 = 0, Zn = 0, Zn2 = 0, Zn3 = 0, Zn4 = 0;

            for (int i = 0; i <= N - 1; i++)
            {
                Xn += datax[i];
                Xn2 += Math.Pow(datax[i], 2);
                Xn3 += Math.Pow(datax[i], 3);
                Xn4 += Math.Pow(datax[i], 4);

                Yn += datay[i];
                Yn2 += Math.Pow(datay[i], 2);
                Yn3 += Math.Pow(datay[i], 3);
                Yn4 += Math.Pow(datay[i], 4);

                Zn += dataz[i];
                Zn2 += Math.Pow(dataz[i], 2);
                Zn3 += Math.Pow(dataz[i], 3);
                Zn4 += Math.Pow(dataz[i], 4);
            }
            T1x = Xn / N;
            T2x = Xn2 / N;
            T3x = Xn3 / N;
            T4x = Xn4 / N;
            V3x = T3x - 3 * T2x * T1x + Math.Pow(2 * T1x, 3);
            V4x = T4x - 3 * T2x * T2x + 2 * T1x - 4 * V3x * T1x;

            T1y = Yn / N;
            T2y = Yn2 / N;
            T3y = Yn3 / N;
            T4y = Yn4 / N;
            V3y = T3y - 3 * T2y * T1y + Math.Pow(2 * T1y, 3);
            V4y = T4y - 3 * T2y * T2y + 2 * T1y - 4 * V3y * T1y;

            T1z = Zn / N;
            T2z = Zn2 / N;
            T3z = Zn3 / N;
            T4z = Zn4 / N;
            V3z = T3z - 3 * T2z * T1z + Math.Pow(2 * T1z, 3);
            V4z = T4z - 3 * T2z * T2z + 2 * T1z - 4 * V3z * T1z;

            if (V4x <= th && V4x >= -th)
            {
                motionx = 0; //no motion
            }
            else
            {
                motionx = 1; //motion
            }
            if (V4y <= th && V4y >= -th)
            {
                motiony = 0; //no motion
            }
            else
            {
                motiony = 1; //motion
            }
            if (V4z <= th && V4z >= -th)
            {
                motionz = 0; //no motion
            }
            else
            {
                motionz = 1; //motion
            }
        }
        #endregion
    }
    

