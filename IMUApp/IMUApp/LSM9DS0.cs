using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using OpenTK.Graphics.OpenGL;
using System.Drawing.Imaging;
using OpenTK;
using OpenTK.Graphics;
using System.IO;
using System.Windows.Forms.DataVisualization.Charting;
using System.Diagnostics;
using MatrixLibrary;

namespace IMUApp
{
    public partial class LSM9DS0 : Form
    {

        #region Declaração Variáveis Globais
        Stopwatch stopwatch2 = new Stopwatch();
        SensorFusion SensorFusionAlgorithms = new SensorFusion();
        SerialPort LSM = new SerialPort();
        double accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz;
        double RealAccx, RealAccy, RealAccz;
        

        SerialPort Motoman3 = new SerialPort();
        Stopwatch stopwatch = new Stopwatch();
        


        double[] datax = new double[25];
        double[] datay = new double[25];
        double[] dataz = new double[25];
        double[] datax2 = new double[25];
        double[] datay2 = new double[25];
        double[] dataz2 = new double[25];

        double dt = 0.05;

        string data;
        int handle;
        string path;
        bool Mahony, Madgwick, GNCF, GNKF, EKFDS, EKFIMU, MahonyIMU;
        System.IO.FileStream fs;
        #endregion

        #region State Machine
        bool getAcc = false, getGyro = false, getMag = false;
        #endregion

        #region Variáveis Modelo 3D
        bool gl3dloaded = false;
        int width, height;
        double roll = 0, yaw = 0, pitch = 0;
        #endregion

        #region Variáveis Algoritmos
        double[,] Euler;
        double[,] qFinal = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };
        double[,] qMadgwick = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };

        double[,] qMahony = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };
        double[,] qMahonyIMU = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };

        double[,] qEKF = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };
        double[,] pEKF = new double[4, 4] {{1.0f,0.0f,0.0f,0.0f},
                                           {0.0f,1.0f,0.0f,0.0f},
                                           {0.0f,0.0f,1.0f,0.0f},
                                           {0.0f,0.0f,0.0f,1.0f}};
        double[,] qEKFIMU = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };
        double[,] pEKFIMU = new double[4, 4] {{1.0f,0.0f,0.0f,0.0f},
                                           {0.0f,1.0f,0.0f,0.0f},
                                           {0.0f,0.0f,1.0f,0.0f},
                                           {0.0f,0.0f,0.0f,1.0f}};
        double[,] qComplementarGN = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };
        double[,] qOsservGN = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };
        double[,] qGNKF = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };
        double[,] pGNKF = new double[4, 4] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };

        double[,] EulerMadgwick, EulerMahony, EulerGNCF, EulerGNKF, EulerEKF;
        double rollMadgwick, pitchMadgwick, yawMadgwick, rollMahony, pitchMahony, yawMahony, rollGNCF, pitchGNCF, yawGNCF, rollGNKF, pitchGNKF, yawGNKF, rollEKF, pitchEKF, yawEKF;

        double[,] Q;
        double[,] R1;
        double[,] R2;
        double Kp;
        double Ki;
        double beta;
        double zeta;
        double gain;

        double[,] R;
        double[,] Soft_Iron = new double[3, 3] {{0.8984,0.0095,-0.0116},
                                                {0.0095, 0.9513,0.0017},
                                                {-0.0116 ,0.0017,0.9987}};

        double[,] mag;

        #endregion

        public LSM9DS0()
        {
            InitializeComponent();
            algorithm.SelectedIndex = 0;
            ScanPorts();
            start_log.Enabled = false;

        

        }

        private void ScanPorts()
        {
            string[] ports = SerialPort.GetPortNames();
            foreach (string s in ports)
            {
                portsAvailable.Items.Add(s);
            }
            if (portsAvailable.Items.Count != 0)
            {
                portsAvailable.SelectedIndex = 0;
            }
        }

        private void scan_Click(object sender, EventArgs e)
        {
            portsAvailable.Items.Clear();
            ScanPorts();
        }

        private void closeapp_Click(object sender, EventArgs e)
        {
            System.Windows.Forms.Application.Exit();
        }

        private void open_Click(object sender, EventArgs e)
        {
            LSM = new SerialPort();
            LSM.PortName = portsAvailable.SelectedItem.ToString();
            LSM.BaudRate = 115200;
            LSM.Parity = Parity.None;
            LSM.StopBits = StopBits.One;
            LSM.Handshake = Handshake.None;
            LSM.DataBits = 8;
            LSM.DataReceived += new SerialDataReceivedEventHandler(data_received);

            if (!LSM.IsOpen)
            {
                LSM.Open();
                LSM.DtrEnable = false;
                LSM.DiscardInBuffer();
                LSM.DtrEnable = true;
            }
            else
            {
                LSM.DtrEnable = false;
                LSM.Close();
                LSM.Open();
                LSM.DtrEnable = true;
            }
        }

        private void close_Click(object sender, EventArgs e)
        {
            if (LSM != null)
            {
                LSM.RtsEnable = false;
                LSM.DtrEnable = false;

                LSM.DiscardInBuffer();
                LSM.Close();
                open.Enabled = true;
            }
        }

        private void ReadData()
        {
            if (LSM.IsOpen)
            {
                data = LSM.ReadTo("\r\n");
                if (data.Contains("."))
                {
                    data = data.Replace(".", ",");
                }
            }
        }

        private void data_received(object sender, SerialDataReceivedEventArgs e)
        {
            handle = LSM.ReadByte();
            switch (handle)
            {
                case 0x48:
                    ReadData();
                    accx = double.Parse(data);
                    ReadData();
                    accy = -double.Parse(data);
                    ReadData();
                    accz = double.Parse(data);

                    accx -= -0.1202f;
                    accy -= 0.01f;
                    accz -= -0.0532f;

                    getAcc = true;
                    break;
                case 0x49:
                    ReadData();
                    gyrx = -float.Parse(data);
                    ReadData();
                    gyry = float.Parse(data); 
                    ReadData();
                    gyrz = -float.Parse(data);

                    gyrx -= -0.1438f;
                    gyry -= 2.2487f;
                    gyrz -= 3.9429f;

                    getGyro = true;

                    break;
                case 0x50:

                    ReadData();
                    magx = -float.Parse(data);
                    ReadData();
                    magy = float.Parse(data); 
                    ReadData();
                    magz = float.Parse(data);

                    magx -= 0.0403f;
                    magy -= 0.0216f;
                    magz -= -0.0345f;

                    //Soft-Iron
                    mag = new double[3, 1] { { magx }, { magy }, { magz } };
                    mag = Matrix.Multiply(Soft_Iron, mag);
                    magx = mag[0, 0];
                    magy = mag[1, 0];
                    magz = mag[2, 0];

                    getMag = true;
                    break;
     
                default:
                    break;
            }
            if (getAcc == true && getGyro == true && getMag == true)
            {
                this.BeginInvoke(new EventHandler(DataProcessing));
                getAcc = false;
                getGyro = false;
                getMag = false;

            }
        }

        private void DataProcessing(object sender, EventArgs e)
        {
            #region Calculate dt
            if (stopwatch2.IsRunning)
            {
                stopwatch2.Stop();
                dt = stopwatch2.Elapsed.TotalSeconds;
                //tempo.Text = dt.ToString();

            }
            stopwatch2.Restart();
            #endregion
            
            #region EKF imu
            Q = new double[4, 4]       {{0.007,0    ,0    ,0    },
                                            {0    ,0.007,0    ,0    },
                                            {0    ,0    ,0.007,0    },
                                            {0    ,0    ,0    ,0.007}};
            R1 = new double[3, 3]       {{1.0f,0    ,0    },
                                             {0    ,1.0f,0    },
                                             {0    ,0    ,1.0f}};
            SensorFusion.EKFimu(qEKFIMU, pEKFIMU, Q, R1, dt, accx, accy, accz, gyrx, gyry, gyrz, out qEKFIMU, out pEKFIMU);
            
            if (EKFIMU)
            {
    
                qFinal = qEKFIMU;
            }
            #endregion

            #region SebMadgwick
            beta = Math.Sqrt(3.0f / 4.0f) * (Math.PI * (20.0f / 180.0f));
            zeta = Math.Sqrt(3.0f / 4.0f) * (Math.PI * (00.0f / 180.0f));
            SensorFusion.Madgwick(qMadgwick, beta, zeta, dt, accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz, out qMadgwick);
            EulerMadgwick = SensorFusion.QuaternionToEuler(qMadgwick);

            rollMadgwick = SensorFusion.ConvertToDegrees(EulerMadgwick[0, 0]);

            pitchMadgwick = SensorFusion.ConvertToDegrees(EulerMadgwick[1, 0]);

            yawMadgwick = SensorFusion.ConvertToDegrees(EulerMadgwick[2, 0]);
            if (Madgwick)
            {
                
                qFinal = qMadgwick;
            }
            #endregion

            #region Mahony
            Kp = 2.0f * 1.0f;
            Ki = 0.0f;
            SensorFusion.MahonyQuaternionAHRS(qMahony, Kp, Ki, dt, accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz, out qMahony);
            EulerMahony = SensorFusion.QuaternionToEuler(qMahony);

            rollMahony = SensorFusion.ConvertToDegrees(EulerMahony[0, 0]);

            pitchMahony = SensorFusion.ConvertToDegrees(EulerMahony[1, 0]);

            yawMahony = SensorFusion.ConvertToDegrees(EulerMahony[2, 0]);
            if (Mahony)
            {
                
                qFinal = qMahony;
            }
            Kp = 2.0f * 1.0f;
            Ki = 0.0f;
            SensorFusion.MahonyQuaternionUpdateIMU(qMahonyIMU, Kp, Ki, dt, accx, accy, accz, gyrx, gyry, gyrz, out qMahonyIMU);
            if (MahonyIMU)
            {
                
                qFinal = qMahonyIMU;
            }
            #endregion

            #region DoubleStageEKF
            Q = new double[4, 4]       {{0.007,0    ,0    ,0    },
                                            {0    ,0.007,0    ,0    },
                                            {0    ,0    ,0.007,0    },
                                            {0    ,0    ,0    ,0.007}};
            R1 = new double[3, 3]       {{1.0f,0    ,0    },
                                             {0    ,1.0f,0    },
                                             {0    ,0    ,1.0f}};
            R2 = new double[3, 3]       {{2.0f,0   ,0   },
                                             {0   ,2.0f,0   },
                                             {0   ,0   ,2.0f}};
            SensorFusion.DoubleStageEKF(pEKF, qEKF, Q, R1, R2, dt, gyrx, gyry, gyrz, accx, accy, accz, magx, magy, magz, out qEKF, out pEKF);
            EulerEKF = SensorFusion.QuaternionToEuler(qEKF);

            rollEKF = SensorFusion.ConvertToDegrees(EulerEKF[0, 0]);

            pitchEKF = SensorFusion.ConvertToDegrees(EulerEKF[1, 0]);

            yawEKF = SensorFusion.ConvertToDegrees(EulerEKF[2, 0]);
            if (EKFDS)
            {
                
                qFinal = qEKF;
            }

            #endregion

            #region Gauss-Newton
            gain = 0.75;

            SensorFusion.GaussNewtonComplementar(qComplementarGN, qOsservGN, gain, dt, accx, accy, accz, magx, magy, magz, gyrx, gyry, gyrz, out qComplementarGN, out qOsservGN);
            EulerGNCF = SensorFusion.QuaternionToEuler(qComplementarGN);

            rollGNCF = SensorFusion.ConvertToDegrees(EulerGNCF[0, 0]);

            pitchGNCF = SensorFusion.ConvertToDegrees(EulerGNCF[1, 0]);

            yawGNCF = SensorFusion.ConvertToDegrees(EulerGNCF[2, 0]);
            if (GNCF)
            {
                
                qFinal = qComplementarGN;
            }
            Q = new double[4, 4]       {{0.007,0    ,0    ,0    },
                                            {0    ,0.007,0    ,0    },
                                            {0    ,0    ,0.007,0    },
                                            {0    ,0    ,0    ,0.007}};
            R = new double[4, 4]        {{0.5f ,0    ,0     ,0},
                                             {0    ,0.5f ,0    ,0},
                                             {0    ,0    ,0.5f ,0},
                                             {0    ,0    ,0    ,0.5f}};
            SensorFusion.GaussNewtonKalmanFilter(qGNKF, pGNKF, Q, R, dt, accx, accy, accz, magx, magy, magz, gyrx, gyry, gyrz, out qGNKF, out pGNKF);
            EulerGNKF = SensorFusion.QuaternionToEuler(qGNKF);

            rollGNKF = SensorFusion.ConvertToDegrees(EulerGNKF[0, 0]);

            pitchGNKF = SensorFusion.ConvertToDegrees(EulerGNKF[1, 0]);

            yawGNKF = SensorFusion.ConvertToDegrees(EulerGNKF[2, 0]);
            if (GNKF)
            {
                
                qFinal = qGNKF;
            }
            #endregion

            #region Get Euler
            //datax[count] = gyrx;
            //datay[count] = gyry;
            //dataz[count] = gyrz;
            //count++;
            //if (count == N)
            //{
            //    SensorFusion.GaussianEval(datax, datay, dataz, N, th, out motionx, out motiony, out motionz);
            //    count = 0;
            //}

            //if (motionx == 1 || motiony == 1 || motionz == 1)
            //{
                Euler = SensorFusion.QuaternionToEuler(qFinal);
                roll = SensorFusion.ConvertToDegrees(Euler[0, 0]);
                pitch = SensorFusion.ConvertToDegrees(Euler[1, 0]);
                yaw = SensorFusion.ConvertToDegrees(Euler[2, 0]);

            //}
           
            #endregion

            #region Remove Gravity
            if (WithoutGravity.Checked == true)
            {
                SensorFusion.RemoveGravityQuatEarthFrame(accx, accy, accz, qFinal, out  RealAccx, out  RealAccy, out RealAccz);
            }
            #endregion

            #region Display Data on Graphs
            if (Acc.Series["Acc X"].Points.Count == 100)
            {
                Acc.Series["Acc X"].Points.RemoveAt(0);
                Acc.Series["Acc Y"].Points.RemoveAt(0);
                Acc.Series["Acc Z"].Points.RemoveAt(0);
            }
            if (WithoutGravity.Checked == true)
            {
                Acc.Series["Acc X"].Points.AddY(RealAccx);
                Acc.Series["Acc Y"].Points.AddY(RealAccy);
                Acc.Series["Acc Z"].Points.AddY(RealAccz);
                Acc.ChartAreas["ChartArea1"].RecalculateAxesScale();
            }
            else
            {
                Acc.Series["Acc X"].Points.AddY(accx);
                Acc.Series["Acc Y"].Points.AddY(accy);
                Acc.Series["Acc Z"].Points.AddY(accz);
                Acc.ChartAreas["ChartArea1"].RecalculateAxesScale();

            }


            if (Gyr.Series["Gyr X"].Points.Count == 100)
            {
                Gyr.Series["Gyr X"].Points.RemoveAt(0);
                Gyr.Series["Gyr Y"].Points.RemoveAt(0);
                Gyr.Series["Gyr Z"].Points.RemoveAt(0);
            }
            Gyr.Series["Gyr X"].Points.AddY(gyrx);
            Gyr.Series["Gyr Y"].Points.AddY(gyry);
            Gyr.Series["Gyr Z"].Points.AddY(gyrz);
            Gyr.ChartAreas["ChartArea1"].RecalculateAxesScale();


            if (Mag.Series["Mag X"].Points.Count == 100)
            {
                Mag.Series["Mag X"].Points.RemoveAt(0);
                Mag.Series["Mag Y"].Points.RemoveAt(0);
                Mag.Series["Mag Z"].Points.RemoveAt(0);
            }
            
                Mag.Series["Mag X"].Points.AddY(magx);
                Mag.Series["Mag Y"].Points.AddY(magy);
                Mag.Series["Mag Z"].Points.AddY(magz);
                Mag.ChartAreas["ChartArea1"].RecalculateAxesScale();
            
            if (EulerGraph.Series["Roll"].Points.Count == 100)
            {
                EulerGraph.Series["Roll"].Points.RemoveAt(0);
                EulerGraph.Series["Pitch"].Points.RemoveAt(0);
                EulerGraph.Series["Yaw"].Points.RemoveAt(0);
            }
            EulerGraph.Series["Roll"].Points.AddY(roll);
            EulerGraph.Series["Pitch"].Points.AddY(pitch);
            EulerGraph.Series["Yaw"].Points.AddY(yaw);
            EulerGraph.ChartAreas["ChartArea1"].RecalculateAxesScale();

            #endregion

            #region Write to File
            if (start_log.Checked == true)
            {
                string gnuplotDataString = "";
                gnuplotDataString += accx.ToString().Replace(",", ".") + " ";
                gnuplotDataString += accy.ToString().Replace(",", ".") + " ";
                gnuplotDataString += accz.ToString().Replace(",", ".") + " ";
                gnuplotDataString += gyrx.ToString().Replace(",", ".") + " ";
                gnuplotDataString += gyry.ToString().Replace(",", ".") + " ";
                gnuplotDataString += gyrz.ToString().Replace(",", ".") + " ";
                gnuplotDataString += magx.ToString().Replace(",", ".") + " ";
                gnuplotDataString += magy.ToString().Replace(",", ".") + " ";
                gnuplotDataString += magz.ToString().Replace(",", ".") + " ";
                gnuplotDataString += qMadgwick[0, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qMadgwick[1, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qMadgwick[2, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qMadgwick[3, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += rollMadgwick.ToString().Replace(",", ".") + " ";
                gnuplotDataString += pitchMadgwick.ToString().Replace(",", ".") + " ";
                gnuplotDataString += yawMadgwick.ToString().Replace(",", ".") + " ";
                gnuplotDataString += qMahony[0, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qMahony[1, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qMahony[2, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qMahony[3, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += rollMahony.ToString().Replace(",", ".") + " ";
                gnuplotDataString += pitchMahony.ToString().Replace(",", ".") + " ";
                gnuplotDataString += yawMahony.ToString().Replace(",", ".") + " ";
                gnuplotDataString += qEKF[0, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qEKF[1, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qEKF[2, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qEKF[3, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += rollEKF.ToString().Replace(",", ".") + " ";
                gnuplotDataString += pitchEKF.ToString().Replace(",", ".") + " ";
                gnuplotDataString += yawEKF.ToString().Replace(",", ".") + " ";
                gnuplotDataString += qComplementarGN[0, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qComplementarGN[1, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qComplementarGN[2, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qComplementarGN[3, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += rollGNCF.ToString().Replace(",", ".") + " ";
                gnuplotDataString += pitchGNCF.ToString().Replace(",", ".") + " ";
                gnuplotDataString += yawGNCF.ToString().Replace(",", ".") + " ";
                gnuplotDataString += qGNKF[0, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qGNKF[1, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qGNKF[2, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += qGNKF[3, 0].ToString().Replace(",", ".") + " ";
                gnuplotDataString += rollGNKF.ToString().Replace(",", ".") + " ";
                gnuplotDataString += pitchGNKF.ToString().Replace(",", ".") + " ";
                gnuplotDataString += yawGNKF.ToString().Replace(",", ".") + " ";
                gnuplotDataString += "1" + " ";
                gnuplotDataString += "1" + " ";
                gnuplotDataString += "1" + " ";
                gnuplotDataString += "1" + " ";
                gnuplotDataString += "1" + " ";
                gnuplotDataString += "1" + " ";
                gnuplotDataString += "1" + " ";
                gnuplotDataString += RealAccx.ToString().Replace(",", ".") + " ";
                gnuplotDataString += RealAccy.ToString().Replace(",", ".") + " ";
                gnuplotDataString += RealAccz.ToString().Replace(",", ".") + " ";
                gnuplotDataString += "1" + " ";
                gnuplotDataString += "1" + " ";
                gnuplotDataString += "1" + " ";
                gnuplotDataString += dt.ToString().Replace(",", ".") + " ";
                WriteGnuplotFile(gnuplotDataString, path);
            }

            #endregion

            #region Display Quaternions TextBox

            FusedQuatWText.Text = qFinal[0, 0].ToString();
            FusedQuatXText.Text = qFinal[1, 0].ToString();
            FusedQuatYText.Text = qFinal[2, 0].ToString();
            FusedQuatZText.Text = qFinal[3, 0].ToString();

            FusedRollText.Text = roll.ToString();
            FusedPitchText.Text = pitch.ToString();
            FusedYawText.Text = yaw.ToString();
            #endregion

            #region Update 3D Cube
            glControl1.Invalidate();
            #endregion
        }

        #region 3DAnimation
        private int front_texture;
        private int back_texture;
        private void glControl1_Load(object sender, EventArgs e)
        {
            gl3dloaded = true;

            GL.ClearColor(System.Drawing.SystemColors.Control);
            GL.Enable(EnableCap.DepthTest);
            front_texture = UploadTexture("../../Content/lsm9ds0front.jpg");
            back_texture = UploadTexture("../../Content/lsm9ds0back.jpg");

            GL.Enable(EnableCap.Texture2D);
            //Basically enables the alpha channel to be used in the color buffer
            GL.Enable(EnableCap.Blend);
            //The operation/order to blend
            GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha);
            //Use for pixel depth comparing before storing in the depth buffer
            GL.Enable(EnableCap.DepthTest);
            //UploadTexture(@"Content\transferir.jpg");
            SetupViewport();


        }
        private void SetupViewport()
        {


            float aspectRatio = (float)glControl1.Width / (float)glControl1.Height;

            width = glControl1.Width;
            height = glControl1.Height;

            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();
            GL.Viewport(0, 0, width, height); // Use all of the glControl painting area

            Matrix4 perspective = Matrix4.CreatePerspectiveFieldOfView(MathHelper.PiOver4, aspectRatio, 0.5f, 100.0f);

            GL.MultMatrix(ref perspective);

            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadIdentity();
        }
        private void glControl1_Resize(object sender, EventArgs e)
        {
            SetupViewport();
            glControl1.Invalidate();

        }
        private void glControl1_Paint(object sender, PaintEventArgs e)
        {
            if (!gl3dloaded)
                return;

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            Matrix4 lookat = Matrix4.LookAt(Vector3.UnitZ * 2.5f, Vector3.Zero, Vector3.UnitY);
            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadMatrix(ref lookat);


            GL.Rotate(30.0f, Vector3.UnitY); // Inserir visão já das outras faces :)
            GL.Rotate(20.0f, Vector3.UnitX);
            GL.Rotate(8.0f, Vector3.UnitZ);

            GL.LineWidth(5.0f);





            GL.Enable(EnableCap.LineSmooth);
            GL.Begin(PrimitiveType.Lines);

            GL.Color3(Color.Red); // Z axis Line
            GL.Vertex3(0.0f, 0.0f, -2.0f);
            GL.Vertex3(0.0f, 0.0f, 2.0f);


            GL.Color3(Color.Green); // Y axis Line
            GL.Vertex3(0.0f, -2.0f, 0.0f);
            GL.Vertex3(0.0f, 2.0f, 0.0f);

            GL.Color3(Color.Blue); // X axis Line
            GL.Vertex3(-2.0f, 0.0f, 0.0f);
            GL.Vertex3(2.0f, 0.0f, 0.0f);

            GL.End();

            //Euler

            GL.Rotate(-yaw, Vector3d.UnitY);
            GL.Rotate(pitch, Vector3d.UnitX);
            GL.Rotate(-roll, Vector3d.UnitZ);



            DrawCube();



            //Draw Axis
            GL.LineWidth(1.0f);
            GL.Enable(EnableCap.LineSmooth);
            GL.Begin(PrimitiveType.Lines);

            GL.Color3(Color.Red); // Z axis Line
            GL.Vertex3(0.0f, 0.0f, -2.0f);
            GL.Vertex3(0.0f, 0.0f, 2.0f);


            GL.Color3(Color.Green); // Y axis Line
            GL.Vertex3(0.0f, -2.0f, 0.0f);
            GL.Vertex3(0.0f, 2.0f, 0.0f);

            GL.Color3(Color.Blue); // X axis Line
            GL.Vertex3(-2.0f, 0.0f, 0.0f);
            GL.Vertex3(2.0f, 0.0f, 0.0f);

            GL.End();

            glControl1.SwapBuffers();
        }
        static public int UploadTexture(string pathname)
        {
            // Create a new OpenGL texture object
            int id = GL.GenTexture();

            // Select the new texture
            GL.BindTexture(TextureTarget.Texture2D, id);

            // Configure minification and magnification filters
            GL.TexParameter(TextureTarget.Texture2D,
                    TextureParameterName.TextureMinFilter,
                    (int)TextureMinFilter.Linear);
            GL.TexParameter(TextureTarget.Texture2D,
                    TextureParameterName.TextureMagFilter,
                    (int)TextureMagFilter.Linear);

            // Load the image
            Bitmap bmp = new Bitmap(pathname);

            // Lock image data to allow direct access
            BitmapData bmp_data = bmp.LockBits(
                    new Rectangle(0, 0, bmp.Width, bmp.Height),
                    System.Drawing.Imaging.ImageLockMode.ReadOnly,
                    System.Drawing.Imaging.PixelFormat.Format32bppArgb);

            // Import the image data into the OpenGL texture
            GL.TexImage2D(TextureTarget.Texture2D,
                          0,
                          PixelInternalFormat.Rgba,
                          bmp_data.Width,
                          bmp_data.Height,
                          0,
                          OpenTK.Graphics.OpenGL.PixelFormat.Bgra,
                          OpenTK.Graphics.OpenGL.PixelType.UnsignedByte,
                          IntPtr.Zero);

            GL.TexSubImage2D(TextureTarget.Texture2D, 0, 0, 0, bmp_data.Width, bmp_data.Height, OpenTK.Graphics.OpenGL.PixelFormat.Bgra, PixelType.UnsignedByte, bmp_data.Scan0);


            // Unlock the image data
            bmp.UnlockBits(bmp_data);

            bmp.Dispose();

            GL.BindTexture(TextureTarget.Texture2D, 0);



            // Return the OpenGL object ID for use
            return id;
        }
        private int DrawCube()
        {
            GL.BindTexture(TextureTarget.Texture2D, front_texture);

            GL.Begin(PrimitiveType.QuadStrip);
            GL.PushMatrix();
            GL.Color3(Color.Transparent);
            GL.TexCoord2(0.0f, 1.0f);
            GL.Vertex3(-0.5f, 0.015f, 1.0f);
            GL.TexCoord2(0f, 0f);
            GL.Vertex3(-0.5f, 0.015f, -1.0f);
            GL.TexCoord2(1f, 1f);
            GL.Vertex3(0.5f, 0.015f, 1.0f);
            GL.TexCoord2(1f, 0f);
            GL.Vertex3(0.5f, 0.015f, -1.0f);

            GL.PopMatrix();
            GL.End();  // End of drawing color-cube
            GL.BindTexture(TextureTarget.Texture2D, 0);

            GL.BindTexture(TextureTarget.Texture2D, back_texture);

            GL.Begin(PrimitiveType.QuadStrip);
            GL.PushMatrix();
            GL.Color3(Color.Red);
            GL.Vertex3(-0.5f, 0.015f, 1.0f);
            GL.Vertex3(0.5f, 0.015f, 1.0f);
            GL.Vertex3(-0.5f, -0.015f, 1.0f);
            GL.Vertex3(0.5f, -0.015f, 1.0f);

            GL.Color3(Color.Red);
            GL.Vertex3(-0.5f, 0.015f, -1.0f);
            GL.Vertex3(0.5f, 0.015f, -1.0f);
            GL.Vertex3(-0.5f, -0.015f, -1.0f);
            GL.Vertex3(0.5f, -0.015f, -1.0f);

            GL.Color3(Color.Red);
            GL.Vertex3(0.5f, 0.015f, 1.0f);
            GL.Vertex3(0.5f, 0.015f, -1.0f);
            GL.Vertex3(0.5f, -0.015f, 1.0f);
            GL.Vertex3(0.5f, -0.015f, -1.0f);

            GL.Color3(Color.Red);
            GL.Vertex3(-0.5f, 0.015f, 1.0f);
            GL.Vertex3(-0.5f, 0.015f, -1.0f);
            GL.Vertex3(-0.5f, -0.015f, 1.0f);
            GL.Vertex3(-0.5f, -0.015f, -1.0f);

            GL.Color3(Color.Transparent);
            GL.TexCoord2(0.0f, 1.0f);
            GL.Vertex3(-0.5f, -0.015f, 1.0f);
            GL.TexCoord2(0f, 0f);
            GL.Vertex3(-0.5f, -0.015f, -1.0f);
            GL.TexCoord2(1f, 1f);
            GL.Vertex3(0.5f, -0.015f, 1.0f);
            GL.TexCoord2(1f, 0f);
            GL.Vertex3(0.5f, -0.015f, -1.0f);


            GL.PopMatrix();
            GL.End();  // End of drawing color-cube
            GL.BindTexture(TextureTarget.Texture2D, 0);

            return 6;   // Return number of faces drawn
        }
        #endregion

        public int WriteGnuplotFile(string s, string path)
        {

            //Append to file
            fs = new FileStream(path, FileMode.Append, FileAccess.Write);
            using (StreamWriter writer = new StreamWriter(fs))
            {
                writer.WriteLine(s);
            }

            return 0;
        }

        private void browse_Click(object sender, EventArgs e)
        {
            if (start_log.Enabled == false)
            {
                start_log.Enabled = true;
            }
            SaveFileDialog fdlg = new SaveFileDialog();
            fdlg.Title = "Choose File to Write";
            string CombinedPath = System.IO.Path.Combine(Directory.GetCurrentDirectory(), "../../../../SavedFiles");
            fdlg.InitialDirectory = System.IO.Path.GetFullPath(CombinedPath);
            fdlg.Filter = "Text (*.txt*)|*.txt*";
            fdlg.FilterIndex = 1;
            fdlg.DefaultExt = ".txt";
            fdlg.RestoreDirectory = true;
            if (fdlg.ShowDialog() == DialogResult.OK)
            {
                path = fdlg.FileName;
                pathText.Text = path;
                if (!System.IO.File.Exists(path))
                {
                    fs = System.IO.File.Create(path);
                    fs.Close();
                }
                else
                {
                    System.IO.File.Delete(path);
                    fs = System.IO.File.Create(path);
                    fs.Close();
                }

            }
            else
            {
                start_log.Enabled = false;
            }
        }

        private void algorithm_SelectedIndexChanged(object sender, EventArgs e)
        {
            switch (algorithm.SelectedIndex)
            {
                case 0: //Choose Algorithm
                    Mahony = false;
                    MahonyIMU = false;
                    Madgwick = false;
                    GNCF = false;
                    GNKF = false;
                    EKFDS = false;
                    EKFIMU = false;

                    roll = 0;
                    pitch = 0;
                    yaw = 0;
                    break;
                case 1: //Madgwick
                    Mahony = false;
                    MahonyIMU = false;
                    Madgwick = true;
                    GNCF = false;
                    GNKF = false;
                    EKFDS = false;
                    EKFIMU = false;
                    qMadgwick = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };
                    break;
                case 2: //Mahony
                    Mahony = true;
                    MahonyIMU = false;
                    Madgwick = false;
                    GNCF = false;
                    GNKF = false;
                    EKFDS = false;
                    EKFIMU = false;
                    qMahony = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };

                    break;
                
                case 3: //Gauss-Newton Complementary Filter
                    Mahony = false;
                    MahonyIMU = false;
                    Madgwick = false;
                    GNCF = true;
                    GNKF = false;
                    EKFDS = false;
                    EKFIMU = false;

                    qComplementarGN = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };
                    qOsservGN = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };

                    break;
                case 4: //Gauss-Newton Kalman Filter
                    Mahony = false;
                    MahonyIMU = false;
                    Madgwick = false;
                    GNCF = false;
                    GNKF = true;
                    EKFDS = false;
                    EKFIMU = false;

                    qGNKF = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };
                    pGNKF = new double[4, 4] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };

                    break;
                case 5: //Extended Kalman Filter -Double Stage
                    Mahony = false;
                    MahonyIMU = false;
                    Madgwick = false;
                    GNCF = false;
                    GNKF = false;
                    EKFDS = true;
                    EKFIMU = false;

                    qEKF = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };
                    pEKF = new double[4, 4] {{1.0f,0.0f,0.0f,0.0f},
                                           {0.0f,1.0f,0.0f,0.0f},
                                           {0.0f,0.0f,1.0f,0.0f},
                                           {0.0f,0.0f,0.0f,1.0f}};

                    break;
                case 6: //Extended Kalman Filter - IMU
                    Mahony = false;
                    MahonyIMU = false;
                    Madgwick = false;
                    GNCF = false;
                    GNKF = false;
                    EKFDS = false;
                    EKFIMU = true;

                    qEKF = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };
                    pEKF = new double[4, 4] {{1.0f,0.0f,0.0f,0.0f},
                                           {0.0f,1.0f,0.0f,0.0f},
                                           {0.0f,0.0f,1.0f,0.0f},
                                           {0.0f,0.0f,0.0f,1.0f}};

                    break;
                case 7: //Mahony - IMU
                    Mahony = false;
                    MahonyIMU = true;
                    Madgwick = false;
                    GNCF = false;
                    GNKF = false;
                    EKFDS = false;
                    EKFIMU = false;
                    qMahonyIMU = new double[4, 1] { { 1 }, { 0 }, { 0 }, { 0 } };

                    break;
                default:

                    break;
            }       
        }    

    }
}
