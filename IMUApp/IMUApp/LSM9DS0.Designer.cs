namespace IMUApp
{
    partial class LSM9DS0
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(LSM9DS0));
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea5 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend5 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series13 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series14 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series15 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea6 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend6 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series16 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series17 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series18 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea7 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend7 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series19 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series20 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series21 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea8 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend8 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series22 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series23 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series24 = new System.Windows.Forms.DataVisualization.Charting.Series();
            this.closeapp = new System.Windows.Forms.Button();
            this.algorithm = new System.Windows.Forms.ComboBox();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.FusedYawText = new System.Windows.Forms.TextBox();
            this.FusedPitchText = new System.Windows.Forms.TextBox();
            this.FusedRollText = new System.Windows.Forms.TextBox();
            this.FusedQuatYText = new System.Windows.Forms.TextBox();
            this.FusedQuatXText = new System.Windows.Forms.TextBox();
            this.FusedQuatWText = new System.Windows.Forms.TextBox();
            this.browse = new System.Windows.Forms.Button();
            this.pathText = new System.Windows.Forms.TextBox();
            this.EulerGraph = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.Mag = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.Gyr = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.Acc = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.glControl1 = new OpenTK.GLControl();
            this.close = new System.Windows.Forms.Button();
            this.open = new System.Windows.Forms.Button();
            this.scan = new System.Windows.Forms.Button();
            this.portsAvailable = new System.Windows.Forms.ComboBox();
            this.start_log = new System.Windows.Forms.CheckBox();
            this.WithoutGravity = new System.Windows.Forms.CheckBox();
            this.FusedQuatZText = new System.Windows.Forms.TextBox();
            ((System.ComponentModel.ISupportInitialize)(this.EulerGraph)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.Mag)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.Gyr)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.Acc)).BeginInit();
            this.SuspendLayout();
            // 
            // closeapp
            // 
            this.closeapp.BackColor = System.Drawing.Color.Transparent;
            this.closeapp.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("closeapp.BackgroundImage")));
            this.closeapp.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.closeapp.DialogResult = System.Windows.Forms.DialogResult.Cancel;
            this.closeapp.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.closeapp.Location = new System.Drawing.Point(843, 442);
            this.closeapp.Name = "closeapp";
            this.closeapp.Size = new System.Drawing.Size(37, 29);
            this.closeapp.TabIndex = 73;
            this.closeapp.UseVisualStyleBackColor = false;
            this.closeapp.Click += new System.EventHandler(this.closeapp_Click);
            // 
            // algorithm
            // 
            this.algorithm.FormattingEnabled = true;
            this.algorithm.Items.AddRange(new object[] {
            "Choose Algorithm",
            "Madgwick",
            "Mahony",
            "Gauss-Newton Complementary Filter",
            "Gauss-Newton Kalman Filter",
            "Extended Kalman Filter -Double Stage",
            "Extended Kalman Filter - IMU",
            "Mahony - IMU"});
            this.algorithm.Location = new System.Drawing.Point(620, 274);
            this.algorithm.Name = "algorithm";
            this.algorithm.Size = new System.Drawing.Size(248, 24);
            this.algorithm.TabIndex = 71;
            this.algorithm.SelectedIndexChanged += new System.EventHandler(this.algorithm_SelectedIndexChanged);
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(73, 199);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(41, 17);
            this.label5.TabIndex = 70;
            this.label5.Text = "Euler";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(11, 199);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(39, 17);
            this.label6.TabIndex = 69;
            this.label6.Text = "Quat";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(11, 182);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(81, 17);
            this.label2.TabIndex = 66;
            this.label2.Text = "Fused Data";
            // 
            // FusedYawText
            // 
            this.FusedYawText.Location = new System.Drawing.Point(73, 274);
            this.FusedYawText.Name = "FusedYawText";
            this.FusedYawText.Size = new System.Drawing.Size(55, 22);
            this.FusedYawText.TabIndex = 64;
            // 
            // FusedPitchText
            // 
            this.FusedPitchText.Location = new System.Drawing.Point(73, 247);
            this.FusedPitchText.Name = "FusedPitchText";
            this.FusedPitchText.Size = new System.Drawing.Size(55, 22);
            this.FusedPitchText.TabIndex = 63;
            // 
            // FusedRollText
            // 
            this.FusedRollText.Location = new System.Drawing.Point(73, 218);
            this.FusedRollText.Name = "FusedRollText";
            this.FusedRollText.Size = new System.Drawing.Size(55, 22);
            this.FusedRollText.TabIndex = 62;
            // 
            // FusedQuatYText
            // 
            this.FusedQuatYText.Location = new System.Drawing.Point(12, 274);
            this.FusedQuatYText.Name = "FusedQuatYText";
            this.FusedQuatYText.Size = new System.Drawing.Size(55, 22);
            this.FusedQuatYText.TabIndex = 60;
            // 
            // FusedQuatXText
            // 
            this.FusedQuatXText.Location = new System.Drawing.Point(12, 247);
            this.FusedQuatXText.Name = "FusedQuatXText";
            this.FusedQuatXText.Size = new System.Drawing.Size(55, 22);
            this.FusedQuatXText.TabIndex = 59;
            // 
            // FusedQuatWText
            // 
            this.FusedQuatWText.Location = new System.Drawing.Point(12, 218);
            this.FusedQuatWText.Name = "FusedQuatWText";
            this.FusedQuatWText.Size = new System.Drawing.Size(55, 22);
            this.FusedQuatWText.TabIndex = 58;
            // 
            // browse
            // 
            this.browse.Location = new System.Drawing.Point(12, 111);
            this.browse.Name = "browse";
            this.browse.Size = new System.Drawing.Size(75, 32);
            this.browse.TabIndex = 49;
            this.browse.Text = "Browse...";
            this.browse.UseVisualStyleBackColor = true;
            this.browse.Click += new System.EventHandler(this.browse_Click);
            // 
            // pathText
            // 
            this.pathText.Location = new System.Drawing.Point(12, 145);
            this.pathText.Name = "pathText";
            this.pathText.Size = new System.Drawing.Size(215, 22);
            this.pathText.TabIndex = 48;
            // 
            // EulerGraph
            // 
            this.EulerGraph.BackColor = System.Drawing.SystemColors.Control;
            this.EulerGraph.BorderlineColor = System.Drawing.SystemColors.Control;
            chartArea5.Name = "ChartArea1";
            this.EulerGraph.ChartAreas.Add(chartArea5);
            legend5.BackColor = System.Drawing.SystemColors.Control;
            legend5.Name = "Legend1";
            this.EulerGraph.Legends.Add(legend5);
            this.EulerGraph.Location = new System.Drawing.Point(574, 318);
            this.EulerGraph.Name = "EulerGraph";
            series13.ChartArea = "ChartArea1";
            series13.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series13.Legend = "Legend1";
            series13.Name = "Roll";
            series14.ChartArea = "ChartArea1";
            series14.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series14.Legend = "Legend1";
            series14.Name = "Pitch";
            series15.ChartArea = "ChartArea1";
            series15.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series15.Legend = "Legend1";
            series15.Name = "Yaw";
            this.EulerGraph.Series.Add(series13);
            this.EulerGraph.Series.Add(series14);
            this.EulerGraph.Series.Add(series15);
            this.EulerGraph.Size = new System.Drawing.Size(306, 153);
            this.EulerGraph.TabIndex = 47;
            this.EulerGraph.Text = "chart3";
            // 
            // Mag
            // 
            this.Mag.BackColor = System.Drawing.SystemColors.Control;
            this.Mag.BorderlineColor = System.Drawing.SystemColors.Control;
            chartArea6.Name = "ChartArea1";
            this.Mag.ChartAreas.Add(chartArea6);
            legend6.BackColor = System.Drawing.SystemColors.Control;
            legend6.Name = "Legend1";
            this.Mag.Legends.Add(legend6);
            this.Mag.Location = new System.Drawing.Point(283, 318);
            this.Mag.Name = "Mag";
            series16.ChartArea = "ChartArea1";
            series16.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series16.Legend = "Legend1";
            series16.Name = "Mag X";
            series17.ChartArea = "ChartArea1";
            series17.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series17.Legend = "Legend1";
            series17.Name = "Mag Y";
            series18.ChartArea = "ChartArea1";
            series18.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series18.Legend = "Legend1";
            series18.Name = "Mag Z";
            this.Mag.Series.Add(series16);
            this.Mag.Series.Add(series17);
            this.Mag.Series.Add(series18);
            this.Mag.Size = new System.Drawing.Size(306, 153);
            this.Mag.TabIndex = 46;
            // 
            // Gyr
            // 
            this.Gyr.BackColor = System.Drawing.SystemColors.Control;
            this.Gyr.BorderlineColor = System.Drawing.SystemColors.Control;
            chartArea7.Name = "ChartArea1";
            this.Gyr.ChartAreas.Add(chartArea7);
            legend7.BackColor = System.Drawing.SystemColors.Control;
            legend7.Name = "Legend1";
            this.Gyr.Legends.Add(legend7);
            this.Gyr.Location = new System.Drawing.Point(283, 170);
            this.Gyr.Name = "Gyr";
            series19.ChartArea = "ChartArea1";
            series19.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series19.Legend = "Legend1";
            series19.Name = "Gyr X";
            series20.ChartArea = "ChartArea1";
            series20.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series20.Legend = "Legend1";
            series20.Name = "Gyr Y";
            series21.ChartArea = "ChartArea1";
            series21.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series21.Legend = "Legend1";
            series21.Name = "Gyr Z";
            this.Gyr.Series.Add(series19);
            this.Gyr.Series.Add(series20);
            this.Gyr.Series.Add(series21);
            this.Gyr.Size = new System.Drawing.Size(306, 153);
            this.Gyr.TabIndex = 45;
            this.Gyr.Text = "Gyr";
            // 
            // Acc
            // 
            this.Acc.BackColor = System.Drawing.SystemColors.Control;
            this.Acc.BorderlineColor = System.Drawing.SystemColors.Control;
            chartArea8.Name = "ChartArea1";
            this.Acc.ChartAreas.Add(chartArea8);
            legend8.BackColor = System.Drawing.SystemColors.Control;
            legend8.Name = "Legend1";
            this.Acc.Legends.Add(legend8);
            this.Acc.Location = new System.Drawing.Point(283, 11);
            this.Acc.Name = "Acc";
            series22.ChartArea = "ChartArea1";
            series22.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series22.Legend = "Legend1";
            series22.Name = "Acc X";
            series23.ChartArea = "ChartArea1";
            series23.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series23.Legend = "Legend1";
            series23.Name = "Acc Y";
            series24.ChartArea = "ChartArea1";
            series24.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series24.Legend = "Legend1";
            series24.Name = "Acc Z";
            this.Acc.Series.Add(series22);
            this.Acc.Series.Add(series23);
            this.Acc.Series.Add(series24);
            this.Acc.Size = new System.Drawing.Size(306, 153);
            this.Acc.TabIndex = 44;
            // 
            // glControl1
            // 
            this.glControl1.BackColor = System.Drawing.SystemColors.Control;
            this.glControl1.ForeColor = System.Drawing.SystemColors.Control;
            this.glControl1.Location = new System.Drawing.Point(620, 12);
            this.glControl1.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.glControl1.Name = "glControl1";
            this.glControl1.Size = new System.Drawing.Size(248, 216);
            this.glControl1.TabIndex = 43;
            this.glControl1.VSync = false;
            this.glControl1.Load += new System.EventHandler(this.glControl1_Load);
            this.glControl1.Paint += new System.Windows.Forms.PaintEventHandler(this.glControl1_Paint);
            this.glControl1.Resize += new System.EventHandler(this.glControl1_Resize);
            // 
            // close
            // 
            this.close.Location = new System.Drawing.Point(12, 74);
            this.close.Name = "close";
            this.close.Size = new System.Drawing.Size(123, 27);
            this.close.TabIndex = 42;
            this.close.Text = "Close Port";
            this.close.UseVisualStyleBackColor = true;
            this.close.Click += new System.EventHandler(this.close_Click);
            // 
            // open
            // 
            this.open.Location = new System.Drawing.Point(12, 41);
            this.open.Name = "open";
            this.open.Size = new System.Drawing.Size(123, 27);
            this.open.TabIndex = 41;
            this.open.Text = "Open Port";
            this.open.UseVisualStyleBackColor = true;
            this.open.Click += new System.EventHandler(this.open_Click);
            // 
            // scan
            // 
            this.scan.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("scan.BackgroundImage")));
            this.scan.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.scan.ImageAlign = System.Drawing.ContentAlignment.MiddleRight;
            this.scan.Location = new System.Drawing.Point(200, 11);
            this.scan.Name = "scan";
            this.scan.Size = new System.Drawing.Size(27, 24);
            this.scan.TabIndex = 40;
            this.scan.UseVisualStyleBackColor = true;
            this.scan.Click += new System.EventHandler(this.scan_Click);
            // 
            // portsAvailable
            // 
            this.portsAvailable.FormattingEnabled = true;
            this.portsAvailable.Location = new System.Drawing.Point(12, 11);
            this.portsAvailable.Name = "portsAvailable";
            this.portsAvailable.Size = new System.Drawing.Size(182, 24);
            this.portsAvailable.TabIndex = 39;
            // 
            // start_log
            // 
            this.start_log.AutoSize = true;
            this.start_log.Location = new System.Drawing.Point(104, 118);
            this.start_log.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.start_log.Name = "start_log";
            this.start_log.Size = new System.Drawing.Size(88, 21);
            this.start_log.TabIndex = 75;
            this.start_log.Text = "Start Log";
            this.start_log.UseVisualStyleBackColor = true;
            // 
            // WithoutGravity
            // 
            this.WithoutGravity.AutoSize = true;
            this.WithoutGravity.Location = new System.Drawing.Point(487, 82);
            this.WithoutGravity.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.WithoutGravity.Name = "WithoutGravity";
            this.WithoutGravity.Size = new System.Drawing.Size(127, 21);
            this.WithoutGravity.TabIndex = 76;
            this.WithoutGravity.Text = "Without Gravity";
            this.WithoutGravity.UseVisualStyleBackColor = true;
            // 
            // FusedQuatZText
            // 
            this.FusedQuatZText.Location = new System.Drawing.Point(12, 303);
            this.FusedQuatZText.Name = "FusedQuatZText";
            this.FusedQuatZText.Size = new System.Drawing.Size(55, 22);
            this.FusedQuatZText.TabIndex = 61;
            // 
            // LSM9DS0
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.CancelButton = this.closeapp;
            this.ClientSize = new System.Drawing.Size(892, 482);
            this.Controls.Add(this.WithoutGravity);
            this.Controls.Add(this.start_log);
            this.Controls.Add(this.closeapp);
            this.Controls.Add(this.algorithm);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.FusedYawText);
            this.Controls.Add(this.FusedPitchText);
            this.Controls.Add(this.FusedRollText);
            this.Controls.Add(this.FusedQuatZText);
            this.Controls.Add(this.FusedQuatYText);
            this.Controls.Add(this.FusedQuatXText);
            this.Controls.Add(this.FusedQuatWText);
            this.Controls.Add(this.browse);
            this.Controls.Add(this.pathText);
            this.Controls.Add(this.EulerGraph);
            this.Controls.Add(this.Mag);
            this.Controls.Add(this.Gyr);
            this.Controls.Add(this.Acc);
            this.Controls.Add(this.glControl1);
            this.Controls.Add(this.close);
            this.Controls.Add(this.open);
            this.Controls.Add(this.scan);
            this.Controls.Add(this.portsAvailable);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.Name = "LSM9DS0";
            this.Text = "LSM9DS0";
            ((System.ComponentModel.ISupportInitialize)(this.EulerGraph)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.Mag)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.Gyr)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.Acc)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button closeapp;
        private System.Windows.Forms.ComboBox algorithm;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox FusedYawText;
        private System.Windows.Forms.TextBox FusedPitchText;
        private System.Windows.Forms.TextBox FusedRollText;
        private System.Windows.Forms.TextBox FusedQuatYText;
        private System.Windows.Forms.TextBox FusedQuatXText;
        private System.Windows.Forms.TextBox FusedQuatWText;
        private System.Windows.Forms.Button browse;
        private System.Windows.Forms.TextBox pathText;
        private System.Windows.Forms.DataVisualization.Charting.Chart EulerGraph;
        private System.Windows.Forms.DataVisualization.Charting.Chart Mag;
        private System.Windows.Forms.DataVisualization.Charting.Chart Gyr;
        private System.Windows.Forms.DataVisualization.Charting.Chart Acc;
        private OpenTK.GLControl glControl1;
        private System.Windows.Forms.Button close;
        private System.Windows.Forms.Button open;
        private System.Windows.Forms.Button scan;
        private System.Windows.Forms.ComboBox portsAvailable;
        private System.Windows.Forms.CheckBox start_log;
        private System.Windows.Forms.CheckBox WithoutGravity;
        private System.Windows.Forms.TextBox FusedQuatZText;
    }
}