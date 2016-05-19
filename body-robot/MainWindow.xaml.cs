using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Windows;
using System.Windows.Media;

namespace body_robot
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        #region 成员变量定义
        /// <summary>
        /// 串口对象
        /// </summary>
        private Uart uart;

        /// <summary>
        /// wifi模块对象
        /// </summary>
        private ESP8266 wifi;

        /// <summary>
        /// kinect传感器对象
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// body数据 数组
        /// </summary>
        private Body[] body;

        /// <summary>
        /// 存放过滤后的关节结构体数组
        /// </summary>
        private Joint[] joints;

        /// <summary>
        /// 存放PWM数组
        /// </summary>
        private int[] position;

        /// <summary>
        /// bodyFram的阅读器
        /// </summary>
        private BodyFrameReader bodyFrameReader;

        /// <summary>
        /// 串口是否打开标志
        /// </summary>
        private bool IsOpen = false;

        /// <summary>
        /// 上次PWM信号和本次PWM是否接近标志，true为接近，不发送PWM，false为差距较大，发送PWM
        /// </summary>
        private bool difference = false;

        /// <summary>
        /// 追踪者ID
        /// </summary>
        private ulong TrackID = 0;

        /// <summary>
        /// 速度控制
        /// </summary>
        private int speed = 0;

        /// <summary>
        /// 画刷
        /// </summary>
        private readonly Brush[] brushes;

        /// <summary>
        /// 位图
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// 骨骼
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// 用于计算舵机PWM的angle对象
        /// </summary>
        private angle ToAngle;
        #endregion
        /// <summary>
        /// 主窗口构造函数
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
            this.bones = new List<Tuple<JointType, JointType>>();
            ToAngle = new angle();
            joints = new Joint[Body.JointCount];
            position = new int[Constants.POSITION_LENTH];
            this.sensor = KinectSensor.GetDefault();
            // Torso躯干
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm右手
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm左手
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg右腿
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg左腿
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));
        }
        /// <summary>
        /// 窗口加载时的回调函数，用于初始化
        /// </summary>
        /// <param name="sender">发送者对象</param>
        /// <param name="e">路由事件参数</param>
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            this.bodyFrameReader = this.sensor.BodyFrameSource.OpenReader();//打开阅读器
            comboBox_BaudRate.Items.Add("2400");//添加波特率
            comboBox_BaudRate.Items.Add("115200");
            comboBox_BaudRate.SelectedIndex = comboBox_BaudRate.Items.Count - 1;//默认选择最后一个            
            //init_peripheral();
            String[] a = Uart.get_com();//添加串口名
            foreach (var item in a)
            {
                comboBox_com.Items.Add(item);
            }
            comboBox_com.SelectedIndex = comboBox_com.Items.Count - 1;
            B_open_clicked(B_open, new RoutedEventArgs());//调用打开按钮点击事件
        }

        /// <summary>
        /// wifi回传数据显示
        /// </summary>
        /// <param name="show">wifi回传的数据</param>
        private void Wifi_DataShow(string show)
        {
            this.Dispatcher.Invoke(new Action(() =>
            {
                TextBox_show.AppendText(show);//追加至TextBox_show
                TextBox_show.ScrollToEnd();//滚动至末尾
            }));

        }

        /// <summary>
        /// 窗口关闭时回调函数，主要用于释放和关闭设备
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)//窗口关闭时回调
        {
            deinit_peripheral();
            if (this.bodyFrameReader != null)//body数据帧阅读器存在则关闭
            {
                this.bodyFrameReader.Dispose();//阅读器释放资源
                this.bodyFrameReader = null;
            }
            if (this.sensor != null)//传感器存在则关闭
            {
                this.sensor.Close();
                this.sensor = null;
            }

        }

        /// <summary>
        /// Kinect数据帧事件处理函数
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            Stopwatch sw = new Stopwatch(); //新建计时器
            sw.Start();                     //开始计时器
            if (wifi.finish)    //待wifi初始化完成再进行数据处理
            {
                if (speed++ == 20)             //数据帧处理速度控制，每对应帧过后处理一次，kinect数据帧获取速度约为30fps，为了和传输模块同步可修改此数值进行数据发送速度调整
                {
                    int[] position_pre = new int[Constants.POSITION_LENTH];   //新建PWM数组
                    speed = 0;                          //重置速度控制
                    using (BodyFrame b = e.FrameReference.AcquireFrame())   //获取body数据帧，using会在括号结束后自动销毁申请的数据结构
                    {
                        try
                        {
                            body = new Body[b.BodyCount];   //新建body数组
                            b.GetAndRefreshBodyData(body);  //从body数据帧里获取body对象放入body数组
                        }
                        catch (Exception ie)
                        {
                            Console.WriteLine(ie.ToString());
                            return;
                        }
                    }

                    foreach (var item in body)  //遍历每个body
                    {

                        #region 关节角度计算        
                        if (item.JointOrientations[JointType.Head].Orientation.Z < 1.5) //Z轴距离大于1.5不识别
                        {
                            if (TrackID == 0)
                            {
                                TrackID = item.TrackingId;
                                Wifi_DataShow("ID:" + TrackID.ToString());
                            }
                            if (TrackID == item.TrackingId)   //只有当TrackID和当前ID一致时进行处理
                            {
                                if (item.IsTracked)    //body处于被追踪状态时处理
                                {
                                    int c = 0;
                                    foreach(var j in item.Joints)
                                    {
                                        joints[c++] = ToAngle.filter(j.Value);
                                    }
                                    //Console.WriteLine("State:{0}    x:{1} y:{2}", item.LeanTrackingState, item.Lean.X, item.Lean.Y);
                                    position_pre[(int)angle.servos.ShoulderRight] = ToAngle.ToPWMshoulder(joints[(int)JointType.ShoulderRight], joints[(int)JointType.ElbowRight], joints[(int)JointType.HandRight]);
                                    position_pre[(int)angle.servos.ShoulderLeft] = ToAngle.ToPWMshoulder(joints[(int)JointType.ShoulderLeft], joints[(int)JointType.ElbowLeft], joints[(int)JointType.HandLeft]);
                                    position_pre[(int)angle.servos.ElbowRight] = ToAngle.ToPWM_elbow(joints[(int)JointType.ShoulderRight], joints[(int)JointType.ElbowRight]);
                                    position_pre[(int)angle.servos.HandRight] = ToAngle.ToPWM_hand(joints[(int)JointType.ShoulderRight], joints[(int)JointType.ElbowRight], joints[(int)JointType.HandRight]);
                                    position_pre[(int)angle.servos.ElbowLeft] = ToAngle.ToPWM_elbow(joints[(int)JointType.ShoulderLeft], joints[(int)JointType.ElbowLeft]);
                                    position_pre[(int)angle.servos.HandLeft] = ToAngle.ToPWM_hand(joints[(int)JointType.ShoulderLeft], joints[(int)JointType.ElbowLeft], joints[(int)JointType.HandLeft]);
                                    position_pre[(int)angle.servos.HipRight] = ToAngle.ToPWM_hip(joints[(int)JointType.HipRight], joints[(int)JointType.KneeRight]);
                                    position_pre[(int)angle.servos.HipLeft] = ToAngle.ToPWM_hip(joints[(int)JointType.HipLeft], joints[(int)JointType.KneeLeft]);
                                    position_pre[(int)angle.servos.ThighRight] = ToAngle.ToPWM_thigh(joints[(int)JointType.HipRight], joints[(int)JointType.KneeRight]);
                                    position_pre[(int)angle.servos.ThighLeft] = ToAngle.ToPWM_thigh(joints[(int)JointType.HipLeft], joints[(int)JointType.KneeLeft]);
                                    position_pre[(int)angle.servos.KneeRight] = ToAngle.ToPWM_knee(joints[(int)JointType.HipRight], joints[(int)JointType.KneeRight], joints[(int)JointType.AnkleRight]);
                                    position_pre[(int)angle.servos.AnkleRight] = ToAngle.ToPWM_ankle(joints[(int)JointType.AnkleRight]);
                                    position_pre[(int)angle.servos.KneeLeft] = ToAngle.ToPWM_knee(joints[(int)JointType.HipLeft], joints[(int)JointType.KneeLeft], joints[(int)JointType.AnkleLeft]);
                                    position_pre[(int)angle.servos.AnkleLeft] = ToAngle.ToPWM_ankle(joints[(int)JointType.AnkleLeft]);

                                    if (position_pre[(int)angle.servos.HipRight] == 0)//当左右髋读取值为0时，赋默认值60
                                    {
                                        position_pre[(int)angle.servos.HipRight] = 60;
                                    }
                                    if (position_pre[(int)angle.servos.HipLeft] == 0)
                                    {
                                        position_pre[(int)angle.servos.HipLeft] = 60;
                                    }
                                    difference = true;
                                    for (int i = 0; i < Constants.POSITION_LENTH; i++)
                                    {
                                        if (position[i] - position_pre[i] > 5 || position[i] - position_pre[i] < -5)//若无任何舵机变化值大于5则不发送
                                            difference = false;
                                        position[i] = position_pre[i];
                                    }
                                    position[Constants.POSITION_LENTH - 1] = 0;
                                    if (difference == true)
                                    {
                                        difference = true;
                                        return;
                                    }
                                    ToAngle.PoseDect(ref position);
                                    String PWM = "";
                                    for (int i = 0; i < Constants.POSITION_LENTH - 1; i++)
                                    {
                                        if (position[i] >= 0 && position[i] < 10)//1-9补两个0
                                        {
                                            PWM += "00" + position[i];
                                            if (position[i] == 0)
                                            {
                                                if (i != (int)angle.servos.FootLeft && i != (int)angle.servos.FootRight && i != (int)angle.servos.Head && i != Constants.POSITION_LENTH - 1)
                                                {
                                                    Console.WriteLine("invalid 0 value:{0} invalid PWM:{1}", PWM, position[i]);
                                                    PWM = "";
                                                    return;
                                                }
                                            }
                                        }



                                        else if (position[i] < 100 && position[i] >= 10)//10-99补一个0
                                            PWM += "0" + position[i];
                                        else if (position[i] > 255 || position[i] < 0)//大于255或者小于等于0为异常
                                        {
                                            Console.WriteLine("invalid value:{0} invalid PWM:{1}", PWM, position[i]);
                                            PWM = "";
                                            return;
                                        }
                                        else//100 - 250不补0
                                            PWM += position[i];
                                    }
                                    if (position[Constants.POSITION_LENTH - 1] > 0)
                                    {
                                        PWM = PWM.Insert(0, "p" + position[Constants.POSITION_LENTH - 1]);
                                    }
                                    else
                                    {
                                        PWM = PWM.Insert(0, "sb");
                                    }


                                    //    foreach (var ite in position)
                                    //    {
                                    //        Console.Write(ite + "\t");
                                    //        if (ite < 0 || ite > 255)
                                    //        {
                                    //            Console.Write("\n");
                                    //            return;
                                    //        }
                                    //    }

                                    this.Dispatcher.Invoke(new Action(() =>
                                    {
                                        //String p;
                                        //TextBox_data.AppendText(PWM.Substring(0, 2));
                                        //p = System.Text.RegularExpressions.Regex.Replace(PWM.Substring(2, 51), @".{3}","$0 ");
                                        //TextBox_data.AppendText(p + "\n");
                                        TextBox_data.AppendText(PWM + "\n");
                                        TextBox_data.ScrollToEnd();
                                    }));
                                    wifi.SendPWM(PWM);
                                    //foreach (var p in position)
                                    //{
                                    //    Console.Write(p + " ");
                                    //}                                     
                                    //Console.Write("\n");
                                }
                            }

                        }
                        #endregion
                    }
                }

            }
            //Console.WriteLine("time:" + sw.Elapsed);
        }

        /// <summary>
        /// B_open按钮点击函数
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void B_open_clicked(object sender, RoutedEventArgs e)
        {       
            comboBox_com.SelectedIndex = comboBox_com.Items.Count - 1;//默认选择最后一个
            if (IsOpen == true)
            {
                if (wifi.finish == true)
                {     
                    try
                    {
                        deinit_peripheral();
                        B_open.Content = "close";
                        B_open.Background = Brushes.Red;
                    }            
                    catch(Exception ex)
                    {
                        MessageBox.Show(ex.ToString());
                        IsOpen = false;
                    }                
                    comboBox_com.Items.Clear();
                    String[] a = Uart.get_com();//添加串口名
                    foreach (var item in a)
                    {
                        comboBox_com.Items.Add(item);
                    }
                    comboBox_com.SelectedIndex = comboBox_com.Items.Count - 1;
                   
                }
                else
                    MessageBox.Show("初始化未完成，不能操作");
            }
            else
            {
                if (uart == null || wifi.finish == true)
                {
                    try
                    {
                        init_peripheral();
                        B_open.Content = "open";
                        B_open.Background = Brushes.Yellow;
                    }
                    catch(Exception ex)
                    {
                        MessageBox.Show(ex.ToString() + "\n" + IsOpen);
                        comboBox_com.Items.Clear();
                        String[] a = Uart.get_com();//添加串口名
                        foreach (var item in a)
                        {
                            comboBox_com.Items.Add(item);
                        }
                        comboBox_com.SelectedIndex = comboBox_com.Items.Count - 1;
                        IsOpen = false;
                    }
                }
                else
                    MessageBox.Show("初始化未完成，不能操作");
            }
        }

        /// <summary>
        /// 初始化外围设备，包括串口、wifi和kinect传感器
        /// </summary>
        public void init_peripheral()
        {
                int baud = int.Parse(comboBox_BaudRate.SelectedValue.ToString());//选中的波特率转为int
                string com = comboBox_com.SelectedValue.ToString();//选中的com口转为string
                this.uart = new Uart(baud, com);//新建uart对象               
                this.sensor.Open();//打开kinect传感器
                wifi = new ESP8266(this.uart);//新建wifi对象
                wifi.DataShow += Wifi_DataShow;//添加wifi数据回显事件处理函数
                if (this.bodyFrameReader != null)//如果body数据帧阅读器存在则添加数据帧到达事件处理函数
                {
                    this.bodyFrameReader.FrameArrived += Reader_FrameArrived;
                }
                wifi.initESP();//wifi初始化
                IsOpen = true;
        }

        /// <summary>
        /// 外围设备释放函数 
        /// </summary>
        public void deinit_peripheral()
        {
                if (this.sensor.IsOpen)
                {
                    this.sensor.Close();
                    this.bodyFrameReader.FrameArrived -= Reader_FrameArrived;
                }
                wifi.disposeESP();//退出AP
                uart.Uart_close();//串口关闭
                IsOpen = false;
        }

    }
}
