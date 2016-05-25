using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Windows;
using System.Windows.Media;
using System.Threading.Tasks;
using System.Threading;
using System.Linq;

namespace body_robot
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        #region 成员变量定义
        ///// <summary>
        ///// 串口对象
        ///// </summary>
        //private Uart uart;

        ///// <summary>
        ///// wifi模块对象
        ///// </summary>
        //private ESP8266 wifi;

        /// <summary>
        /// socket连接对象
        /// </summary>
        private Connector conn;

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
        /// 上次PWM信号和本次PWM是否接近标志，true为接近，不发送PWM，false为差距较大，发送PWM
        /// </summary>
        private bool difference = false;

        /// <summary>
        /// 追踪者ID
        /// </summary>
        private ulong TrackID = 9;

        /// <summary>
        /// 速度控制
        /// </summary>
        private int speed = 0;

        /// <summary>
        /// 存放position数组的list
        /// </summary>
        private System.Collections.Generic.List<int[]> positions;

        /// <summary>
        /// 连接状态
        /// </summary>
        private string status;

        /// <summary>
        /// 数据帧处理是否是第一次进入
        /// </summary>
        private bool IsFirst = true;
        /// <summary>
        /// 画刷
        /// </summary>
        //private readonly Brush[] brushes;

        /// <summary>
        /// 位图
        /// </summary>
        //private CoordinateMapper coordinateMapper = null;

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
            positions = new List<int[]>();
            this.sensor = KinectSensor.GetDefault();
            conn = new Connector();//新建TCP连接对象
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
            this.sensor.Open();//打开kinect传感器      
            if (this.bodyFrameReader != null)//如果body数据帧阅读器存在则添加数据帧到达事件处理函数
            {
                this.bodyFrameReader.FrameArrived += Reader_FrameArrived;
            }
            freshIP();//刷新IP和控件
        }

        /// <summary>
        /// 刷新界面里的IP
        /// </summary>
        private void freshIP()
        {
            comboBox_hostIP.ItemsSource = conn.getHostIP();//获取本机IP添加至combobox
            comboBox_hostIP.SelectedIndex = comboBox_hostIP.Items.Count - 1;//默认选择最后一个       

        }

        /// <summary>
        /// 按钮B_connect点击事件处理函数
        /// </summary>
        /// <param name="sender">事件发送者对象，此处为按钮对象本身</param>
        /// <param name="e">路由事件参数</param>
        private void B_connect_clicked(object sender, RoutedEventArgs e)
        {
            try
            {
                if (conn.IsConnect == false)//点击之前为断开状态，则打开连接
                {
                    uint port;
                    string ip = comboBox_targetIP.SelectedItem.ToString();
                    if (UInt32.TryParse(TextBox_Port.Text, out port) == false)//如果TextBox_Port选择的值不能转为Uint则抛出异常
                    {
                        throw new InvalidOperationException("端口Port请输入数字");
                    }
                    conn.connectComplete += Conn_connectComplete;//增加连接完成事件处理函数              
                    conn.OpenConnection(ip, port);//连接    
                    conn.ShowReceiveData += Conn_ShowReceiveData;//添加数据接收事件处理函数
                    B_connect.Background = Brushes.Red;
                    B_connect.Content = "connected";

                }
                else//点击之前为连接状态，则断开连接
                {
                    freshIP();
                    conn.CloseConnection();//断开连接
                    conn.ShowReceiveData -= Conn_ShowReceiveData;//移除处理函数
                    conn.connectComplete -= Conn_connectComplete;//移除连接完成事件处理函数
                    B_connect.Content = "disconnect";
                    B_connect.Background = Brushes.Yellow;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString(), "错误");
            }

        }

        /// <summary>
        /// socket连接完成事件处理函数
        /// </summary>
        private void Conn_connectComplete()
        {
            status = Constants.connect_success;
            Console.WriteLine(status);
            try
            {
                conn.Send(Constants.init_PWM);//发送初始PWM
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        /// <summary>
        /// socket接收数据处理函数
        /// </summary>
        /// <param name="data">接收到的数据</param>
        private void Conn_ShowReceiveData(string data)
        {
            this.Dispatcher.Invoke(new Action(() =>
            {
                TextBox_show.AppendText(data);
            }));
        }

        /// <summary>
        /// 窗口关闭时回调函数，主要用于释放和关闭设备
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)//窗口关闭时回调
        {
            if (conn.IP_list.Count == 0)
                conn.CloseConnection();
            if (this.sensor.IsOpen)
            {
                this.sensor.Close();
                this.bodyFrameReader.FrameArrived -= Reader_FrameArrived;
            }
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
            if (conn.IsConnect == true)    //待socket连接完成再进行数据处理
            {                         
                using (BodyFrame b = e.FrameReference.AcquireFrame())   //获取body数据帧，using会在括号结束后自动销毁申请的数据结构
                {
                    try
                    {
                        body = new Body[b.BodyCount];   //新建body数组
                        b.GetAndRefreshBodyData(body);  //从body数据帧里获取body对象放入body数组                        
                    }
                    catch (Exception ie)
                    {
                        Console.WriteLine(ie.Message);
                        return;
                    }
                }
                foreach (var item in body)  //遍历每个body
                {

                    #region 关节角度计算 
                    if (item.IsTracked)    //body处于被追踪状态时处理
                    {
                        if (item.JointOrientations[JointType.Head].Orientation.Z < 2) //Z轴距离大于2不处理
                        {
                            int[] position_pre = new int[Constants.POSITION_LENTH];   //新建PWM数组      

                            if (TrackID == 9)//moren
                            {
                                TrackID = item.TrackingId;
                            }

                            if (TrackID == item.TrackingId)   //只有当TrackID和当前ID一致时进行处理
                            {
                               
                                int c = 0;
                                foreach (var j in item.Joints)//遍历所有关节结构体
                                {
                                    joints[c++] = ToAngle.filter(j.Value);//过滤
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
                                #endregion
                                if (position_pre[(int)angle.servos.HipRight] == 0)//当左右髋读取值为0时，赋默认值60
                                {
                                    position_pre[(int)angle.servos.HipRight] = 60;
                                }
                                if (position_pre[(int)angle.servos.HipLeft] == 0)
                                {
                                    position_pre[(int)angle.servos.HipLeft] = 60;
                                }
                                if(IsFirst)//如果是第一次进入
                                {
                                    Console.WriteLine("第一次进入");
                                    IsFirst = false;//设置第一次进入标志位false
                                    position = position_pre;//直接将本次计算所得数据赋给上一组数据
                                    return;//返回
                                }
                                difference = true;
                                int diff = 0;

                                #region 舵机PMW异常为0，数据帧丢弃
                                for (int i = 0; i < Constants.POSITION_LENTH; i++)
                                {
                                    switch (position_pre[i])
                                    {
                                        case 0:
                                            if (i != (int)angle.servos.FootLeft && i != (int)angle.servos.FootRight && i != (int)angle.servos.Head && i != Constants.POSITION_LENTH - 1)
                                            {
                                                return;//非脚部和头部舵机  值为0，计算异常，数据帧丢弃
                                            }
                                            break;
                                        case Constants.INVALID_JOINT_VALUE:
                                            return;//有舵机值异常，数据帧丢                                          
                                        default:
                                            break;
                                    }
                                    #endregion

                                    #region 若计算的PWM值变化小于5，返回不发送
                                    diff = Math.Abs(position[i] - position_pre[i]);//计算本帧舵机PWM值和上帧的差值
                                    if (diff > 5)//若无任何舵机变化值大于5则不发送
                                        difference = false;
                                    if (diff > 40)//变化值大于40说明计算异常，数据帧丢弃
                                        return;
                                }
                                position = position_pre;
                                if (difference == false)//无变化
                                {
                                    difference = true;
                                    return;//返回
                                }
                                #endregion
                                #region 平均滤波                       
                                                                        //开始执行PWM算术平均滤波
                                if (speed++ < Constants.frame_count)//先判断，再自增
                                {
                                    positions.Add(position);//把当前舵机值添加至列表
                                    return;//返回
                                }
                                else//采集处理了超过frame_count帧数据
                                {
                                    speed = 0;//复位计数器
                                    positions.Add(position);//添加当前帧至列表
                                    position = filter_position(positions);//执行算术平均滤波
                                    positions.Clear();//清除列表
                                }
                                #endregion

                                #region 姿态检测
                                position[Constants.POSITION_LENTH - 1] = 0;//设置position数组尾部
                                ToAngle.PoseDect(ref position);//姿态检测
                                String PWM = "";
                                for (int i = 0; i < Constants.POSITION_LENTH - 1; i++)//组成PWM数据帧
                                {
                                    if (position[i] >= 0 && position[i] < 10)//1-9补两个0
                                    {
                                        PWM += "00" + position[i];
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
                                #endregion

                                #region PWM数据帧发送和显示
                                this.Dispatcher.Invoke(new Action(() => //在与 Dispatcher 关联的线程上同步执行指定的委托
                                {
                                    /*  在 WPF 中，只有创建 DispatcherObject 的线程才能访问该对象。例如，一个从主 UI 线程派生的后台线程不能更新在该 UI 线程上创建的 Button 的内容。为了使该后台线程能够访问 Button 的 Content 属性，该后台线程必须将此工作委托给与该 UI 线程关联的 Dispatcher。它使用Invoke 或 BeginInvoke完成。Invoke是同步，BeginInvoke 是异步。该操作将按指定的 DispatcherPriority 添加到 Dispatcher 的事件队列中。
                                  Invoke 是同步操作；因此，直到回调返回之后才会将控制权返回给调用对象。              */

                                    //String p;
                                    //TextBox_data.AppendText(PWM.Substring(0, 2));
                                    //p = System.Text.RegularExpressions.Regex.Replace(PWM.Substring(2, 51), @".{3}","$0 ");
                                    //TextBox_data.AppendText(p + "\n");
                                    TextBox_data.AppendText(PWM + "\n");//textbox追加PWM
                                    scroller_dataShow.ScrollToEnd();//滚动至尾部                             
                                }));

                                try
                                {
                                    conn.Send(PWM);
                                }
                                catch (Exception ex)
                                {
                                    MessageBox.Show(ex.ToString());
                                }
                                #endregion
                            }
                        }

                    }

                }

            }
            //Console.WriteLine("time:" + sw.Elapsed);
        }

        /// <summary>
        /// 舵机数据帧 算术平均滤波
        /// </summary>
        /// <param name="positions">舵机PWM数组list</param>
        /// <returns>滤波后的舵机PWM数组</returns>
        private int[] filter_position(List<int[]> positions)
        {
            int i, count = positions[0].Length;
            int[] s = new int[count];
            foreach (var p in positions)
            {
                for (i = 0; i < count; i++)
                {
                    s[i] += p[i];
                }
            }
            for (i = 0; i < count; i++)
            {
                s[i] = s[i] / positions.Count;
            }
            return s;
        }


        /// <summary>
        /// comboBox选择项改变事件处理函数
        /// </summary>
        /// <param name="sender">发送者对象，这里是combobox本身</param>
        /// <param name="e">路由事件参数</param>
        private void comboBox_hostIP_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            conn.ipFreshComplete += targetIP_change;//添加IP刷新完成事件处理函数
            conn.AllIP(comboBox_hostIP.SelectedValue.ToString());//更新局域网在线IP
        }

        /// <summary>
        /// 添加IP刷新完成事件处理函数
        /// </summary>
        private void targetIP_change()
        {
            comboBox_targetIP.ItemsSource = null;//清空comboBox里的选项
            comboBox_targetIP.ItemsSource = conn.IP_list;//重新添加
            comboBox_targetIP.SelectedIndex = comboBox_targetIP.Items.Count - 1;//默认选择最后一个
            conn.ipFreshComplete -= targetIP_change;//删除处理函数
            
        }

        ///// <summary>
        ///// B_open按钮点击函数
        ///// </summary>
        ///// <param name="sender"></param>
        ///// <param name="e"></param>
        //private void B_open_clicked(object sender, RoutedEventArgs e)
        //{
        //    comboBox_com.SelectedIndex = comboBox_com.Items.Count - 1;//默认选择最后一个
        //    if (IsOpen == true)
        //    {
        //        if (wifi.finish == true)
        //        {
        //            try
        //            {
        //                //deinit_peripheral();
        //                B_open.Content = "close";
        //                B_open.Background = Brushes.Red;
        //            }
        //            catch (Exception ex)
        //            {
        //                MessageBox.Show(ex.ToString());
        //                IsOpen = false;
        //            }
        //            //comboBox_com.Items.Clear();
        //            //String[] a = Uart.get_com();//添加串口名
        //            //foreach (var item in a)
        //            //{
        //            //    comboBox_com.Items.Add(item);
        //            //}
        //            //comboBox_com.SelectedIndex = comboBox_com.Items.Count - 1;

        //        }
        //        else
        //            MessageBox.Show("初始化未完成，不能操作");
        //    }
        //    else
        //    {
        //        //if (uart == null || wifi.finish == true)
        //        {
        //            try
        //            {
        //                //init_peripheral();
        //                B_open.Content = "open";
        //                B_open.Background = Brushes.Yellow;
        //            }
        //            catch (Exception ex)
        //            {
        //                MessageBox.Show(ex.ToString() + "\n" + IsOpen);
        //                comboBox_com.Items.Clear();
        //                String[] a = Uart.get_com();//添加串口名
        //                foreach (var item in a)
        //                {
        //                    comboBox_com.Items.Add(item);
        //                }
        //                comboBox_com.SelectedIndex = comboBox_com.Items.Count - 1;
        //                IsOpen = false;
        //            }
        //        }
        //        else
        //            MessageBox.Show("初始化未完成，不能操作");
        //    }
        //}

        ///// <summary>
        ///// 初始化外围设备，包括串口、wifi和kinect传感器
        ///// </summary>
        //public void init_peripheral()
        //{
        //    int baud = int.Parse(comboBox_BaudRate.SelectedValue.ToString());//选中的波特率转为int
        //    string com = comboBox_com.SelectedValue.ToString();//选中的com口转为string
        //    this.uart = new Uart(baud, com);//新建uart对象               
        //    this.sensor.Open();//打开kinect传感器
        //    wifi = new ESP8266(this.uart);//新建wifi对象
        //    wifi.DataShow += Wifi_DataShow;//添加wifi数据回显事件处理函数
        //    if (this.bodyFrameReader != null)//如果body数据帧阅读器存在则添加数据帧到达事件处理函数
        //    {
        //        this.bodyFrameReader.FrameArrived += Reader_FrameArrived;
        //    }
        //    wifi.initESP();//wifi初始化
        //    IsOpen = true;
        //}

        ///// <summary>
        ///// 外围设备释放函数 
        ///// </summary>
        //public void deinit_peripheral()
        //{
        //    if (this.sensor.IsOpen)
        //    {
        //        this.sensor.Close();
        //        this.bodyFrameReader.FrameArrived -= Reader_FrameArrived;
        //    }
        //    wifi.disposeESP();//退出AP
        //    uart.Uart_close();//串口关闭
        //    IsOpen = false;
        //}

    }
}
