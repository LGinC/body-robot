using System;
using System.IO.Ports;
using System.Windows;

namespace body_robot
{
    /// <summary>
    /// 自定义串口类，对系统串口类进一步封装
    /// </summary>
    class Uart
    {
        /// <summary>
        /// 系统串口对象
        /// </summary>
        private SerialPort p;

        /// <summary>
        /// 串口数据接收委托
        /// </summary>
        /// <param name="s">串口接收到的数据</param>
        public delegate void data_recieve(string s);
        /// <summary>
        /// 串口数据接收事件
        /// </summary>
        public event data_recieve DataRecieve;
        /// <summary>
        /// 获取系统可用COM口(串行通信端口)
        /// </summary>
        /// <returns>串口名字符串数组</returns>
        public static String[] get_com()
        {
            string[] ports = SerialPort.GetPortNames();
            Array.Sort(ports);
            return ports;
        }

        /// <summary>
        /// 串口构造函数
        /// </summary>
        /// <param name="baud">波特率</param>
        /// <param name="com">串口名</param>
        public Uart(int baud, String com)
        {
            Uart_open(baud, com);
        }

        public Uart() { }
        /// <summary>
        /// 系统串口数据接收事件处理函数，将接收到的数据交给上层的数据接收事件处理
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void P_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            string data = null;
            try
            {
                data = p.ReadLine();//进入此事件处理函数，表示有数据收到，读取一个字符串
            }
            catch(Exception ex)
            {
                Console.WriteLine(ex.ToString());
            }           
            DataRecieve(data);//执行上层的数据接收事件，将接收到的数据传入
        }

        /// <summary>
        /// 打开串口，作用和串口构造函数类似,都会新建一个系统串口对象
        /// </summary>
        /// <param name="baud">波特率</param>
        /// <param name="com">串口名</param>
        public void Uart_open(int baud, String com)
        {
            if (p == null)
            {
                p = new SerialPort(com, baud, Parity.None, 8, StopBits.One);//新建系统串口对象，串口名和波特率传入，无校验位，8位数据位，1位停止位
                p.WriteTimeout = 1000;              //写 超时 1s
                p.ReadTimeout = 1000;               //读 超时 1s
                p.Open();//打开串口
                p.DataReceived += P_DataReceived;//添加串口数据接收事件处理函数
            }
            else//若系统串口对象不为空则返回            
            {
                MessageBox.Show("串口已存在，需要先释放资源再打开", "警告");
                return;
            }

        }

        /// <summary>
        /// 打开串口，前提是串口对象已经传入对应参数
        /// </summary>
        public void Uart_open()
        {
            if (p == null)//若系统串口对象不存在则不能打开
            {
                MessageBox.Show("串口未给波特率及串口名", "错误");
                return;
            }

            if (p.IsOpen)//若串口已打开
            {
                System.Windows.MessageBox.Show("串口已经打开");
            }
            else//若未打开则尝试打开
            {
                try
                {
                   p.Open();
                }
                catch (Exception)
                {
                   System.Windows.MessageBox.Show("串口打开失败");
                }
            }
        }

        /// <summary>
        /// 关闭串口，不释放资源，可以再打开
        /// </summary>
        public void Uart_close()
        {

            if (p.IsOpen)//如果系统串口对象已经打开，则关闭
            {
                try
                {
                    p.Close();//尝试关闭，可能系统串口对象不存在，所以用try catch捕获异常
                }
                catch (Exception e)
                {
                    System.Windows.MessageBox.Show("串口关闭失败\n");
                    System.Windows.MessageBox.Show(e.ToString());
                }
            }
        }

        /// <summary>
        /// 释放串口所占用资源
        /// </summary>
        public void Uart_dispose()
        {
            if(p != null)//若系统串口对象存在，才进行资源释放
            {
                try
                {
                    p.DataReceived -= P_DataReceived;//移除串口数据事件处理
                    p.Dispose();//释放资源
                    p = null;
                }
                catch (Exception e)
                {
                    System.Windows.MessageBox.Show("串口释放失败\n");
                    System.Windows.MessageBox.Show(e.ToString());
                }
            }
        }
        /// <summary>
        /// 串口发送数据
        /// </summary>
        /// <param name="s">要发送的数据（字符串）</param>
        public void Send(String s)
        {
            p.Write(s);//串口写入数据
        }

        /// <summary>
        /// 串口读取一个字符串，只能是即时读取
        /// </summary>
        /// <returns>读取到的字符串</returns>
        public String Recieve()
        {
            return p.ReadLine();//读取一个字符串
        }

        /// <summary>
        /// 串口是否打开
        /// </summary>
        /// <returns>true串口已打开，false串口已关闭</returns>
        public bool IsOpen()
        {
            return p.IsOpen;
        }

    }
}
