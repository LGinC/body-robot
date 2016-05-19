using System;
using System.Diagnostics;
using System.IO.Ports;
using System.Threading.Tasks;

namespace body_robot
{

    class ESP8266
    {
        #region 成员变量定义
        /// <summary>
        /// 串口对象
        /// </summary>
        private Uart u;

        /// <summary>
        /// wifi对象状态
        /// </summary>
        private string status = "";

        /// <summary>
        /// 定时器，用于计算当前执行时间
        /// </summary>
        private Stopwatch sw;

        /// <summary>
        /// 数据接收委托
        /// </summary>
        /// <param name="show">接收到的数据</param>
        public delegate void DataRecieve(String show);

        /// <summary>
        /// 数据接收事件
        /// </summary>
        public event DataRecieve DataShow;

        /// <summary>
        /// 初始化完成标志
        /// </summary>
        public bool finish = false;

        /// <summary>
        /// 数据发送完成标志
        /// </summary>
        public bool SendFinish = true;

        #endregion
        /// <summary>
        /// 传入串口，添加串口接收事件
        /// </summary>
        /// <param name="uart">串口对象</param>
        public ESP8266(Uart uart)
        {
            if (uart == null)            //串口对象为空则抛出异常
                throw new InvalidOperationException("串口未创建或未打开");
            this.u = uart;
            u.DataRecieve += OnDataReceived;//添加串口接收数据事件处理函数
            sw = new Stopwatch();//新建计时器
        }

        /// <summary>
        /// wifi模块初始化，建立连接
        /// </summary>
        public async void initESP()
        {
            finish = false;
            sw.Start();//启动计时器
            DataShow(sw.Elapsed + " wifi模块初始化开始：\n");//主窗体显示
                u.Send("AT+CWMODE=1\r\n");//设置wifi模块模式，为station模式
                await Task.Delay(500);
                u.Send("AT+CWQAP\r\n");//退出AP
                await Task.Delay(500);
                u.Send("AT+CWJAP=\"XXX\",\"12345678\"\r\n");//加入AP，ssid=ESP8266 passwd=012345678
                await Task.Delay(5000);
                u.Send("AT+CIFSR\r\n");
                await Task.Delay(500);
                u.Send("AT+CIPMUX=0\r\n");//设置单路连接
                await Task.Delay(500);
                u.Send("AT+CIPSTART=\"TCP\",\"192.168.4.1\",5000\r\n");//建立TCP连接，目标IP 192.168.4.1 端口5000
                await Task.Delay(2000);
                SendPWM(Constants.init_PWM);//发送初始PWM数据
                DataShow("wifi模块初始化结束：\n");
                finish = true;//wifi模块初始化标志设为true           
            //for (int i = 0; i < 10; i++)
            //{
            //    u.Send("AT+GMR\n");
            //    await Task.Delay(1000);
            //}
        }

        /// <summary>
        /// 退出AP，关闭计时器
        /// </summary>
        public void disposeESP()
        {
            
            u.Send("AT + CWQAP");//退出AP
            sw.Stop();//关闭计时器
        }

        /// <summary>
        /// 串口数据接收事件处理函数
        /// </summary>
        /// <param name="sender">发送者对象</param>
        /// <param name="e">串口数据接收时间参数</param>
        private void OnDataReceived(string s)
        {
            Console.Write(s);
            DataShow(sw.Elapsed.ToString("G") + " " + s);//在主窗体输出时间和数据
            status = s;
            //if (status.Contains("CWJAP:1"))
            //{
            //    this.init_failed = true;
            //    initESP();
            //}                
        }

        /// <summary>
        /// wifi发送PWM信号
        /// </summary>
        /// <param name="PWM">PWM</param>
        public  void SendPWM(string PWM)
        {
            if (SendFinish == true)//若发送完成标志为true则设置为false，表示正在发送
                SendFinish = !SendFinish;

            u.Send("AT+CIPSEND=53\r\n");//发送17*3 = 51 位 长度数据
            //System.Threading.Thread.Sleep(100);//当前线程延时100ms      
            u.Send(PWM + "\r\n");       //发送PWM  /r/n 是AT命令的结尾部分           
            if (SendFinish == false)     //若发送完成标志为false则设置为true，表示已发送完成
                SendFinish = !SendFinish;
        }
    }
}