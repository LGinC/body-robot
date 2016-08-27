using System;
using System.Collections.Generic;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace body_robot
{
    public class Connector
    {
        /// <summary>
        /// 系统socket对象
        /// </summary>
        private Socket conn;

        /// <summary>
        /// 自定义操作对象，socket收发时使用
        /// </summary>
        private State state = new State();

        /// <summary>
        /// 显示socket接收数据 委托
        /// </summary>
        /// <param name="data">socket接收到的数据</param>
        public delegate void showReceiveData(String data);

        /// <summary>
        /// 显示socket接收数据事件
        /// </summary> 
        public event showReceiveData ShowReceiveData;

        /// <summary>
        /// 连接完成事件
        /// </summary>
        public event Action connectComplete;

        /// <summary>
        /// 局域网在线IP列表
        /// </summary>
        private List<string> _IP_list = new List<string>();

        /// <summary>
        /// socket是否连接标志，true为已连接
        /// </summary>
        public bool IsConnect = false;

        /// <summary>
        /// socket异常
        /// </summary>
        private Exception socketException;

        /// <summary>
        /// 计数器，计算ping局域网主机个数
        /// </summary>
        private int c = 0;

        /// <summary>
        /// 刷新局域网所有在线IP完成事件 
        /// </summary>
        public event Action ipFreshComplete;

        /// <summary>
        /// socket连接关闭事件
        /// </summary>
        public event Action ConnectClose;

        /// <summary>
        /// 自定义比较器对象
        /// </summary>
        private IpComparer comparer = new IpComparer();

        private static ManualResetEvent timeoutobject = new ManualResetEvent(false);

        /// <summary>
        /// 打开socket连接
        /// </summary>
        /// <param name="ip">目标IP地址</param>
        /// <param name="port">端口</param>
        public void OpenConnection(string ip, uint port)
        {            
            if (port < 1024 || port > 65535)//输入端口未在正常值内抛出异常
                throw new Exception("port端口未正确输入（1024-65535）");
            conn = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);//新建系统socket对象     
            conn.ReceiveTimeout = Constants.receive_timeout;//接收超时时间设置
            conn.SendTimeout = Constants.send_timeout;//发送超时时间设置
            conn.BeginConnect(ip, (int)port, connectCallBack, state);//开始异步连接
            if(timeoutobject.WaitOne(Constants.connect_timeout, false))//当前线程阻塞connect_timeout  ms，若在此时间内进行timeoutobject.set则返回线程继续执行并返回true，否则超时后返回false
            {//在规定时间内返回
                if (IsConnect == false)//连接失败
                {
                    throw socketException;//抛出socketException异常
                }
            }
            else//超时返回
            {
                throw new TimeoutException("socket连接超时");//抛出异常
            }           
            conn.BeginReceive(state.buffer, 0, state.buffer.Length, 0, new AsyncCallback(ReadCallBack), state);//开始异步接收                     
        }

        /// <summary>
        /// 连接回调函数，连接完成时调用
        /// </summary>
        /// <param name="ar">异步操作对象</param>
        private void connectCallBack(IAsyncResult ar)
        {
            try
            {             
                if (conn != null)//如果已经连接
                {
                    conn.EndConnect(ar);//结束异步连接
                    IsConnect = true;
                    connectComplete();//触发连接完成事件
                    state.ar = ar;
                }               
            }
            catch(Exception e)
            {
                Console.WriteLine(e.ToString());
                if (e.Message.Contains("the target machine actively refused it"))//如果当前异常信息里包含对应内容
                    socketException = new Exception("连接失败，目标机器拒绝访问");//新建异常，目的是为了在提示窗口显示中文
                else
                    socketException = e;//将当前异常赋给socketException
                IsConnect = false;
            }
            finally//无论是否连上
            {
                timeoutobject.Set();//都返回主线程
            }
        }

        /// <summary>
        /// 关闭连接
        /// </summary>
        public void CloseConnection()
        {
            IsConnect = false;
            _IP_list.Clear();
            conn.EndReceive(state.ar);//结束异步接收
            conn.Disconnect(false);//关闭连接，不再使用此socket
            conn.Dispose();//释放所占资源
            
        }

        /// <summary>
        /// socket数据接收回掉函数，收到数据时调用
        /// </summary>
        /// <param name="ar">异步操作状态对象</param>
        private void ReadCallBack(IAsyncResult ar)
        {
            string receive = string.Empty;//初始化空string
            ShowReceiveData(receive);//调用数据接收事件
            state.ar = ar;
            try
            {
                int m_read = conn.EndReceive(ar);//结束接收
                if (m_read > 0)//接收到的字节数大于0
                {
                    receive = Encoding.Default.GetString(state.buffer);//将缓冲区数据转为string
                    receive = receive.Substring(0, receive.IndexOf('\u0000'));
                    ShowReceiveData(receive);//调用数据接收事件                         
                    conn.BeginReceive(state.buffer, 0, state.buffer.Length, 0, ReadCallBack, state);//开始异步接收                    
                }
            }
            catch(Exception e)
            {
                Console.WriteLine(e.ToString());
                if(e.Message.Contains("An existing connection was forcibly closed by the remote host"))
                {
                    ConnectClose();//触发连接关闭事件
                }
            }            
        }

        /// <summary>
        /// 数据发送函数
        /// </summary>
        /// <param name="message">要发送的数据</param>
        public void Send(String message)
        {
            byte[] data = Encoding.Default.GetBytes(message);//将message转为byte[]
            conn.BeginSend(data, 0, data.Length, 0, SendCallback, state);//开始异步发送
        }

        /// <summary>
        /// 发送回调函数，当发送完成时调用
        /// </summary>
        /// <param name="ar">异步操作状态对象</param>
        private void SendCallback(IAsyncResult ar)
        {
            conn.EndSend(ar);//当前socket结束发送                       
        }

        /// <summary>
        /// 获取在ip这个网段内所有在线的ip，并添加至IP_list
        /// </summary>
        /// <param name="ip">主机所在局域网IP</param>
        public void AllIP(string ip)
        {
            string IP_C = ip.Remove(ip.LastIndexOf('.'));//去掉最后一个.后面的数字，即去掉主机号，后面ping通的就添加
            _IP_list.Clear();//清除IP列表
            c = 0;
            try
            {
                for (int i = 0; i <= 255; i++)
                {
                    Ping ping = new Ping();//新建ping
                    ping.PingCompleted += Ping_PingCompleted;//ping通过完成触发事件
                    string IP_ping = IP_C + "." + i.ToString();
                    ping.SendAsync(IP_ping, 1000, null);//发送异步ping包
                }
            }
            catch(Exception e)
            {
                Console.WriteLine(e.ToString());
            }
        }

        /// <summary>
        /// 获取本机IP列表
        /// </summary>
        /// <returns>返回IP列表</returns>
        public List<string> getHostIP()
        {
            List<string> ip = new List<string>();//新建string列表
            IPAddress[] p = Dns.GetHostEntry(Dns.GetHostName()).AddressList;//获取本机ip地址列表
            foreach(var i in p)//遍历
            {
                if(!i.IsIPv6LinkLocal && !i.IsIPv6Teredo)//如果是非IPv6地址
                {
                    ip.Add(i.ToString());//添加至list
                   //Console.WriteLine(i.ToString());
                }
            }
            return ip;
        }

        /// <summary>
        /// ping完成事件处理函数，ping完成时触发
        /// </summary>
        /// <param name="sender">发送者对象</param>
        /// <param name="e">ping完成事件参数</param>
        private void Ping_PingCompleted(object sender, PingCompletedEventArgs e)
        {
            string ip = e.Reply.Address.ToString();
            if (e.Reply.Status == IPStatus.Success)//如果ping成功
            {
                if(!_IP_list.Contains(ip))//当返回成功IP不在列表内时添加
                {
                    _IP_list.Add(ip);//添加返回IP至列表
                }                
            }
            if(c++ == 255)//局域网最后一个IPping结束
            {
                _IP_list.Sort(comparer);//列表排序
                ipFreshComplete();//触发ipFreshComplete事件
            }                
        }

        /// <summary>
        /// 私有字段IP_list的get方法
        /// </summary>
        public List<string> IP_list
        {
            get { return _IP_list; }
        }
        
    }

    /// <summary>
    /// 自定义容器类
    /// </summary>
    class State
    {

        /// <summary>
        /// 异步操作
        /// </summary>
        public IAsyncResult ar;

        /// <summary>
        /// 接收缓冲区
        /// </summary>
        public byte[] buffer = new byte[2048];
    }

    /// <summary>
    /// 自定义比较器类，用于局域网IP比较，只对IP最后的4位，即主机号进行比较
    /// </summary>
    class IpComparer : IComparer<string>
    {
        int IComparer<string>.Compare(string x, string y)
        {
            
            if (x == null)
            {
                if (y == null)
                {
                    return 0;//相等
                }
                else
                {
                    return -1;//小于
                }
            }
            else
            {
                if (y == null)
                {
                    return 1;//大于
                }
                else
                {
                    string a = x.Remove(0, x.LastIndexOf(".") + 1);//移除最后一个.（包括.）之前的部分
                    string b = y.Remove(0, y.LastIndexOf(".") + 1);
                    Int32 i = Convert.ToInt32(a);
                    Int32 j = Convert.ToInt32(b);
                    return i - j;
                }
            }

        }
    }

}
