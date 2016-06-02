using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
public class UDPconn
{

    private UdpClient udp;

	public UDPconn()
	{
        udp = new UdpClient();
	}

    public int connect(string RemoteIp, int port)
    {
        IPAddress addr = IPAddress.Parse(RemoteIp);
        udp.Connect(addr, port);
        return 1;
    }

    public int Send(string data)
    {
        if (udp.Active)
            throw new InvalidOperationException("没有完成远程连接，无法进行UDP发送");

        byte[] SendData = Encoding.Default.GetBytes(data);
        udp.BeginSend(SendData, SendData.Length, SendCallBack, null);
    }

    /// <summary>
    /// 发送回调函数，当发送完成时调用
    /// </summary>
    /// <param name="ar">异步操作状态对象</param>
    private void SendCallback(IAsyncResult ar)
    {
        udp.EndSend(ar);//当前socket结束发送                       
    }


}
