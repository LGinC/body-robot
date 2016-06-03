using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
public class UDPconn
{

    private UdpClient udp;
    private IPEndPoint broadcast;
    public Action<string> ReceiveShow;
    private bool _IsConnected = false;
    public bool IsCOnnected { get { return _IsConnected; } }

	public UDPconn()
	{
        udp = new UdpClient();
	}

    public int connect(string RemoteIp, int port)
    {
        IPAddress addr = IPAddress.Parse(RemoteIp);
        broadcast = new IPEndPoint(addr, port);
        try
        {
            udp.Connect(broadcast);
            udp.BeginReceive(ReceiveCallBack, null);
            _IsConnected = true;
            return 1;
        }
        catch(Exception e)
        {            
            _IsConnected = false;
            throw e;
        }
    }

    private void ReceiveCallBack(IAsyncResult ar)
    {
        try
        {
            byte[] Receive = udp.EndReceive(null, ref broadcast);
            string result = Encoding.Default.GetString(Receive);
            ReceiveShow(result);
        }
        catch(Exception e)
        {
            _IsConnected = false;
            throw e;
        }
    }

    public int Send(string data)
    {
        if (udp != null)
            throw new InvalidOperationException("没有完成远程连接，无法进行UDP发送");
        byte[] SendData = Encoding.Default.GetBytes(data);
        try
        {
            udp.Send(SendData, SendData.Length);
            return 1;
        }
        catch(Exception e)
        {
            _IsConnected = false;
            throw e;
        }

    }
}
