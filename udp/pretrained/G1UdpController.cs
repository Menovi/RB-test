using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System;

public class G1UdpController : MonoBehaviour
{
    private UdpClient udpClient;
    private IPEndPoint remoteEndPoint;

    [Header("Network")]
    public string ip = "127.0.0.1";
    public int port = 5005;

    [Header("Current Selection")]
    // 0 is now treated as "Idle/Stand", 1-8 are your motions
    public int currentMode = 0;
    private float[] dataToSend = new float[30];

    void Start()
    {
        udpClient = new UdpClient();
        remoteEndPoint = new IPEndPoint(IPAddress.Parse(ip), port);
    }

    void Update()
    {
        SendToMujoco();
    }

    public void SetMotion(int modeID)
    {
        currentMode = modeID;
        Debug.Log("Commanding Motion ID: " + currentMode);
    }

    void SendToMujoco()
    {
        try
        {
            // Reset buffer
            Array.Clear(dataToSend, 0, dataToSend.Length);

            // We only care about the 30th slot (index 29) for the Motion ID
            dataToSend[29] = (float)currentMode;

            byte[] sendBytes = new byte[dataToSend.Length * 4];
            Buffer.BlockCopy(dataToSend, 0, sendBytes, 0, sendBytes.Length);
            udpClient.Send(sendBytes, sendBytes.Length, remoteEndPoint);
        }
        catch (Exception e) { Debug.LogError(e.Message); }
    }
}