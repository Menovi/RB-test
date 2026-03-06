/* manual controls only

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

    [Header("Joint Targets")]
    public float[] q_target = new float[29];

    void Start()
    {
        udpClient = new UdpClient();
        remoteEndPoint = new IPEndPoint(IPAddress.Parse(ip), port);
        SetStandingPose();
    }
    
    void Update()
    {
        HandleInput();
        SendToMujoco();
    }
    

    void HandleInput()  
    {
        float speed = 0.5f* Time.deltaTime;

        // --- LEFT LEG (NUMPAD 7,4,1 / 8,5,2) ---
        if (Input.GetKey(KeyCode.Keypad7)) q_target[0] += speed; // Hip Pitch
        if (Input.GetKey(KeyCode.Keypad4)) q_target[0] -= speed;
        if (Input.GetKey(KeyCode.Keypad8)) q_target[1] += speed; // Hip Roll
        if (Input.GetKey(KeyCode.Keypad5)) q_target[1] -= speed;
        if (Input.GetKey(KeyCode.Keypad9)) q_target[2] += speed; // Hip Yaw
        if (Input.GetKey(KeyCode.Keypad6)) q_target[2] -= speed;
        if (Input.GetKey(KeyCode.KeypadDivide))  q_target[3] += speed; // Knee
        if (Input.GetKey(KeyCode.KeypadMultiply))q_target[3] -= speed;
        if (Input.GetKey(KeyCode.Numlock)) q_target[4] += speed; // Ankle Pitch
        if (Input.GetKey(KeyCode.Clear))   q_target[4] -= speed; 
        if (Input.GetKey(KeyCode.Insert))  q_target[5] += speed; // Ankle Roll
        if (Input.GetKey(KeyCode.Delete))  q_target[5] -= speed;

        // --- RIGHT LEG (ALPHA 1-6) ---
        if (Input.GetKey(KeyCode.Alpha1)) q_target[6] += speed; // Hip Pitch
        if (Input.GetKey(KeyCode.Alpha2)) q_target[7] += speed; // Hip Roll
        if (Input.GetKey(KeyCode.Alpha3)) q_target[8] += speed; // Hip Yaw
        if (Input.GetKey(KeyCode.Alpha4)) q_target[9] += speed; // Knee
        if (Input.GetKey(KeyCode.Alpha5)) q_target[10] += speed; // Ankle Pitch
        if (Input.GetKey(KeyCode.Alpha6)) q_target[11] += speed; // Ankle Roll
        // (Hold Left Shift + Number to decrease Right Leg joints)
        if (Input.GetKey(KeyCode.LeftShift)) {
            if (Input.GetKey(KeyCode.Alpha1)) q_target[6] -= speed * 2;
            if (Input.GetKey(KeyCode.Alpha2)) q_target[7] -= speed * 2;
            if (Input.GetKey(KeyCode.Alpha3)) q_target[8] -= speed * 2;
            if (Input.GetKey(KeyCode.Alpha4)) q_target[9] -= speed * 2;
            if (Input.GetKey(KeyCode.Alpha5)) q_target[10] -= speed * 2;
            if (Input.GetKey(KeyCode.Alpha6)) q_target[11] -= speed * 2;
        }

        // --- WAIST (ARROWS & HOME/END) ---
        if (Input.GetKey(KeyCode.UpArrow))    q_target[14] += speed; // Pitch
        if (Input.GetKey(KeyCode.DownArrow))  q_target[14] -= speed;
        if (Input.GetKey(KeyCode.LeftArrow))  q_target[12] += speed; // Yaw
        if (Input.GetKey(KeyCode.RightArrow)) q_target[12] -= speed;
        if (Input.GetKey(KeyCode.Home))       q_target[13] += speed; // Roll
        if (Input.GetKey(KeyCode.End))        q_target[13] -= speed;

        // --- LEFT ARM (Q,E,A,D,Z,X,C) ---
        if (Input.GetKey(KeyCode.Q)) q_target[15] += speed; // Sh Pitch
        if (Input.GetKey(KeyCode.E)) q_target[15] -= speed;
        if (Input.GetKey(KeyCode.A)) q_target[16] += speed; // Sh Roll
        if (Input.GetKey(KeyCode.D)) q_target[16] -= speed;
        if (Input.GetKey(KeyCode.Z)) q_target[17] += speed; // Sh Yaw
        if (Input.GetKey(KeyCode.X)) q_target[17] -= speed;
        if (Input.GetKey(KeyCode.C)) q_target[18] += speed; // Elbow
        if (Input.GetKey(KeyCode.V)) q_target[18] -= speed;
        if (Input.GetKey(KeyCode.F)) q_target[19] += speed; // Wrist Roll
        if (Input.GetKey(KeyCode.G)) q_target[20] += speed; // Wrist Pitch
        if (Input.GetKey(KeyCode.H)) q_target[21] += speed; // Wrist Yaw

        // --- RIGHT ARM (R,T,Y,U,I,O,P) ---
        if (Input.GetKey(KeyCode.R)) q_target[22] += speed; // Sh Pitch
        if (Input.GetKey(KeyCode.T)) q_target[22] -= speed;
        if (Input.GetKey(KeyCode.Y)) q_target[23] += speed; // Sh Roll
        if (Input.GetKey(KeyCode.U)) q_target[23] -= speed;
        if (Input.GetKey(KeyCode.I)) q_target[24] += speed; // Sh Yaw
        if (Input.GetKey(KeyCode.O)) q_target[24] -= speed;
        if (Input.GetKey(KeyCode.P)) q_target[25] += speed; // Elbow
        if (Input.GetKey(KeyCode.L)) q_target[26] += speed; // Wrist Roll
        if (Input.GetKey(KeyCode.K)) q_target[27] += speed; // Wrist Pitch
        if (Input.GetKey(KeyCode.J)) q_target[28] += speed; // Wrist Yaw

        // Emergency Reset
        if (Input.GetKeyDown(KeyCode.Space)) SetStandingPose();
    }

    public void SetStandingPose()
    {
        for (int i = 0; i < q_target.Length; i++) q_target[i] = 0;
        q_target[0] = -0.5f; q_target[6] = -0.5f; // Hips
        q_target[3] = 0.8f; q_target[9] = 0.8f;   // Knees
        q_target[4] = -0.4f; q_target[10] = -0.4f; // Ankles
        q_target[15] = 0.3f; q_target[22] = 0.3f; // Shoulders
    }

    void SendToMujoco()
    {
        try
        {
            byte[] sendBytes = new byte[q_target.Length * 4];
            Buffer.BlockCopy(q_target, 0, sendBytes, 0, sendBytes.Length);
            udpClient.Send(sendBytes, sendBytes.Length, remoteEndPoint);
        }
        catch (Exception e) { Debug.LogError(e.Message); }
    }
}


*/


// toggle switching 


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

    [Header("Joint Targets")]
    public float[] q_target = new float[29];

    [Header("Mode Control")]
    // 0 = Manual, 1 = Pretrained Walk
    public float currentMode = 0f;
    private float[] dataToSend = new float[30];

    void Start()
    {
        udpClient = new UdpClient();
        remoteEndPoint = new IPEndPoint(IPAddress.Parse(ip), port);
        SetStandingPose();
    }

    void Update()
    {
        // We only allow manual keyboard input if we are in Manual Mode (0)
        if (currentMode == 0f)
        {
            HandleInput();
        }

        // Always send data so MuJoCo knows the heartbeat/mode
        SendToMujoco();
    }

    // Call this from a Unity UI Button to switch modes
    public void ToggleWalkingMode()
    {
        if (currentMode == 0f)
        {
            currentMode = 1f;
            Debug.Log("Switched to PRETRAINED WALK MODE");
        }
        else
        {
            currentMode = 0f;
            SetStandingPose(); // Reset pose when returning to manual
            Debug.Log("Switched to MANUAL MODE");
        }
    }

    void HandleInput()
    {
        float speed = 0.5f * Time.deltaTime;

        // --- Key Mappings ---
        if (Input.GetKey(KeyCode.Keypad7)) q_target[0] += speed;
        if (Input.GetKey(KeyCode.Keypad4)) q_target[0] -= speed;
        if (Input.GetKey(KeyCode.Keypad8)) q_target[1] += speed;
        if (Input.GetKey(KeyCode.Keypad5)) q_target[1] -= speed;
        if (Input.GetKey(KeyCode.Keypad9)) q_target[2] += speed;
        if (Input.GetKey(KeyCode.Keypad6)) q_target[2] -= speed;
        if (Input.GetKey(KeyCode.KeypadDivide)) q_target[3] += speed;
        if (Input.GetKey(KeyCode.KeypadMultiply)) q_target[3] -= speed;
        if (Input.GetKey(KeyCode.Numlock)) q_target[4] += speed;
        if (Input.GetKey(KeyCode.Clear)) q_target[4] -= speed;
        if (Input.GetKey(KeyCode.Insert)) q_target[5] += speed;
        if (Input.GetKey(KeyCode.Delete)) q_target[5] -= speed;

        if (Input.GetKey(KeyCode.Alpha1)) q_target[6] += speed;
        if (Input.GetKey(KeyCode.Alpha2)) q_target[7] += speed;
        if (Input.GetKey(KeyCode.Alpha3)) q_target[8] += speed;
        if (Input.GetKey(KeyCode.Alpha4)) q_target[9] += speed;
        if (Input.GetKey(KeyCode.Alpha5)) q_target[10] += speed;
        if (Input.GetKey(KeyCode.Alpha6)) q_target[11] += speed;

        if (Input.GetKey(KeyCode.LeftShift))
        {
            if (Input.GetKey(KeyCode.Alpha1)) q_target[6] -= speed * 2;
            if (Input.GetKey(KeyCode.Alpha2)) q_target[7] -= speed * 2;
            if (Input.GetKey(KeyCode.Alpha3)) q_target[8] -= speed * 2;
            if (Input.GetKey(KeyCode.Alpha4)) q_target[9] -= speed * 2;
            if (Input.GetKey(KeyCode.Alpha5)) q_target[10] -= speed * 2;
            if (Input.GetKey(KeyCode.Alpha6)) q_target[11] -= speed * 2;
        }

        if (Input.GetKey(KeyCode.UpArrow)) q_target[14] += speed;
        if (Input.GetKey(KeyCode.DownArrow)) q_target[14] -= speed;
        if (Input.GetKey(KeyCode.LeftArrow)) q_target[12] += speed;
        if (Input.GetKey(KeyCode.RightArrow)) q_target[12] -= speed;
        if (Input.GetKey(KeyCode.Home)) q_target[13] += speed;
        if (Input.GetKey(KeyCode.End)) q_target[13] -= speed;

        if (Input.GetKey(KeyCode.Q)) q_target[15] += speed;
        if (Input.GetKey(KeyCode.E)) q_target[15] -= speed;
        if (Input.GetKey(KeyCode.A)) q_target[16] += speed;
        if (Input.GetKey(KeyCode.D)) q_target[16] -= speed;
        if (Input.GetKey(KeyCode.Z)) q_target[17] += speed;
        if (Input.GetKey(KeyCode.X)) q_target[17] -= speed;
        if (Input.GetKey(KeyCode.C)) q_target[18] += speed;
        if (Input.GetKey(KeyCode.V)) q_target[18] -= speed;
        if (Input.GetKey(KeyCode.F)) q_target[19] += speed;
        if (Input.GetKey(KeyCode.G)) q_target[20] += speed;
        if (Input.GetKey(KeyCode.H)) q_target[21] += speed;

        if (Input.GetKey(KeyCode.R)) q_target[22] += speed;
        if (Input.GetKey(KeyCode.T)) q_target[22] -= speed;
        if (Input.GetKey(KeyCode.Y)) q_target[23] += speed;
        if (Input.GetKey(KeyCode.U)) q_target[23] -= speed;
        if (Input.GetKey(KeyCode.I)) q_target[24] += speed;
        if (Input.GetKey(KeyCode.O)) q_target[24] -= speed;
        if (Input.GetKey(KeyCode.P)) q_target[25] += speed;
        if (Input.GetKey(KeyCode.L)) q_target[26] += speed;
        if (Input.GetKey(KeyCode.K)) q_target[27] += speed;
        if (Input.GetKey(KeyCode.J)) q_target[28] += speed;

        if (Input.GetKeyDown(KeyCode.Space)) SetStandingPose();
    }

    public void SetStandingPose()
    {
        for (int i = 0; i < q_target.Length; i++) q_target[i] = 0;
        q_target[0] = -0.5f; q_target[6] = -0.5f; // Hips
        q_target[3] = 0.8f; q_target[9] = 0.8f;   // Knees
        q_target[4] = -0.4f; q_target[10] = -0.4f; // Ankles
        q_target[15] = 0.3f; q_target[22] = 0.3f; // Shoulders
    }

    void SendToMujoco()
    {
        try
        {
            // 1. Copy our 29 joint targets into the buffer
            Array.Copy(q_target, dataToSend, 29);

            // 2. Set the 30th value as the Mode
            dataToSend[29] = currentMode;

            // 3. Convert all 30 floats to bytes
            byte[] sendBytes = new byte[dataToSend.Length * 4];
            Buffer.BlockCopy(dataToSend, 0, sendBytes, 0, sendBytes.Length);

            udpClient.Send(sendBytes, sendBytes.Length, remoteEndPoint);
        }
        catch (Exception e) { Debug.LogError(e.Message); }
    }
}









