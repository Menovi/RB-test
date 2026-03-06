using UnityEditor;
using UnityEngine;

public class RobotHUD : MonoBehaviour
{
    private G1UdpController controller;
    private Vector2 scrollPos;
    private bool showHUD = true;

    void Start()
    {
        controller = GetComponent<G1UdpController>();
    }

    void OnGUI()
    {
        if (!showHUD) return;

        // Create a container for the whole UI
        GUI.Box(new Rect(10, 10, 280, Screen.height - 20), "G1 Manual Control HUD");


        // Start the scrollable area
        scrollPos = GUILayout.BeginScrollView(scrollPos, GUILayout.Width(270), GUILayout.Height(Screen.height - 50));

        GUILayout.Space(10);
        if (GUILayout.Button("SPACE: RESET POSE", GUILayout.Height(30))) controller.SetStandingPose();
        GUILayout.Space(10);


        // Add this after the Reset Pose button
        Color defaultColor = GUI.color;
        GUI.color = controller.currentMode == 1f ? Color.green : Color.red;
        string modeText = controller.currentMode == 1f ? "WALK MODE: ON (AI)" : "WALK MODE: OFF (MANUAL)";

        if (GUILayout.Button(modeText, GUILayout.Height(40)))
        {
            controller.ToggleWalkingMode();
        }
        GUI.color = defaultColor;
        GUILayout.Space(10);


        // --- LEFT LEG (Indices 0 - 5) ---
        GUILayout.Label("--- LEFT LEG ---", GUI.skin.label);
        DrawJointControl("Hip Pitch (NP 7/4)", 0);
        DrawJointControl("Hip Roll (NP 8/5)", 1);
        DrawJointControl("Hip Yaw (NP 9/6)", 2);
        DrawJointControl("Knee (NP / or *)", 3);
        DrawJointControl("Ankle Pitch (NumLk/Clr)", 4);
        DrawJointControl("Ankle Roll (Ins/Del)", 5);

        GUILayout.Space(10);

        // --- RIGHT LEG (Indices 6 - 11) ---
        GUILayout.Label("--- RIGHT LEG ---", GUI.skin.label);
        DrawJointControl("Hip Pitch (Alpha 1)", 6);
        DrawJointControl("Hip Roll (Alpha 2)", 7);
        DrawJointControl("Hip Yaw (Alpha 3)", 8);
        DrawJointControl("Knee (Alpha 4)", 9);
        DrawJointControl("Ankle Pitch (Alpha 5)", 10);
        DrawJointControl("Ankle Roll (Alpha 6)", 11);
        GUILayout.Label("(Use L-Shift to decrease Right Leg)", EditorStyles.miniLabel);

        GUILayout.Space(10);

        // --- WAIST (Indices 12 - 14) ---
        GUILayout.Label("--- WAIST ---", GUI.skin.label);
        DrawJointControl("Waist Yaw (Left/Right)", 12);
        DrawJointControl("Waist Roll (Home/End)", 13);
        DrawJointControl("Waist Pitch (Up/Down)", 14);

        GUILayout.Space(10);

        // --- LEFT ARM (Indices 15 - 21) ---
        GUILayout.Label("--- LEFT ARM ---", GUI.skin.label);
        DrawJointControl("Shoulder Pitch (Q/E)", 15);
        DrawJointControl("Shoulder Roll (A/D)", 16);
        DrawJointControl("Shoulder Yaw (Z/X)", 17);
        DrawJointControl("Elbow (C/V)", 18);
        DrawJointControl("Wrist Roll (F)", 19);
        DrawJointControl("Wrist Pitch (G)", 20);
        DrawJointControl("Wrist Yaw (H)", 21);

        GUILayout.Space(10);

        // --- RIGHT ARM (Indices 22 - 28) ---
        GUILayout.Label("--- RIGHT ARM ---", GUI.skin.label);
        DrawJointControl("Shoulder Pitch (R/T)", 22);
        DrawJointControl("Shoulder Roll (Y/U)", 23);
        DrawJointControl("Shoulder Yaw (I/O)", 24);
        DrawJointControl("Elbow (P/V)", 25);
        DrawJointControl("Wrist Roll (L)", 26);
        DrawJointControl("Wrist Pitch (K)", 27);
        DrawJointControl("Wrist Yaw (J)", 28);

        GUILayout.EndScrollView();
    }

    void DrawJointControl(string label, int index)
    {
        GUILayout.BeginVertical(GUI.skin.box);
        GUILayout.Label(label, GUILayout.Width(200));
        GUILayout.BeginHorizontal();

        // These buttons mimic the keyboard input
        if (GUILayout.RepeatButton("-"))
            controller.q_target[index] -= 0.8f * Time.deltaTime;

        GUILayout.Box(controller.q_target[index].ToString("F2"), GUILayout.Width(50));

        if (GUILayout.RepeatButton("+"))
            controller.q_target[index] += 0.8f * Time.deltaTime;

        GUILayout.EndHorizontal();
        GUILayout.EndVertical();
    }
}