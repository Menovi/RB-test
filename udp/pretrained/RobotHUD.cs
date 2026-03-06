using UnityEngine;

public class RobotHUD : MonoBehaviour
{
    private G1UdpController controller;

    // Exact mapping to your uploaded .pkl files
    private string[] motionLabels = {
        "Idle",      // ID 0
        "Walk - Stand",  //ID 1
        "Basic Walk",        // ID 2
        "Dance",             // ID 3
        "Squat",             // ID 4
        "Kick Walk",         // ID 5
        "Waltz",             // ID 6
        "Crouch Walk",       // ID 7
        "Walk Stand",        // ID 8
        "Air Kick"           // ID 9
    };

    void Start()
    {
        controller = GetComponent<G1UdpController>();
    }

    void OnGUI()
    {
        // Fixed window for the Motion Menu
        GUI.Box(new Rect(10, 10, 200, 420), "G1 Motion Library");

        for (int i = 0; i < motionLabels.Length; i++)
        {
            Rect buttonRect = new Rect(20, 40 + (i * 40), 170, 35);

            // Highlight the active motion button
            Color defaultColor = GUI.color;
            if (controller.currentMode == i) GUI.color = Color.cyan;

            if (GUI.Button(buttonRect, motionLabels[i]))
            {
                controller.SetMotion(i);
            }

            GUI.color = defaultColor;
        }
    }
}