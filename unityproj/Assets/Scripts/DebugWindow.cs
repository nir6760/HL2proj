using UnityEngine;

public class DebugWindow : MonoBehaviour
{
    TextMesh textMesh;

    // Use this for initialization
    void Start()
    {
        //textMesh = gameObject.GetComponentInChildren<TextMesh>();
    }

    void OnEnable()
    {
        Application.logMessageReceived += LogMessage;
    }

    void OnDisable()
    {
        Application.logMessageReceived -= LogMessage;
    }

    public void LogMessage(string message, string stackTrace, LogType type)
    {
        if (gameObject.GetComponentInChildren<TextMesh>().text.Length > 2000)
        {
            gameObject.GetComponentInChildren<TextMesh>().text = message + "\n";
        }
        else
        {
            gameObject.GetComponentInChildren<TextMesh>().text += message + "\n";
        }
    }
}