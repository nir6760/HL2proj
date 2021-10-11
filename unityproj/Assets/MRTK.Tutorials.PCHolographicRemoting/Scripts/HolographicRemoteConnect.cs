using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.WSA;

public class HolographicRemoteConnect : MonoBehaviour
{

    [SerializeField]
    private string IP;

    private bool connected = false;

    [SerializeField, Tooltip("The configuration information for the remote connection.")]
    private Microsoft.MixedReality.OpenXR.Remoting.RemotingConfiguration remotingConfiguration = new Microsoft.MixedReality.OpenXR.Remoting.RemotingConfiguration { RemotePort = 8265, MaxBitrateKbps = 20000 };

    public void Connect()
    {
        connected = true;

        remotingConfiguration.RemoteHostName = IP;

        StartCoroutine(Microsoft.MixedReality.OpenXR.Remoting.AppRemoting.Connect(remotingConfiguration));
    }


    private void OnGUI()
    {
        IP = GUI.TextField(new Rect(10, 10, 200, 30), IP, 25);

        string button = (connected ? "Disconnect" : "Connect");

        if (GUI.Button(new Rect(220, 10, 100, 30), button))
        {
            if (connected)
            {
                Microsoft.MixedReality.OpenXR.Remoting.AppRemoting.Disconnect();
                connected = false;
            }
            else
                Connect();
            Debug.Log(button);

        }

    }
}