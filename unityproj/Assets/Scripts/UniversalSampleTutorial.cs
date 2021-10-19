using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO;

#if !UNITY_EDITOR
using Windows.Networking;
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
#endif

//Able to act as a reciever 
public class UniversalSampleTutorial : MonoBehaviour
{

#if !UNITY_EDITOR
    StreamSocket socket;
    StreamSocketListener listener;
    String port;
    String message;
#endif

    // Use this for initialization
    void Start()
    {
#if !UNITY_EDITOR
        listener = new StreamSocketListener();
        port = "13000";
        listener.ConnectionReceived += Listener_ConnectionReceived;
        listener.Control.KeepAlive = false;
        Listener_Start();
#endif
    }

#if !UNITY_EDITOR
    private async void Listener_Start()
    {
        Debug.Log("Listener started");
        try
        {
            await listener.BindServiceNameAsync(port);
        }
        catch (Exception e)
        {
            Debug.Log("Error: " + e.Message);
        }

        Debug.Log("Listening");
    }

    private async void Listener_ConnectionReceived(StreamSocketListener sender, StreamSocketListenerConnectionReceivedEventArgs args)
    {
        Debug.Log("Connection received");

        try
        {
            while (true)
            {

                /*using (var dw = new DataWriter(args.Socket.OutputStream))
                {
                    dw.WriteString("Hello There");
                    await dw.StoreAsync();
                    dw.DetachStream();
                }*/
                MemoryStream memory_stream = new MemoryStream();
                using (var dr = new DataReader(args.Socket.InputStream))
                {
                    dr.InputStreamOptions = InputStreamOptions.Partial;
                    uint count_bytes = dr.UnconsumedBufferLength;
                    await dr.LoadAsync(count_bytes);
                    var input = dr.ReadString(count_bytes);
                    //Debug.Log("received: " + input);
                    byte[] data = System.Text.Encoding.UTF8.GetBytes(input);
                    //Debug.Log("recived " + i);
                    memory_stream.Write(data, 0, Convert.ToInt32(count_bytes));

                }
            }
        }
        catch (Exception e)
        {
            Debug.Log("disconnected!!!!!!!! " + e);
        }

    }

#endif

    void Update()
    {
    }
}