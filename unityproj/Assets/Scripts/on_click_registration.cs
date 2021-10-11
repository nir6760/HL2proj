using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

using System.Threading;
using Dummiesman;
using System.IO;
using Newtonsoft.Json.Serialization;

using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Security.Cryptography;
using System.Net.Configuration;
#if WINDOWS_UWP
using Windows.Storage;
#endif

public static class Globals
{
    public const Int32 BUFFER_SIZE = 512; // Unmodifiable
    public static String obj_string;
    public static bool new_sent = false;
}

public class on_click_registration : MonoBehaviour
{
    public GameObject buttomn;
    Matrix4x4 transform1_matrix;
    bool first_time = true;
    void TcpListener()
    {
        TcpListener server = null;
        try
        {
            // Set the TcpListener on port 13000.
            Int32 port = 13000;
            IPAddress localAddr = IPAddress.Parse("127.0.0.1");
            // TcpListener server = new TcpListener(port);
            server = new TcpListener(localAddr, port);
            // Start listening for client requests.
            Debug.Log("start listen...");
            server.Start();
            // Buffer for reading data
            // Enter the listening loop.
            Debug.Log("Waiting for a connection... ");
            // Perform a blocking call to accept requests.
            // You could also use server.AcceptSocket() here.
            TcpClient client = server.AcceptTcpClient();
            Byte[] bytes_curr = new Byte[client.ReceiveBufferSize];
            Debug.Log("Connected!");
            // Get a stream object for reading and writing
            NetworkStream stream = client.GetStream();
            int i;
            MemoryStream memory_stream = new MemoryStream();
            // Loop to receive all the data sent by the client.
            while ((i = stream.Read(bytes_curr, 0, bytes_curr.Length)) != 0)
            {
                string msg = Encoding.UTF8.GetString(bytes_curr);
                byte[] data = System.Text.Encoding.UTF8.GetBytes(msg);
                Debug.Log("recived " + i);
                memory_stream.Write(data, 0, i);
            }
            // Shutdown and end connection
            Debug.Log("close connection");
            client.Close();
            // Stop listening for new clients.
            server.Stop();
            Globals.obj_string = System.Text.Encoding.UTF8.GetString(memory_stream.ToArray());
        }
        catch (SocketException e)
        {
            Debug.Log("SocketException:" + e.ToString());
        }
        finally
        {
            Globals.new_sent = true;
        }
    }
    // Start is called before the first frame update
    void Start()
    {
  
        transform1_matrix.m00 = 0.98898529f;
        transform1_matrix.m01 = -0.11914511f;
        transform1_matrix.m02 =  0.08782101f;
        transform1_matrix.m03 = -0.15361886f;

        transform1_matrix.m10 = 0.08349251f;
        transform1_matrix.m11 = 0.93898689f;
        transform1_matrix.m12 =  0.33366544f;
        transform1_matrix.m13 = -0.40734829f;

        transform1_matrix.m20 =  -0.12221738f;
        transform1_matrix.m21 =  -0.32265782f;
        transform1_matrix.m22 =  0.93859195f;
        transform1_matrix.m23 =  0.13752421f;

        transform1_matrix.m30 = 0;
        transform1_matrix.m31 = 0;
        transform1_matrix.m32 = 0;
        transform1_matrix.m33 = 1;
    }
    
    int cnt_pressed = 0;
    // Update is called once per frame
    void Update()
    {
        if (Globals.new_sent == true)
        {
            Globals.new_sent = false;
            Debug.Log("sent is true");
            var ct_parent_obj = GameObject.Find("CT_Scan");
            try
            {
               
                MemoryStream ms = new MemoryStream(Encoding.UTF8.GetBytes(Globals.obj_string));
                var loadedObj = new OBJLoader().Load(ms);
                loadedObj.name = "after_reg_mesh";
                loadedObj.transform.parent = ct_parent_obj.transform;
            }
            catch
            {
                Debug.Log("exception on OBJLoader");
            }
            Thread thread = new Thread(TcpListener);
            thread.IsBackground = true;
            thread.Start();

        }
        if (!buttomn.transform.hasChanged)
        {
            Debug.Log("was pressed " + cnt_pressed);
            cnt_pressed++;
            var try_to_move_obj = GameObject.Find("try_to_move");
            buttomn.transform.hasChanged = !buttomn.transform.hasChanged;
            var obj_old = GameObject.Find("after_reg_mesh");
            if (obj_old != null)
            {
                Destroy(obj_old);
                Debug.Log("was destroied " + cnt_pressed);
                ++cnt_pressed;
            }
            if (first_time == true)
            {
                first_time = false;
                Thread thread = new Thread(TcpListener);
                thread.IsBackground = true;
                thread.Start();
            }
        }
    }
}
