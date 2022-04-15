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
using System.Threading.Tasks;

#if !UNITY_EDITOR
using Windows.Networking;
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
#endif

public static class Globals
{
    public const Int32 BUFFER_SIZE = 512; // Unmodifiable
    public static String obj_string = "#";
    public static String try_string = "try_msg";
    public static String all_recived = "string was not received";
    public static bool new_sent = false;
    public static uint g_msg0;
}

public class on_click_registration : MonoBehaviour
{
#if !UNITY_EDITOR
    StreamSocket socket;
    StreamSocketListener listener;
    String port;
    Thread thread_0;
#else
      Thread thread_1;
#endif


    public GameObject buttomn;
    public GameObject ct_parent_obj;
    Matrix4x4 transform1_matrix;

    MemoryStream ms;
    bool first_time = true;


    // Start is called before the first frame update
    void Start()
    {

        transform1_matrix.m00 = 0.98898529f;
        transform1_matrix.m01 = -0.11914511f;
        transform1_matrix.m02 = 0.08782101f;
        transform1_matrix.m03 = -0.15361886f;

        transform1_matrix.m10 = 0.08349251f;
        transform1_matrix.m11 = 0.93898689f;
        transform1_matrix.m12 = 0.33366544f;
        transform1_matrix.m13 = -0.40734829f;

        transform1_matrix.m20 = -0.12221738f;
        transform1_matrix.m21 = -0.32265782f;
        transform1_matrix.m22 = 0.93859195f;
        transform1_matrix.m23 = 0.13752421f;


        transform1_matrix.m30 = 0;
        transform1_matrix.m31 = 0;
        transform1_matrix.m32 = 0;
        transform1_matrix.m33 = 1;

#if !UNITY_EDITOR
        Debug.Log("Not Unity Editor, UWP");
        listener = new StreamSocketListener();
        port = "13002";
        listener.ConnectionReceived += Listener_ConnectionReceived;
        listener.Control.KeepAlive = false;

        thread_0 = new Thread(Listener_Start);
        thread_0.IsBackground = true;
        thread_0.Start();
        //Listener_Start();
#else
        Debug.Log("Unity Editor");
        thread_1 = new Thread(TcpListener);
        thread_1.IsBackground = true;
        thread_1.Start();
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
        uint max_buffer_size_for_file = 15; //1092323 file size,  1092323# Create
        try
        {
            while (true)
            {

                /* using (var dw = new DataWriter(args.Socket.OutputStream))
                 {
                     dw.WriteString("Hello There");
                     await dw.StoreAsync();
                     dw.DetachStream();
                 }*/
                using (var dr = new DataReader(args.Socket.InputStream))
                {
                    //// The encoding and byte order need to match the settings of the writer 
                    //// we previously used.
                    dr.UnicodeEncoding = Windows.Storage.Streams.UnicodeEncoding.Utf8;
                    dr.ByteOrder = Windows.Storage.Streams.ByteOrder.LittleEndian;

                    dr.InputStreamOptions = InputStreamOptions.Partial;
                    await dr.LoadAsync(1200000);
                    string msg0 = dr.ReadString(max_buffer_size_for_file);
                    Globals.try_string = String.Copy(msg0);
                    string file_size_str = msg0.Split('#')[0];

                    uint file_size = Convert.ToUInt32(file_size_str);
                    //uint file_size = 15;
                    Debug.Log(file_size);
                    Globals.g_msg0 = file_size;
                    uint updated_file_size = file_size - max_buffer_size_for_file;
                    string input_string;
                    Globals.obj_string = "#";
                    Globals.try_string = "try_msg";
                    input_string = dr.ReadString(updated_file_size);
                    Globals.obj_string += String.Copy(input_string);
                    Globals.all_recived = "string received";
                    Globals.new_sent = true;


                    //Debug.Log("received: " + input);
                    //Debug.Log("recived " + i);
                }
            }
        }
        catch (Exception e)
        {
            Debug.Log("disconnected!!!!!!!! " + e);
        }
        Debug.Log("all received, close connection");
        //System.Threading.Thread.Sleep(1000);
    }
#else
    void TcpListener()
    {
        TcpListener server = null;
        try
        {
            // Set the TcpListener on port 13000.
            Int32 port = 13001;
            IPAddress localAddr = IPAddress.Parse("127.0.0.1");
            server = new TcpListener(localAddr, port);
            // Start listening for client requests.
            Debug.Log("start listen...");
            server.Start();
            // Buffer for reading data
            // Enter the listening loop.
            while (true)
            {
                Debug.Log("Waiting for a connection... ");
                // Perform a blocking call to accept requests.
                // You could also use server.AcceptSocket() here.
                TcpClient client = server.AcceptTcpClient();
                int buffer_size = client.ReceiveBufferSize;
                Byte[] bytes_curr = new Byte[buffer_size];
                Debug.Log("Connected!");
                // Get a stream object for reading and writing
                NetworkStream stream = client.GetStream();
                int i,s;
                MemoryStream memory_stream = new MemoryStream();
                // Loop to receive all the data sent by the client.
                //s = stream.Read(bytes_curr, 0, buffer_size);
                //string msg0 = Encoding.UTF8.GetString(bytes_curr);
                //Debug.Log(msg0);
                //uint file_size = Convert.ToUInt32(msg0);
                //Debug.Log(file_size);
                while ((i = stream.Read(bytes_curr, 0, buffer_size)) != 0)
                {
                    string msg = Encoding.UTF8.GetString(bytes_curr);
                    byte[] data = System.Text.Encoding.UTF8.GetBytes(msg);
                    //Debug.Log("recived " + i);
                    memory_stream.Write(data, 0, i);
                }
                // Shutdown and end connection
                Debug.Log("all recived, close connection");
                client.Close();
                Globals.obj_string = System.Text.Encoding.UTF8.GetString(memory_stream.ToArray());
                Globals.new_sent = true;
                //System.Threading.Thread.Sleep(1000);
            }
        }
        catch (SocketException e)
        {
            Debug.Log("SocketException:" + e.ToString());
        }
        finally
        {
            // Stop listening for new clients.
            server.Stop();
        }
    }
#endif

    int cnt_pressed = 0;
    // Update is called once per frame
    void Update()
    {
        //if (Globals.new_sent == true)
        if (Globals.new_sent == true)
        {
            Debug.Log("all recived string is");
            Debug.Log(Globals.all_recived);
            Debug.Log("i string is");
            Debug.Log(Globals.try_string);
            Debug.Log("obj string size is " + (Globals.obj_string.Length).ToString());
            Debug.Log(Globals.obj_string);
            Debug.Log("obj string is");
            Debug.Log(Globals.obj_string);
            Debug.Log("msg size is " + (Globals.g_msg0).ToString());

            if (buttomn.transform.hasChanged == true)
            {
                buttomn.transform.hasChanged = false;
                //destroy old one
                foreach (Transform child in ct_parent_obj.transform)
                {
                    Destroy(child.gameObject);
                }
                ms = new MemoryStream(Encoding.UTF8.GetBytes(Globals.obj_string));

                cnt_pressed++;
                try
                {
                    GameObject loadedObj;

                    try
                    {
                        //loading new one
                        loadedObj = new OBJLoader().Load(ms);
                        //rename new
                        loadedObj.name = "after_reg_mesh";
                        loadedObj.transform.parent = ct_parent_obj.transform;
                        Globals.new_sent = false;

                    }
                    catch
                    {
                        Debug.Log("ms is fucked up");
                    }

                    Debug.Log("obj was loaded " + cnt_pressed);
                }
                catch
                {
                    Debug.Log("exception on OBJLoader");
#if !UNITY_EDITOR
                    thread_0.Abort();
#else
                    thread_1.Abort();
#endif
                }
                //var try_to_move_obj = GameObject.Find("try");
                //try_to_move_obj.transform.position = transform1_matrix.MultiplyPoint(try_to_move_obj.transform.position);
                first_time = false;
            }
        }

    }
}
