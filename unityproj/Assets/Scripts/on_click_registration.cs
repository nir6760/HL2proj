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
using PubSub;
using TMPro;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.UI;



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
        public enum ClientStatus
        {
            Inactive,
            Activating,
            Active,
            Deactivating
        }

        [SerializeField] private string host;
        [SerializeField] private string port;
        private Listener _listener;
        private ClientStatus _clientStatus = ClientStatus.Inactive;


        public GameObject buttomn;
        public GameObject ct_parent_obj;
        private string input_host;
        private string input_port;

    MemoryStream ms;



    // Start is called before the first frame update
    void Start()
        {

        
#if !UNITY_EDITOR
        //Debug.Log("Not Unity Editor, UWP");
#else
        //Debug.Log("Unity Editor");
            /*thread_1 = new Thread(ZMQClient);
            thread_1.IsBackground = true;
            thread_1.Start();*/
#endif


        }


#if !UNITY_EDITOR
    
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
                    int i;
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
    void ZMQClient()
    {
        _listener = new Listener(host, port, HandleMessage);
        OnStartClient();
        _clientStatus = ClientStatus.Active;
    }

    private void HandleMessage(string message)
    {

        
        Debug.Log("At Handle message! ");
        Globals.obj_string = message;
        Globals.new_sent = true;

    }

    private void OnStartClient()
    {
        //Debug.Log("Starting client...");
        _clientStatus = ClientStatus.Activating;
        _listener.Start();
        Debug.Log("Client started!");
    }

    int cnt_pressed = 0;
        // Update is called once per frame
        void Update()
        {
        
        if (_clientStatus == ClientStatus.Active)
            _listener.DigestMessage();

        

        }
    public void OnRegistrationClick()
    {
        Debug.Log("Registration Click!");
        if (Globals.new_sent == true)
        {




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
                    loadedObj.AddComponent<SphereCollider>();
                    loadedObj.AddComponent<NearInteractionGrabbable>();
                    loadedObj.AddComponent<ObjectManipulator>();
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
            }
        }
    }

        public void ReadStringInputUI(string s)
        {
            input_host = s;
            host = input_host;
            
        IPAddress ip;
        bool ValidateIP = IPAddress.TryParse(input_host, out ip);
        if (ValidateIP)
        {
            Debug.Log("Valid IP: " + input_host);
            //GameObject.Find("Canvas").transform.localScale = new Vector3(0, 0, 0);
            GameObject canvas_obj = GameObject.Find("Canvas");
            canvas_obj.SetActive(!canvas_obj.active);
            ZMQClient();
        }
        else
        {
            TextMeshProUGUI textmeshPro = GameObject.Find("Text (TMP)").GetComponent<TextMeshProUGUI>();
            textmeshPro.SetText("Invalid IP, please re-enter");
        }
            
            

        }
        public void ReadStringInputTMP(string s)
        {
            input_port = s;
            Debug.Log(s);
            Debug.Log(input_port);

        }
}
