
using System;
using System.Runtime.InteropServices;
using UnityEngine;


public class StartStreamer : MonoBehaviour
{
    

#if ENABLE_WINMD_SUPPORT
        
        [DllImport("HL2RmStreamUnityPlugin", EntryPoint = "StartStreaming", CallingConvention = CallingConvention.StdCall)]
        public static extern void StartDll();
#endif


    // Start is called before the first frame update
    void Start()
    {
#if !UNITY_EDITOR
        //Debug.Log("Not unity editor");
#else
    //Debug.Log("Windows Runtime Support enabled 0 ");
#endif
#if WINDOWS_UWP
        //Debug.Log("UWP app");

#else
    //Debug.Log("Not UWP app ");
#endif
#if ENABLE_WINMD_SUPPORT
        //Debug.Log("Again not unity editor");
        StartDll();
#else
    //Debug.Log("Windows Runtime Support enabled 0 ");
#endif
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
