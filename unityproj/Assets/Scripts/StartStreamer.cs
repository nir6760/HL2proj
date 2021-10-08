
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
 
        Debug.Log("Windows Runtime Support enabled 0 ");
#if ENABLE_WINMD_SUPPORT
    Debug.Log("Windows Runtime Support enabled 3");
                    StartDll();
#endif
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
