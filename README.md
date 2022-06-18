# HL2 RegistrationApp
**Face Registration Application for HoloLens 2.**
<br/>
Made with Unity (2020.3.13f1) and Python (3.8.3).
<br/>
## Project FlowChart
![Project FlowChart](/assets)
## Registration Block FlowChart
![Registration FlowChart](/assets)
### How to run?
Clone the repository to a local folder system. \
\
**Unity (2020.3.13f1)**
1. Download and install MRTK and other configurations for the Unity-HoloLens connection using:
https://circuitstream.com/blog/hololens-2-setup-guide/ 
2. Open the "ProjScene" scene in "unityproj\assets\scenes".
3. Build and deploy the Unity project to a HoloLens 2 using: \
https://docs.microsoft.com/en-us/windows/mixed-reality/develop/unity/build-and-deploy-to-hololens 
<br/>

**Python (3.8.3)** 
1. Create an virtual environment (we used conda) using the environment.yml file.
2. From the py folder Run the Python project using:
```
$ <python> py_streamer/hololens2_simpleclient.py

```
Note: \
Special credit to this awesome repository - This helped us extract the point cloud from the HoloLens sensors.
https://github.com/cgsaxner/HoloLens2-Unity-ResearchModeStreamer

