# Hololens2_proj
Face Registration HL2

1) UnityHL2RmStreamer as in https://github.com/cgsaxner/HoloLens2-Unity-ResearchModeStreamer (too big for git)
2)StreamRecorderConverter - python scripts for points cloud processing
3) STL_files - exmaple of obj CT scan face (too big for git)
4) py_streamer - python script for the streamer (socket between UnityHL2RmStreamer to the script)
5)  StreamRecordedOutput(too big for git) - output examples , 
6)  ply_files is 2 ply files from 2 diffrent frames. We have to try register between those.
7)  try_face is ply file example from pv.png and pgm file
8)  
We can try something like http://www.open3d.org/docs/0.9.0/tutorial/Basic/pointcloud.html


The ply files were created by the recorder, we need to do this on live stream (the recorder is in https://github.com/microsoft/HoloLens2ForCV)
