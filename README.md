LIDAR-IMU OpenGL visualiser

This project consists of 3 Threads. Ethernet packet capture and point cloud creation are executed in the GUI thread. Opengl visualization jobs are executed on the 2nd Thread. Serial communication with Tiny is executed on the 3rd Thread.

Currently it is working on Windows(x86) environment and if it is necessary to execute on Linux distributions you should touch; UDP socket and serial library.

OpenGL GLUT library is freeglut and probably work on both Linux and Windows.

Currently only supports VELODYNE Pucklite VLP-16. You can change parser class to read another one.

Don't forget to update serial port name.!
