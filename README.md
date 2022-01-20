# Alex
Alex is a Search and Rescue (SAR) robot built using Raspberry Pi, Arduino, Lidar, with a TLS client-server setup, using C/C++. The goal is to navigate an unseen room solely using commands sent via a laptop, mapping out the room environment via information sent from the robot. [The full report can be found here.](https://github.com/hughjazzman/alex-4-1-2/blob/master/CG1112%20Final%20Report%204-1-2.pdf)

Click the image below for a short video on our project!

[![Project Video](alex.jpg)](https://www.youtube.com/watch?v=A9tuBDnDaaU)

## Get Started
1. Build Alex according to `CG1112 Final Report 4-1-2.pdf`
2. (Pi, Arduino) Compile and Upload `Arduino/Alex/Alex.ino` to Arduino
3. (Pi) Compile and Run `Pi/tls-server-lib/tls-alex-server.cpp`
4. (Laptop) Compile and Run `Laptop/tls-client-lib/tls-alex-client.cpp`
5. (Pi, LiDAR) Compile and Run `LiDAR/w7s2.cpp`
6. (Pi, LiDAR) Run `gnuplot` on `LiDAR/lidar_plot_live.plt`
5. Remotely control Alex using commands

