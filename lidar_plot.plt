set size square 1,1     				#set the aspect ratio to 1:1
plot [120:200] [120:200] "lidar_reading.dat" using 1:2 with points pointtype 4
