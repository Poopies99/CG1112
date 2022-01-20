set size square 1,1     				#set the aspect ratio to 1:1
set object circle at 160,160 size 30  	#draw 3 concentric circle centered on 160,160
set object circle at 160,160 size 20 
set object circle at 160,160 size 10 
set label "25 cm" at 150,148 boxed		#label the 3 circles
set label "50 cm" at 150,138 boxed		#find out the actual distance
set label "75 cm" at 150,128 boxed
plot [200:120] [200:120] "lidar_reading.dat" using 1:2 with points pointtype 4
pause 10 
reread
