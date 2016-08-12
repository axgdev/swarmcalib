# swarmcalib
Swarm Calibration project

Project in python which has 3 parts:
-Communication with Ivy and ROS
-Communication to V-REP
-Calibration


20160812-Comments:

We can set internal and external zones with check values to see how the copter position has changed.

What do the calibration parameters look like in the arena: 
___________________________
|          +theta          |
|           |              |
|   +phi____|_____-phi     |
|           |              |
|           |              |
|   Green copter part      |
____________________________
      THE OBSERVER
      
Calibration parameters that worked good:
0.284
0.305

Good PID values:
P=0.175
I=0.000001
D=0
