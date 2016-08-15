#swarmcalib
Swarm Calibration project

#Paparazzi, Ground station side: 
###(Parallel task to Camera side)
1.  Go into the git "calib" branch of paparazzi:
     Changes of the paparazzi branch include 
        - Killing copter through Ivy without remote control overriding it.
        - Turning front copter leds to green and back to red.
        - Control of copter pitch, roll, yaw remotely. This is added to
          remote control, mening control with both is possible.
2.  Go to paparazzi folder, type in console: su sh conf/system/ftdi.sh 
    This needs a password. Get the password from someone in swarmlab.
3.  Run paparazzi, type in console: ./paparazzi
4.  Note the aircraftID in the upper left side of the paparazzi window,
    this is an important input for the calibration python script.
5.  Click on build
6.  Put battery in copter, use gray cable to connect to copter, then 
    click on upload to flash the copter with the software.
7.  After copter is flashed succesfully, take battery off from copter.
8.  Connect ground station to copter, wait for one light to be on.
9.  Connect the battery to the copter, now the ground station should show
    two lights on (if they blink its also ok)
10. Click on "execute" on paparazzi window upper right hand side, this
    will bring up the GCS window, which shows the state of the copter
    and several other parameters. 
    For calibration the states are important, such as: Empty, Start, 
    inAir and so on. You can see the calibration parameters in: 
    settings -> body2imu. These parameters will be automatically 
    modified by the calibration script.

#Camera side: 
###(Parallel task to Paparazzi)
Please note ROS needs to be installed, in the swarmlab desktop it is
already installed.

1.  You can either go to the calibration user in the computer or download
    clone the git repository "tracking"
2.  Run in console: roslaunch swarmlab_tracking track_cam.launch
3.  Run in console: roslaunch swarmlab_tracking viz.launch

Camera should be running at this point.

#Calibration script side: 
###(After Camera and Paparazzi running)
Please note ROS needs to be installed. In the finken laptop it is
installed under the catkin_workspace folder in the home folder. The
instruction step 1 applies to the laptop.

1.  Run in console: source $HOME/catkin_workspace/devel/setup.bash
2.  Run in console: 
ROS_MASTER_URI="http://finken-brain:11311" python "$HOME/calib/swarmcalib/calibrationV2.py"

That's it! If the camera is working correctly, the calibration should
be getting data, and trying to correct the copter position and finding
calibration parameters

##Some other comments and observations during our work:
An additional dummy parameter was necessary to be added to the direction
messages for a very strange and obscure error that occurs when all the
parameters are float, I think the first one has to be an integer 
#(Note to self: CHECK THIS)

Sometimes the copter initial flight does not work. For these cases, make
sure that before you start the calibration script in the GCS the state
is set to "Empty".
