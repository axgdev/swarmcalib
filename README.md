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
2.  Go to paparazzi folder, type in console: sudo sh conf/system/ftdi.sh
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

#Explanation of scripts:
 * CalibrationV2.py: Class where all the calibration methods are contained
  we have tried to make all the calls to the ivyNode independent so that this
  calibration script can be used in other scenarios such as simulation.
 * runCalibration.py: This scripts starts the calibration process.
 * finkenPID.py: Definition of the PID controller, translated to python from
  lua from the simulation scripts (not in this project).
 * freeflight.py: A script very similar to CalibationV2 which only function
  is to start the copter and make it fly without any control other than altitude
  to check how good are the calibration parameters given as input.
 * PIDTuningWithPlot.py: script that shows a graph for both PID controllers to
  see how is the improvement of the variable controlled with respect to time.
 * IvyCalibrationNode.py: This script contains all the important functions for
  the calibration script to interact with Ivy and it gets the data from ROS.
 * ivyModules/IvyInterfaceTest.py: It is just a small script to test the
 communication to ivy and ROS.
 * ivyModules/ivyTurnOnLeds.py: as the name says, it only turns on the leds of
  the copter.
 * ivyModules/* : the rest of the files here are from ivy library, the most
 relevant if you want to try out something directly with Ivy is to use ivyProbe.py.
 With that you can add listeners to regular expressions and see incoming data
 or send data.

##Some other comments and observations during our work:
An additional dummy parameter was necessary to be added to the direction
messages for a very strange and obscure error that occurs when all the
parameters are float, I think the first one has to be an integer
#(Note to self: CHECK THIS)

Sometimes the copter initial flight does not work. For these cases, make
sure that before you start the calibration script in the GCS the state
is set to "Empty".
