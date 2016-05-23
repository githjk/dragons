========================================================================
  CMDragons 2002 - RoboCup F180 League
========================================================================

2002 Team:
  Project Leader: Manuela Veloso   (mmv    @ cs.cmu.edu)
  Team Members:   James Bruce      (jbruce @ cs.cmu.edu)
                  Michael Bowling  (mhb    @ cs.cmu.edu)
                  Brett Browning   (brettb @ cs.cmu.edu)
                  Dinesh Govindaraju
                  Cathy Chang
  Affiliation:  Computer Science Department
                Carnegie Mellon University
                5000 Forbes Avenue
                Pittsburgh, PA 15213 USA
                phone: (412) 268-2601

For information on our team, see our publication list:
  http://www-2.cs.cmu.edu/~coral/www-papers/class_rescat.html#SmallSize

The main paper for our 2002 system is here:
  James Bruce, Michael Bowling, Brett Browning, and Manuela
  Veloso. "Multi-Robot Team Response to a Multi-Robot Opponent
  Team". In Proceedings of ICRA'03, the 2003 IEEE International
  Conference on Robotics and Automation, Taiwan, May 2003. A previous
  version also submitted to IROS-2002 workshop on Collaborative Robots

Note that this is not so much documentation as it is information about
our design.  All the documentation that we do have is included here,
don't ask us for more because we don't have it.  If you'd like to help
with the documentation however, we'll gladly accept submissions and
include them in a later release.

========================================================================
  Build Instructions
------------------------------------------------------------------------
To make the programs simply go to the main directory (ie ./dragons
after untarring it) and type make. This will make each of the programs
in each relevant subtree. If you have a framegrabber device with
video4linux (determined if the videodev module is loaded) then the
reality software will compile otherwise it will not. Once compiled,
your ready to run the programs

========================================================================
  Running the System
------------------------------------------------------------------------
Two environment variables need to be set to run the system. F180CONFIG
needs to be set to the path pointing to the config files located in
the config directory ie.: <mypath>/dragons/config. A second
environment variable is required to run the vision server. F180VISION
must point to the vision config directory (which can be the same as
F180CONFIG). help.txt in the ./config directory contains help
information for calibrating vision.

To run the simulator, go to the bin directory and execute
./simulator. Run an instance of soccer with ./soccer (usually in
another window) and a GUI with ./gui. If successful, you should see
the field on the GUI with an orange ball in a random location. Click
and drag the ball with the mouse to move it around.

You now need to select some robots.  Click on the blue and yellow
icon. Select what robots you want and hit set. If successful, the
robots should appear on the screen. Click on the referee to get a
simulated referee box, then click start to watch one of the teams
go. You can also run a second soccer instance connecting to the
simulator to run the other team (Note you will need to make use of the
command line switches to choose which team etc.) Blue robots, kicking
to the left goal is the default.

========================================================================
  Code Overview
------------------------------------------------------------------------
Our software consists of a series of programs that communicate via UDP
sockets and occasionally TCP sockets. The main software packages are
the GUI, rserver, simulator, soccer, and logging. The GUI is of course
the main graphical interface for all the programs. It communicates via
TCP to soccer, and UDP to rserver. rserver is the radio/vision server
that is used when running the real robots. It uses a UDP socket for
vision (input is config information, output is the standard frame
packet) and a UDP socket for radio (input is the command packet to
send to a robot, and commands such as pause). Simulator is a 2D
simulation that simulates some of the dynamics of the environment and
provides an identical interface as the rserver. Thus from the point of
view of the soccer code or the GUI it is indistinguishable which one
is running. We are currently developing a 3D simulator built on a real
physics simulation engine which will be released shortly.

Soccer is the main 'AI' program. It consists of tactics, the
navigation and motion control routines, and the playbook strategy
engine. For details on the algorithms we have used in the soccer
program please visit our publications. Logging utilities are logplay
and logrecord. These programs record packets from rvision or simulator
and store these in a file. These can then be played back, through the
GUI, using logplay. The data is stored in the file in a raw binary
format, but can be extracted using the log2text program. All programs
have a number of command line switches, and use configuration files
stored in ./config. What command line options are available can be
found by running the program with -h. Config files have a trivial
format and support comments. The software itself is structured into
directories as follows: General files

========================================================================
  General files
------------------------------------------------------------------------
Directory     Description
utils         Utility or shared code
bin           Final executables
help          Help files and documentation -- incomplete
logging       Logging code
include       Main include files
config        Contains config files
config/plays  Contains all the plays (.ply) and playbooks (.plb files)

========================================================================
  Servers / Main Programs
------------------------------------------------------------------------
reality/client    client class for easy socket interface to servers
reality/cmvision  CMVision library which is available as a seperate
                  release
reality/cmvedito  Editor to support CMVision calibration
reality/radio     class to support interface to radio via serial
reality/vision    high-level vision software
reality/server    Vision server program that runs socket interfac
simulator         2D simulator program
soccer            Main 'AI' program - navigation, control, tactics,
                  playbook strategy
gui               GUI program for monitoring/tasking system

========================================================================

We hope you enjoy poking at our code. - JRB
