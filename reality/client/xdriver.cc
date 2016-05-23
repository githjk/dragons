/* LICENSE:
  =========================================================================
    CMDragons'02 RoboCup F180 Source Code Release
  -------------------------------------------------------------------------
    Copyright (C) 2002 Manuela Veloso, Brett Browning, Mike Bowling,
                       James Bruce; {mmv, brettb, mhb, jbruce}@cs.cmu.edu
    School of Computer Science, Carnegie Mellon University
  -------------------------------------------------------------------------
    This software is distributed under the GNU General Public License,
    version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  ------------------------------------------------------------------------- */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <getopt.h>

#include "utils/util.h"
#include "utils/vector.h"
#include "utils/geometry.h"

#include "colors.h"
#include "utils/win.h"
#include "utils/configreader.h"

#include "client.h"

typedef unsigned char uchar;

#define MAX_HOST    256

/*************************** PROTOTYPES ********************/
void xdrive(void);
void VisionThread(void);

/*************************** GLOBALS ***********************/
char vision_hostname[MAX_HOST] = "localhost";
Client client;
net_vframe frame;
int robotid = 0;
int arrayid = 0;
int team = TEAM_BLUE;

// config data

CR_DECLARE(XDRIVE_PRINT);
CR_DECLARE(XDRIVE_ACC);

CR_DECLARE(XDRIVE_VMAX);
CR_DECLARE(XDRIVE_RMAX);
CR_DECLARE(XDRIVE_RGAIN);

CR_DECLARE(XDRIVE_KICKTIME);
CR_DECLARE(XDRIVE_FRAMERATE);


int main(int argc,char **argv)
{
  pthread_t vthread;

  // process the command line
  char c;
  while ((c = getopt(argc, argv, "hV:r:t:")) != EOF) {
    switch (c) {
    case 'V': strcpy(vision_hostname, optarg); break;
    case 'r': robotid = atoi(optarg); break;
    case 't': 
      if (strcmp(optarg, "yellow") == 0)
	team = TEAM_YELLOW;
      break;
    case 'h':
    default:
      fprintf(stderr, "\nCMDragons 02 XDriver program\n");
      fprintf(stderr, "(c) Carnegie Mellon University, 2002\n");
      fprintf(stderr, "\nUSAGE\n");
      fprintf(stderr, "soccer -[hV]\n");
      fprintf(stderr, "-V <host>\tWhat rserver/simulator host we should contact\n");
      fprintf(stderr, "-r <robot id>\tWhat robot to drive (its real id - default 0)\n");
      fprintf(stderr, "-t <yellow | blue>\tWhat team to drive (default blue)\n");
      return (1);
    }
  }
  
  // initialize the client
  printf("Initializing the client\n");
  printf("\tConnecting to Vision/Radio host %s...\n", vision_hostname);
  if (!client.Initialize(vision_hostname)) {
    fprintf(stderr, "ERROR: Cannot connect to host %s\n", vision_hostname);
    exit(1);
  }

  // setup the data from the config file
  CR_SETUP(xclients, XDRIVE_PRINT, CR_INT);
  CR_SETUP(xclients, XDRIVE_ACC, CR_DOUBLE);
  CR_SETUP(xclients, XDRIVE_VMAX, CR_DOUBLE);
  CR_SETUP(xclients, XDRIVE_RMAX, CR_DOUBLE);
  CR_SETUP(xclients, XDRIVE_RGAIN, CR_DOUBLE);
  CR_SETUP(xclients, XDRIVE_KICKTIME, CR_INT);
  CR_SETUP(xclients, XDRIVE_FRAMERATE, CR_INT);

  // start the vision thread

  if(pthread_create(&vthread, NULL, (pthread_start) VisionThread, NULL)) {
    printf("ERROR: Cannot start vision thread\n");
    exit(1);
  }

  // wait for our robot id
  while (robotid < 0)
    ;

  // go do the grunt work
  printf("Driving robot %i, team %i...\n", team, robotid);
  printf("commands are - cursor key to drive, mouse to turn ...\n");
  printf("\tcursor: up/down for forward/backwards\n");
  printf("\tleft mouse to kick, right mouse to toggle dribbler\n");

  xdrive();
  client.Close();
  return (0);
}

void VisionThread(void)
{
  // vision thread just receives packets and sets the time
  while (client.IsConnected()) {
    client.GetUpdate(frame);

    // work out the robot id mapping
    int id = -1;
    for (uint i = 0; i < MAX_TEAM_ROBOTS; i++) {
      if (frame.config.teams[team].robots[i].id == robotid)
	id = i;
    }

    if ((arrayid >= 0) && (id < 0))
      printf("Cannot find id! Please enable robot %i team %i\n", robotid, team);
    arrayid = id;
  }
}


void xdrive(void)
{
  xwindows xw;
  xwin win;
  XEvent xev;
  KeySym key;
  vector2d pos_accel;

  bool run,grab;
  bool kick = false, dribble = false;
  int b;

  int x,y,dx,dy,n;
  int m;

  bool k_move_left;
  bool k_move_right;
  bool k_move_forward;
  bool k_move_backward;

  double vx,vy,va;
  //  vector vel;


  xw.initialize();
  xw.createWindow(win, 400,400,"XRoboDriver");
  win.setCursor(XC_crosshair,Rgb::Blue,Rgb::Black);
  srand48(time(NULL));

  run = true;
  grab = false;

  k_move_left     = false;
  k_move_right    = false;
  k_move_forward  = false;
  k_move_backward = false;

  vx = vy = va = 0.0;
  m = 0;

  pos_accel.set(DVAR(XDRIVE_ACC),DVAR(XDRIVE_ACC));
  pos_accel *= FRAME_PERIOD;

  while(run){
    if(grab) win.movePointer(200,200);
    dx = dy = n = 0;

    while(xw.checkEvent(xev)){
      // printf("e(%d)\n",xev.type);
      switch(xev.type){
        case KeyPress:
          key = XLookupKeysym(&xev.xkey,0);
          switch(key){
            case XK_space:
	      k_move_left     = false;
	      k_move_right    = false;
	      k_move_forward  = false;
	      k_move_backward = false;
	      break;

	    case(XK_G):
	    case(XK_g):
	      if(!grab){
		win.grabPointer();
	      }else{
		win.ungrabPointer();
	      }
	      grab = !grab;
	      break;

            case(XK_Left):  k_move_left     = true; break;
            case(XK_Right): k_move_right    = true; break;
            case(XK_Up):    k_move_forward  = true; break;
            case(XK_Down):  k_move_backward = true; break;

	    case(XK_bracketleft):  m = (m + 1) % 3; break;
 	    case(XK_bracketright): m = (m + 2) % 3; break;

            case(XK_Escape):
	      vx = vy = va = 0;
	      run = false;
              break;
          }
          break;

        case KeyRelease:
          key = XLookupKeysym(&xev.xkey,0);
          switch(key){
            case(XK_Left):  k_move_left     = false; break;
            case(XK_Right): k_move_right    = false; break;
            case(XK_Up):    k_move_forward  = false; break;
            case(XK_Down):  k_move_backward = false; break;
          }
          break;

        case MotionNotify:
	  x = xev.xbutton.x - 200;
	  y = 200 - xev.xbutton.y;
	  dx += x;
	  dy += y;
	  n++;
          break;

        case ButtonPress:
	  b = (xev.type == ButtonPress)?
	    1<<(xev.xbutton.button + 7) : xev.xmotion.state;
	  switch(b){
	    case Button1Mask:
	      printf("kicking!!!\n");
	      kick = true;
	      break;
	    case Button2Mask:
	      break;
	    case Button3Mask:
	      dribble = !dribble;
	      if (dribble)
		printf("dribbling!!!\n");
	      break;
	  }
          break;
        case Expose:
	  break;
      }
    }

    if (k_move_left    ) vy += pos_accel.y;
    if (k_move_right   ) vy -= pos_accel.y;
    if (k_move_forward ) vx += pos_accel.x;
    if (k_move_backward) vx -= pos_accel.x;

    if (!k_move_left && !k_move_right){
      if(vy < 0.0){
	vy = min(vy + pos_accel.y,0.0);
      }else{
	vy = max(vy - pos_accel.y,0.0);
      }
      vy = 0.0;
      va = 0.0;
    }

    if(!k_move_forward && !k_move_backward){
      if(vx < 0.0){
	vx = min(vx + pos_accel.x,0.0);
      }else{
	vx = max(vx - pos_accel.x,0.0);
      }
    }

    // rotation if we are grabbingteh mouse
    if(grab){
      va = -dx / DVAR(XDRIVE_RGAIN);
      // vx += dy / (0.1);
    }
    va += vy/6;

    vx = bound(vx,-DVAR(XDRIVE_VMAX),DVAR(XDRIVE_VMAX));
    vy = bound(vy,-DVAR(XDRIVE_VMAX),DVAR(XDRIVE_VMAX));
    va = bound(va,-DVAR(XDRIVE_RMAX),DVAR(XDRIVE_RMAX));

    // pack the command and send it
    if (arrayid >= 0) {
      client.SendCommand(frame.timestamp + LATENCY_DELAY, team, 
			 arrayid, vx, vy, va, kick, dribble, 2);

      if (IVAR(XDRIVE_PRINT)) {
	printf("sending cmd: t %f, team %i r %i: v (%f %f %f) k %d d %d prior 2\n",
	       frame.timestamp + LATENCY_DELAY, team, arrayid, vx, vy, va,
	       kick, dribble);
      }
      if (kick) {
	usleep((uint) IVAR(XDRIVE_KICKTIME));
	kick = false;
	// pack the command and send it
	client.SendCommand(frame.timestamp + LATENCY_DELAY, team, 
			   arrayid, vx, vy, va, kick, dribble, 2);
      }
    }
    usleep((uint) DVAR(XDRIVE_FRAMERATE));
  }

  win.close();
  xw.close();
}

