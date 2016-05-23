// Main loop of the entire soccer system.
//
// Created by:  Michael Bowling (mhb@cs.cmu.edu)
//
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

#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <getopt.h>

#include <deque>

#include <configreader.h>

#include "socket.h"
#include "reality/net_vision.h"
#include "reality/net_radio.h"
#include "reality/client/client.h"
#include "soccer/net_gui.h"

#include "include/commands.h"

#include "world.h"
#include "tactic.h"
#include "play.h"
#include "simple_tactics.h"

#include "strategy.h"

#include "soccer.h"

#define DEBUG

const bool print_frames_missing = true;

/******************************* GLOBALS ****************************/

#define MAX_HOST   256

const bool robot_test = false;

// Network connections
//Socket vision_s(NET_VISION_PROTOCOL, NET_VISION_ACK_PERIOD);
//Socket radio_s(NET_RADIO_PROTOCOL, NET_RADIO_ACK_PERIOD);

Client client;
Socket gui_s(NET_GUI_PROTOCOL, NET_GUI_ACK_PERIOD);

char vision_hostname[MAX_HOST] = "localhost";

// World Model
World world;

// Strategy
Strategy strategy;

// Gui Tactics.  These override strategy's tactics.
Tactic *gui_tactics[MAX_TEAM_ROBOTS] = { NULL };

// Gui Messages
struct gmsg {
  char m[net_gui_out_maxsize];
  int size;
};

deque<gmsg> gmsg_queue;
bool run;

/******************************* PROTOTYPES *************************/

// Static Function Declarations
static void do_vision_recv();
static void do_gui_recv();
static void do_gui_send();

// Forward Declarations
void radio_send(const char robot, double vx, double vy, double va,
		bool kicker_on, bool dribbler_on);


//==== Functions =====================================================//

void handle_stop(int i)
// Signal to handle pressing Ctrl-C
{
  fprintf(stderr,"Caught break.\n");
  alarm(1);
  run = false;
}

void handle_alarm(int i)
// Signal to handle when the socket code hangs on exit
{
  fprintf(stderr,"Forced exit.\n");
  exit(0);
}


//==== Main Program ==================================================//

int main(int argc, char *argv[])
{
  Robot::RobotCommand rcmd;
  char side = SIDE_LEFT;
  char team = TEAM_BLUE;
  char *playbook_file = NULL;
  char *tactic_string = NULL;

  double t;

  // process the command line
  char c;
  while ((c = getopt(argc, argv, "ht:s:V:P:T:")) != EOF) {
    switch (c) {
    case 'V': strcpy(vision_hostname, optarg); break;
    case 't':
      if (strcmp(optarg, "yellow") == 0)
	team = TEAM_YELLOW;
      break;
    case 's':
      if (strcmp(optarg, "right") == 0)
	side = SIDE_RIGHT;
      break;
    case 'P':
      playbook_file = optarg;
      break;
    case 'T':
      tactic_string = optarg;
      break;
    case 'h':
    default:
      fprintf(stderr, "\nCMDragons 02 Soccer program\n");
      fprintf(stderr, "(c) Carnegie Mellon University, 2002\n");
      fprintf(stderr, "\nUSAGE\n");
      fprintf(stderr, "soccer -[htsGV]\n");
      fprintf(stderr, "\t-h\t\tthis help message\n");
      fprintf(stderr, "\t-t <blue | yellow>\tteam color\n");
      fprintf(stderr, "\t-s <left | right>\tdefence goal side\n");
      fprintf(stderr, "\t-T \"<tactic-string>\"\n");
      fprintf(stderr, "-V <host>\tWhat rserver/simulator host we should contact\n");
      return (1);
    }
  }

  // show what options were selected
  fprintf(stderr, "Options:\n");
  fprintf(stderr, "side %s, team %s:\n", ((side == SIDE_LEFT) ? "left" : "right"),
	  ((team == TEAM_BLUE) ? "blue" : "yellow"));
  fprintf(stderr, "Connecting to Vision %s\n", vision_hostname);

  // Open network connections
  //
  // Must connect to Vision and Radio servers.
  // Opens server port for gui connections.

  fprintf(stderr, "Connecting to Vision/Radio Sockets...\n");
  if (!client.Initialize(vision_hostname)) {
    fprintf(stderr, "Cannot connect to vision at %s:%d\n", 
	    vision_hostname, NET_VISION_PORT);
    exit(1);
  }

  /*
  if (client.vision_s.connect_client(vision_hostname, NET_VISION_PORT) != 
      Socket::Client) {
    fprintf(stderr, "Cannot connect to vision at %s:%d\n", 
	    vision_hostname, NET_VISION_PORT);
    exit(1);
  }

  fprintf(stderr, "Connecting to Radio...\n");
  if (client.radio_s.connect_client(vision_hostname, NET_RADIO_PORT) != Socket::Client) {
    fprintf(stderr, "Cannot connect to radio at %s:%d\n", 
	    vision_hostname, NET_RADIO_PORT);
    exit(1);
  }
  */

  fprintf(stderr, "Connecting to GUI...\n");
  if (gui_s.connect_server(NET_GUI_PORT) != Socket::Server) {
    fprintf(stderr, "Cannot setup gui server on port %d\n", NET_GUI_PORT);
    fprintf(stderr, "Continuing anyway.\n");
  }

  Tactic *tactics[MAX_TEAM_ROBOTS];
  for (int i=0; i<MAX_TEAM_ROBOTS; i++)
    tactics[i] = NULL;

  // initialize world model
  world.init(side, team);

  // initialize strategy
  strategy.init(playbook_file);
  
  if(robot_test){
    mzero(rcmd);
    rcmd.cmd = Robot::CmdPosition;
    rcmd.target.set(0,0); // FIELD_LENGTH_H,0);
    rcmd.obs = OBS_EVERYTHING;
  }

  // connect the stop signals
  signal(SIGINT,handle_stop);
  signal(SIGALRM,handle_alarm);

  // main loop
  run = true;

  while(run) {
    // Read an incoming vision updates.
    do_vision_recv();

    // initialize tactics from command line
    if (tactic_string) {
      for(int i=0; i<world.n_teammates; i++)
	gui_tactics[i] = Tactic::parse(tactic_string);
      tactic_string = NULL; 
    }

    if (robot_test) {
      for(int i=0; i<world.n_teammates; i++) {
	t = 1.0*world.time + 0.60*i;
	// t = 1.1*world.time+(2*M_PI/5)*i;
	rcmd.target.set(900*cos(t),600*sin(2*t));
	world.robot[i]->run(world, rcmd);
      } 

      client.Send();
    } else {

      // Strategy sets the player's tactics.
      strategy.run(world, tactics);
      
      // Do tactics.
      for(int i=0; i<world.n_teammates; i++) {
	if (gui_tactics[i]) {
	  gui_debug_printf(i, GDBG_TACTICS, "Tactic: %s\n", 
	  		   gui_tactics[i]->name());
	  gui_tactics[i]->run(world, i);
	} else if (tactics[i]) {
	  gui_debug_printf(i, GDBG_TACTICS, "Tactic: %s\n", 
			   tactics[i]->name());
	  tactics[i]->run(world, i);
	} 
      }
      
      // Send all the radio commands.
      client.Send();
      
      // Strategy may need some more intensive thinking time.  This
      // needs to be called every frame since otherwise things like
      // goals might get missed.  Some parts of this, though, may not
      // need run at frame rate.
      strategy.think(world);

    }

    // Handle gui connections.
    while(gui_s.ready_for_accept()) gui_s.accept();
    
    do_gui_recv();
    do_gui_send();
  }

  // Stop all the robots.
  for(int i=0; i<world.n_teammates; i++)
    world.go(i, 0, 0, 0);
  client.Send();
}

// Global Functions
void gui_debug(const net_gdebug &g)
{
  gmsg a;

  a.size = g.size();
  memcpy(a.m, &g, a.size); 

  gmsg_queue.push_back(a);
}

void gui_debug_line(const char robot, const char level, 
		    vector2d p1, vector2d p2, char flags = 0)
{
  net_gdebug d = { NET_GUI_DEBUG_LINE, world.color, robot, level, world.time};

  d.info.line.p[0].set(world.from_world(p1));
  d.info.line.p[1].set(world.from_world(p2));
  d.info.line.flags = flags;

  gui_debug(d);
}

void gui_debug_x(const char robot, const char level, vector2d p)
{
  gui_debug_line(robot, level, p + vector2d(20, 20), p + vector2d(-20, -20));
  gui_debug_line(robot, level, p + vector2d(-20, 20), p + vector2d(20, -20));
}

void gui_debug_arc(const char robot, const char level,
		   vector2d p1, vector2d dimens,
		   double start_angle, double stop_angle, 
		   char flags = 0)
{
  net_gdebug d = { NET_GUI_DEBUG_ARC, world.color, robot, level, world.time};

  d.info.arc.center.set(world.from_world(p1));
  d.info.arc.dimens.set(dimens);
  d.info.arc.a1 = world.from_world_dir(start_angle);
  d.info.arc.a2 = world.from_world_dir(stop_angle);
  d.info.arc.flags = flags;

  gui_debug(d);
}

void gui_debug_printf(const char robot, const char level,
		      const char *fmt, ...)
{
  net_gdebug d = { NET_GUI_DEBUG_MSG, world.color, robot, level, world.time};
  va_list ap;
  va_start(ap, fmt);
  
  vsprintf(d.info.msg.msg, fmt, ap);

  gui_debug(d);
}

void radio_send(const char robot, double vx, double vy, double va,
		bool kicker_on, bool dribbler_on)
{
  client.SetCommand(world.time, world.color, 
		    robot, vx, vy, va, kicker_on, dribbler_on);
}

void radio_halt(const char robot)
{
  client.SetHalt(world.time, world.color, robot);
}

// Static Functions

static void do_vision_recv()
{
  static int times_called = 0;
  static int frames_processed = 0;

  times_called++;

  int rv;

  do {
    static char msg[net_vision_out_maxsize];
    char msgtype;

    rv = client.vision_s.recv_type(msg, net_vision_out_maxsize, msgtype);

    if(rv > 0){
      switch(msgtype) {
        case NET_VISION_FRAME:
          world.update(*((net_vframe *) msg));
	  frames_processed++;
          break;
      default:
        fprintf(stderr, "Unknown vision message type, %d!\n", msgtype);
      }
    }else{
      fprintf(stderr,"No server.\n");
      run = false;
    }
  } while(client.vision_s.ready_for_recv());

  if (print_frames_missing && frames_processed >= 150) {
    if(frames_processed < times_called){
      fprintf(stderr, "Frames Missed: %5g/sec.\n",
	      (frames_processed - times_called) / 5.0);
    }
    frames_processed = times_called = 0;
  }
}

static void do_gui_send()
{
  while(gui_s.ready_for_send() && !gmsg_queue.empty()) {
    gui_s.send(gmsg_queue.front().m, gmsg_queue.front().size);
    gmsg_queue.pop_front();
  }

  if (gui_s.get_status() != Socket::Server) gmsg_queue.clear();
}

static void do_gui_recv()
{
  int rv;

  while(gui_s.ready_for_recv()) {
    char msg[net_gui_in_maxsize];
    char c;

    errno = 0;
    rv = gui_s.recv_type(msg, net_gui_in_maxsize, c);

    // Whenever a gui connects will reread our config file.
    // Kind of a hack but it works.  
    if (rv == 0) {
      fprintf(stderr, "Rereading config files.\n");
      configreader.UpdateFiles();
      continue;
    }

    switch(c) {
    case NET_GUI_TACTIC: {
      net_gtactic *gt = (net_gtactic *) msg;

#ifdef DEBUG
      fprintf(stderr, "Recieving from gui: rid %i, tactic %s\n", gt->robot, gt->string);
#endif

      if (gt->robot < 0) {
        strategy.parse(gt->string);
	      // This could be used for strategy.o
      } else {
	if (gui_tactics[gt->robot]) delete gui_tactics[gt->robot];
	gui_tactics[gt->robot] = Tactic::parse(gt->string);
      }
    }
    }
  }
}
