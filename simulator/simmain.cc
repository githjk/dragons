/*
 * TITLE:	Simmain.cc
 *
 * PURPOSE:	This is the main entry point for the simulator program
 *              This file runs the main routine and performs all the interacitons
 *              with the GUI program by mimicing the socket connections of the
 *              vision and radio software.
 *
 * WRITTEN BY:  Scott Lenser, Michael Bowling, Brett Browning
 */
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

#include <stdio.h>

#include <math.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include <getopt.h>

#include "constants.h"
#include "simulator.h"
#include "../utils/socket.h"

#include "../reality/net_radio.h"
#include "../reality/net_vision.h"

#include "../utils/vtracker.h"

#include "commands.h"

//#define DEBUG


/**************************** TYPES ******************************************/

struct Observations {
  double timestamp;
  vraw ball;
  vraw robots[NUM_TEAMS][MAX_TEAM_ROBOTS];
};

/**************************** GLOBALS ****************************************/
static net_vframe vf = { NET_VISION_FRAME };
VTracker tracker;
Simulator simulator;
Socket vision_s(NET_VISION_PROTOCOL, NET_VISION_ACK_PERIOD);
Socket radio_s(NET_RADIO_PROTOCOL, NET_RADIO_ACK_PERIOD);



/***************************** PROTOTYPES ************************************/
static void do_radio_recv(void);
static bool do_vision_send(Observations &obs);
void do_vision_recv(void);
static void do_tracking_update(Observations &obs);
void make_ball_vraw(vraw &obs, double tstamp);
void make_robot_vraw(vraw &obs, int t, int id, double tstamp);


void make_observations(Observations &obs, double tstamp);
void add2que(Observations &obs);
bool get_from_que(Observations &obs, double time);
void flush_que(void);



/***************************** CODE ******************************************/

/*
 * main -
 *
 * This is the main entry point for the simulator program. It initialises
 * the simulator and starts the server connections that the GUI talks to
 *
 * the command line arguments are:
 *
 */
int main(int argc, char *argv[])
{
  char c;
  bool usenoise = false, goalreturn = true;

  // process the command line options
  while ((c = getopt(argc, argv, "hng")) != EOF) {
    switch (c) {
    case 'n': usenoise = true; break;
    case 'g': goalreturn = false; break;
    case 'h':
    default:
      fprintf(stderr, "CMDragons 02 simulator\n");
      fprintf(stderr, "(c) Carnegie Mellon University, 2002\n");
      fprintf(stderr, "USAGE: simulator -[hd]\n");
      fprintf(stderr, "\t-h\tprint this help list\n");
      fprintf(stderr, "\t-n\tgenerate noise\n");
      fprintf(stderr, "\t-g\tdisable goal return\n");
      return (1);
    }
  }
  
  /* initialise the simulator */
  printf("Initializing simulator...\n");
  if (!simulator.Initialize(goalreturn, usenoise)) {
    fprintf(stderr, "Error: Cannot initialize simulator...\n");
    exit(1);
  }
  if (goalreturn)
    printf("Using goal return option\n");
  if (usenoise)
    printf("Using noise model\n");
  printf("Running...\n\n");

  // intialize the config matrix
  for (int t = 0; t < NUM_TEAMS; t++) {
    for (int i = 0; i < MAX_TEAM_ROBOTS; i++)
      vf.config.teams[t].robots[i].id = -1;
  }

  /*
   * Open network connections. we have a vision and radio
   * server that the main soccer program will talk to.
   */
#ifdef DEBUG
  printf("    SIM: Opening vision socket as server\n");
#endif
  if (vision_s.connect_server(NET_VISION_PORT) != 
      Socket::Server) {
    fprintf(stderr, "Cannot create vision server on %d\n", NET_VISION_PORT);
    exit(1);
  }

#ifdef DEBUG
  printf("    SIM: Opening radio socket as server\n");
#endif
  if (radio_s.connect_server(NET_RADIO_PORT) != Socket::Server) {
    fprintf(stderr, "Cannot create radio port at %d\n", NET_RADIO_PORT);
    exit(1);
  }

  Observations obs;
  bool running =true;
  struct timeval current_time;

  gettimeofday(&current_time, NULL);
  double tstart = (current_time.tv_sec + current_time.tv_usec / 1000000.0);

  while (running) {

 
    /* wait for the next frame */
    simulator.wait_for_update();

    double timestamp = simulator.Time() + tstart;

    // update the latest observations to the observation 
    make_observations(obs, timestamp);

    /* update the Kalman filter */
    do_tracking_update(obs);

    // receive any vision if we need to
    do_vision_recv();

    // send off the latest vision packet if the frame is ready
    if (!do_vision_send(obs)) {
      fprintf(stderr, "Can't send vision\n");
      running = false;
    }

    // get any new radio commands
    do_radio_recv();
  }

  return (0);
}


static void do_tracking_update(Observations &obs)
{
  /* we need to update the ball first */
  tracker.ball.observe(obs.ball, obs.timestamp);

  /* now do all the robots we have */
  for (int t = 0; t < NUM_TEAMS; t++) {
    for (int i = 0; i < MAX_TEAM_ROBOTS; i++) {
      if (vf.config.teams[t].robots[i].id >= 0) 
	tracker.robots[t][i].observe(obs.robots[t][i], obs.timestamp);
    }
  }
}


static void do_radio_recv(void)
{
  while(radio_s.ready_for_recv()) {
    static char msg[net_radio_in_maxsize];
    char msgtype;
    net_rcommands *rcmds = (net_rcommands *) msg;
    net_rcontrol *rctl = (net_rcontrol *) msg;

    // get the message and process it
    radio_s.recv_type(msg, net_radio_in_maxsize, msgtype);
    switch (msgtype) {
    case NET_RADIO_COMMANDS: {

      //      fprintf(stderr, "got command: received %i of them\n", rcmds->nr_commands);

      for (uint i = 0; i < rcmds->nr_commands; i++) {
	int rid = rcmds->cmds[i].id;
	int t = rcmds->cmds[i].team;

#ifdef DEBUG
	fprintf(stderr, "\tradio command t %i r %i type %i -> %i, %i, %i, prior %i\n", 
		t, rid, rcmds->cmds[i].type,
		rcmds->cmds[i].dx, rcmds->cmds[i].dy, rcmds->cmds[i].da, 
		rcmds->cmds[i].priority);
#endif

	// check for a valid array id
	if ((rid < 0) || (rid >= MAX_TEAM_ROBOTS)) {
	  printf("bad id\n");
	  break;
	}

	// set the command -> index is used not real radio id
	vector3d vcmd(rcmds->cmds[i].dx, rcmds->cmds[i].dy, 
		      (double) rcmds->cmds[i].da / 1000.0);
	simulator.SetRobotCommand(t, rid, vcmd.x, vcmd.y, vcmd.z, 
				  rcmds->cmds[i].kick, rcmds->cmds[i].drib);
	tracker.robots[t][rid].command(simulator.Time(), vcmd);
      }
    } break;
    case NET_RADIO_CONTROL:
      switch (rctl->control) {
      case VCR_PAUSE:
	simulator.SetRunState(RUNSTATE_PAUSE);
	break;
      case VCR_PLAY:
	simulator.SetRunState(RUNSTATE_PLAY);
	break;
      case VCR_STEP:
	simulator.SetRunState(RUNSTATE_STEPFORWARD);
	break;
      case VCR_SLOW:
      case VCR_FFWD:
      case VCR_RWND:
	break;
      }
      break;
    default:
      fprintf(stderr, "Unknown radio message type, %d!\n", msgtype);
    }
  }
}

void do_vision_recv(void)
{
  /* see if there is anything we need to read */
  if (!vision_s.ready_for_recv())
    return;

  char msgtype;
  char buff[net_vision_in_maxsize];
  net_vsim *vsim;
  net_vconfig *vc;
  net_vref *vr;
  
  //    if (vision_s.recv_type(buff, net_vision_in_maxsize, msgtype) > 0) {
  vision_s.recv_type(buff, net_vision_in_maxsize, msgtype);

  switch (msgtype) {
  case NET_VISION_CONFIG:
    vc = (net_vconfig *) buff;

    // remove all the bots for now
    simulator.SetNumRobots(false, 0);
    simulator.SetNumRobots(true, 0);
    
      // enable robots here -- ignore cover type for now
    for (int t = 0; t < NUM_TEAMS; t++) {
      for (int i = 0; i < MAX_TEAM_ROBOTS; i++) {
	if (vc->teams[t].robots[i].id >= 0) {
	  simulator.AddRobot(t, vc->teams[t].robots[i].type);
	  simulator.SetRobotRandom(t, i);
	}
      }
    }
    simulator.SetBall(vector2d(0, 0), vector2d(0, 0));

    // update the tracker
    memcpy(&vf.config, vc, sizeof(net_vconfig));
    tracker.SetConfig(vf.config);
    break;
  case NET_VISION_REF:
    vr = (net_vref *) buff;

    // a little user output
    fprintf(stderr, "received netvref command state %c\n", vr->refstate);

    // save it for the next frame
    vf.refstate = vr->refstate;
    break;
  case NET_VISION_SIM:
    vsim = (net_vsim *) buff;
    switch (vsim->command) {
    case VSIM_MOVEBALL: {
      simulator.SetBall(vstructtod(vsim->info.moveball.pos), 
			vstructtod(vsim->info.moveball.vel));
      tracker.ball.reset();
      } break;
    case VSIM_MOVEROBOT: {
      int t = vsim->info.moverobot.team;
      int id = vsim->info.moverobot.array_id;

      simulator.SetRobot(t, id,
			 vstructtod(vsim->info.moverobot.pos), 
			 (double) vsim->info.moverobot.angle);
      tracker.robots[t][id].reset();
      } break;
    }

    break;
  }
}

/*
 * vision_send -
 *
 * this function constructs a vision frame network packet from
 * the simulator information and sends it off to the network socket
 *
 * RETURN VALUE: True on success, FALSE on error
 */
static bool do_vision_send(Observations &obs)
{

  /* check to see if there are new connections */
  while(vision_s.ready_for_accept()) 
    vision_s.accept();

  /* check if socket is okay */
  if (!vision_s.ready_for_send()) 
    return (false);

  // simulator must predict ahead using the tracker
  // the tracked positions/velocities.

  // why is this one frame period behind????
  double predtime = 0.0; //LATENCY_DELAY - FRAME_PERIOD;

  vf.timestamp = obs.timestamp;

  // fill out the raw vision infor and Tracking info
  vf.ball.vision = obs.ball;
  tracker.GetBallData(vf.ball, predtime);


  /* we set up the blue team as the first data and the 
   * yellow team as the second always
   */
  for (int t = 0; t < NUM_TEAMS; t++) {
    for (int i = 0; i < MAX_TEAM_ROBOTS; i++) {
      vf.robots[t][i].vision = obs.robots[t][i];
      tracker.GetRobotData(vf.robots[t][i], t, i, predtime);
    }
  }

  if ((simulator.IsGoal() != 0) && (vf.refstate == COMM_START))
    vf.refstate = COMM_STOP;

  /* work out the referee -- dumb for now */
  //  vf.refstate = REF_GO;
 
  /* send it off */
  vision_s.send(&vf, sizeof(vf));

  return (true);
}


void make_observations(Observations &obs, double tstamp)
{
  vector2d tmp;

  // take an epsilon value off to make sure comparisons work
  //  tstamp += LATENCY_DELAY - FRAME_PERIOD / 2.0;
  obs.timestamp = tstamp;

  // fill out the raw vision infor 
  obs.ball.timestamp  = tstamp;
  vector2d p = simulator.GetBallPosition();
  obs.ball.pos        = vdtof(p);
  obs.ball.angle      = 0.0;
  obs.ball.conf       = simulator.GetBallConfidence();

  for (int t = 0; t < NUM_TEAMS; t++) {
    for (int i = 0; i < MAX_TEAM_ROBOTS; i++) {
      if (i < simulator.GetNumRobots(t)) {
	obs.robots[t][i].timestamp = tstamp;
	tmp = simulator.GetRobotPosition(t, i);
	obs.robots[t][i].pos       = vdtof(tmp);
	obs.robots[t][i].angle     = simulator.GetRobotDirection(t, i);
	obs.robots[t][i].conf      = simulator.GetRobotConfidence(t, i);
      } else
	obs.robots[t][i].conf = 0.0;
    }
  }
}


