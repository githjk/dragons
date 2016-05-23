/*
 * TITLE:	Server.cc
 *
 * PURPOSE:	This is the main entry point for the vision/radio server program
 *              This file runs the main routine, which splits into two threads.
 *              the radio thread monitors the radio socket and spits data to
 *              the serial port as required. The vision thread captures frames,
 *              processes them and spits the results out hte vision socket.
 *
 * WRITTEN BY: James R Bruce, Brett Browning
 *
 * REVISION HISTORY:
 * Nov 7, 2001 - ported code from 2001 code - BB
 * Dec 10, 2001 - added radio port code and restructred - BB
 * Feb 18, 2002 - added refcode and restructured threads - BB
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

#include <sys/types.h>
#include <semaphore.h>
#include <signal.h>
#include <sys/poll.h>

#include <stdlib.h>
#include <stdio.h>

#include <pthread.h>

#include <getopt.h>
#include <ctype.h>

#include "rtypes.h"
#include "constants.h"
#include "geometry.h"
#include "util.h"
#include "timer.h"

// #include "utils/vector.h"
#include "utils/socket.h"

// vision stuff
#include "../cmvision/capture.h"
#include "../vision/camera.h"
#include "../vision/vision.h"
#include "../vision/detect.h"

// radio stuff
#include "../radio/robocomms.h"
#include "../radio/serial.h"

#include "reality/net_vision.h"
#include "reality/net_radio.h"

#include "../utils/vtracker.h"
#include "commands.h"


/**************************** TYPES ******************************************/

#define VISION_MAX_CLIENTS        8
#define VISION_CLIENT_TTL       120

#define NUM_CAMERAS 1

#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 240

#define PIXEL_FORMAT V4L2_PIX_FMT_UYVY


//==== Server Types ====//

 // 0 is missing
char *device_name[4] = {
  "/dev/video0",
  "/dev/video1",
  "/dev/video2",
  "/dev/video3"
};

// serial device for referee
char *ref_device_name = "/dev/ttyS1";

// file to save start/stop info in
const char *run_log_filename = "/var/log/robot/rserver.log";


struct camera_t {
  capture cap;
  camera model;
  pthread_t thread;
};


// macro to work on vision flags -- need work for confidence
#define VFLAGS(present, orient)      (((present) ? V_LOC_ENABLED : 0) |  \
				      ((orient) ? V_LOC_ORIENTATION : 0))

#ifdef PROFILE
#define THREAD_START setitimer(ITIMER_PROF,&itimer,NULL);
struct itimerval itimer;
#else
#define THREAD_START
#endif

/**************************** GLOBALS ****************************************/

const bool print_got_somethings = false;

camera_t camera[NUM_CAMERAS];
LowVision vision;
// class detect detect;
detect *vdetect;
vlocations loc;
VTracker tracker;
RoboComms rcomms;

// we need this for the radio daemon
int team_color = TEAM_BLUE;

sem_t vision_mutex;
bool run_daemon;
bool running;

char fname[256];
bool save_image;
int image_num;

net_vframe vframe;

Socket vision_s(NET_VISION_PROTOCOL, NET_VISION_ACK_PERIOD);
Socket radio_s(NET_RADIO_PROTOCOL, NET_RADIO_ACK_PERIOD);
serial referee;

// vision control
extern bool dump_vision_stats;
extern bool find_calib_patterns;

/***************************** PROTOTYPES ************************************/
void thread_start();
void VisionDaemon(camera_t *cam);
bool Initialize();
void Close();
void SaveThresholdImage();
void Reset();
void handle_stop(int i);
void handle_alarm(int i);

void run_log_entry(bool start);

void do_ref_recv(void);
void do_radio_recv(void);
void do_vision_recv(void);
void do_vision_send(void);


void do_tracking_update(void);

/***************************** CODE ******************************************/


/*
 * main -
 *
 * RETURN VALUE: 0 on success, error code on failure
 */
int main(int argc,char *argv[])
{
  pollfd pfd[2];
  // int n;
  bool sleeping;
  // net_vconfig vc;
  // pthread_t radio_thread;

  char c;

#ifdef PROFILE
  getitimer(ITIMER_PROF,&itimer);
#endif

  // process the command line
  while ((c = getopt(argc, argv, "csh")) != EOF) {
    switch (c) {
      case 'c':
	find_calib_patterns = true;
	break;
      case 's':
	dump_vision_stats = true;
	break;
      case 'h':
      default:
        fprintf(stderr, "\nUSAGE: rserver -[h]\n");
        fprintf(stderr, "\n-h\tthis message\n");
        return (0);
    }
  }

  // Init vision & detection system
  Initialize();

  // Open the vision socket
  printf("Opening vision socket as server...\n");
  if (vision_s.connect_server(NET_VISION_PORT) != Socket::Server) {
    fprintf(stderr, "Cannot create vision server on %d\n", NET_VISION_PORT);
    return(-1);
  }

  // Open the radio socket
  printf("Opening radio socket as server...\n");
  if (radio_s.connect_server(NET_RADIO_PORT) != Socket::Server) {
    fprintf(stderr, "Cannot create radio server on %d\n", NET_RADIO_PORT);
    return(-1);
  }

  // if we are running with a referee then open the referee connection
  printf("opening referee connection on device %s\n", ref_device_name);
  if (!referee.open(ref_device_name, COMM_BAUD_RATE, O_RDONLY|O_NONBLOCK)) {
    fprintf(stderr, "Cannot open referee serial port %s\n", ref_device_name);
    return (-1);
  }

  // Initialize ref state
  vframe.refstate = COMM_STOP;

  printf("==== Running ================\n");

  // Set up polling for radio commands and referee connection
  pfd[0].fd = radio_s.get_fd();
  pfd[0].events = POLLIN;
  pfd[1].fd = referee.get_fd();
  pfd[1].events = POLLIN;

  sleeping = false;
  running = true;

  // connect the stop signals
  signal(SIGINT,handle_stop);
  signal(SIGALRM,handle_alarm);

  // intiali;ze the robocomms module
  rcomms.init();

  // The main loop.  Handles the referee from serial, and radio
  // commands from over the network
  while(running){
    // if we are sleeping, wait for some input
    if(sleeping) poll(pfd,2,1000);

    // recieve radio commands
    do_radio_recv();

    // update ref command
    do_ref_recv();

    // run radio
    sleeping = !rcomms.run();
  }

  printf("closing down...");

  // close everything up
  vision_s.disconnect();
  radio_s.disconnect();
  rcomms.close();
  referee.close();
  Close();
  printf("done!");

  // all done
  return(0);
}


void do_ref_recv(void)
{
  char c;

  // try to read off a ref state
  if(referee.read(&c, 1) != 1) return;

  // check the status of the referee byte we read
  if (vframe.refstate == COMM_START) {
    if (c == COMM_STOP) {
      vframe.refstate = COMM_STOP;
      printf("Read %c\n", vframe.refstate);
    }
  } else {
    if (strchr("SG12OARKPFTZ", toupper(c)) != NULL) {
      vframe.refstate = c;
      printf("Read %c\n", vframe.refstate);
    }
  }
}

void do_radio_recv(void)
{
  // check to see if something is in the buffer
  if (!radio_s.ready_for_recv()) 
    return;

  if(print_got_somethings) fprintf(stderr, "got something\n");

  static char msg[net_radio_in_maxsize];
  char msgtype;
  static net_rcommands *rcmds = (net_rcommands *) msg;
  static net_rcontrol *rctl = (net_rcontrol *) msg;

  // get the message
  radio_s.recv_type(msg, net_radio_in_maxsize, msgtype);

  // find the time
  // BB - HACK - is this right? What time should be here???
  double tstamp = loc.timestamp;

  switch (msgtype) {
  case NET_RADIO_COMMANDS: {

    // printf("got command: received %i of them\n", rcmds->nr_commands);
    for (uint i = 0; i < rcmds->nr_commands; i++) {
      int rid = rcmds->cmds[i].id;
      int t = rcmds->cmds[i].team;

#ifdef DEBUG
      printf("\tradio command t %i r %i type %i -> %i, %i, %i, prior %i\n", 
	      t, rid, rcmds->cmds[i].type,
	      rcmds->cmds[i].dx, rcmds->cmds[i].dy, rcmds->cmds[i].da, 
	      rcmds->cmds[i].priority);
#endif

      // check for a valid array id
      if ((rid < 0) || (rid >= MAX_TEAM_ROBOTS)) {
	printf("bad id: was %i, command %i\n", rid, i);
	break;
      }

      // now check for a valid id
      int id = tracker.index2id[t][rid];
      if ((id < 0) || (id >= MAX_ROBOT_ID)) {
	printf("bad robot id2: array id %i, rid %i, cmd indx %i\n",
	       rid, id, i);
	break;
      }

      // need to fix up the id in the command packet before sending it
      // to the robocomms 
      rcmds->cmds[i].id = id;
	
      // inform the tracker of the latest command
      vector3d vcmd(rcmds->cmds[i].dx, rcmds->cmds[i].dy, 
		    (double) rcmds->cmds[i].da / 1000.0);
      tracker.robots[t][rid].command(tstamp, vcmd);
    }

    // now set all the commands in one hit
    rcomms.setCommands((net_rcommand *) rcmds->cmds, rcmds->nr_commands);

  } break;
  case NET_RADIO_CONTROL:
    rcomms.setSleep(rctl->control == VCR_PAUSE);
    break;
  }
}

void do_vision_recv(void)
{
  /* see if there is anything we need to read */
  if (!vision_s.ready_for_recv())
    return;

  if(print_got_somethings) fprintf(stderr, "got something\n");

  char msgtype;
  static char buffer[256];
  static net_vconfig *vc = (net_vconfig *) buffer;
  static net_vref *vr = (net_vref *) buffer;
  
  vision_s.recv_type(buffer, net_vision_in_maxsize, msgtype);
  
  // check if we got a config command
  switch (msgtype) {
  case NET_VISION_CONFIG:
    fprintf(stderr, "Enabling config\n");
    
    // update the detection system
    memcpy(&vframe.config, vc, sizeof(net_vconfig));
    vdetect->updateParams(vframe.config);
    
    // update the tracking system
    tracker.SetConfig(vframe.config);
    break;
  case NET_VISION_REF:

    // we are using the simrefbox so just save ref state
    vframe.refstate = vr->refstate;

    fprintf(stderr, "got a simref cmd %c\n", vframe.refstate);

    break;
  default:
    fprintf(stderr, "Unimplemented command %c\n", msgtype);
  }
}

void do_vision_send(void)
{
  /* check to see if there are new connections */
  while(vision_s.ready_for_accept()) 
    vision_s.accept();

  /* check if socket is okay */
  if (!vision_s.ready_for_send()) 
    return;

  // prediction time
  // HACK - BB - predtime should just be latency but this seems to help???
  //  double predtime = LATENCY_DELAY;
  double predtime = 0.0;

  /* make the vframe packet */
  vframe.msgtype         = NET_VISION_FRAME;
  vframe.timestamp       = loc.timestamp;

  // copy the data across
  vframe.ball.vision.timestamp = loc.ball.timestamp;
  vframe.ball.vision.pos = vdtof(loc.ball.cur.loc);
  vframe.ball.vision.angle = loc.ball.cur.angle;
  vframe.ball.vision.conf = loc.ball.conf;

  // fill out tracking info
  tracker.GetBallData(vframe.ball, predtime);

  /* we set up the blue team as the first data and the 
   * yellow team as the second always
   */
  for (int t = 0; t < NUM_TEAMS; t++) {
    for (int i = 0; i < MAX_TEAM_ROBOTS; i++) {
      if (vframe.config.teams[t].robots[i].id >= 0) {

	vframe.robots[t][i].vision.timestamp = loc.robot[t][i].timestamp;
	vframe.robots[t][i].vision.pos       = vdtof(loc.robot[t][i].cur.loc);
	vframe.robots[t][i].vision.angle     = loc.robot[t][i].cur.angle;
	vframe.robots[t][i].vision.conf      = loc.robot[t][i].conf;
	
	// Get the state information from the Kalman filter
	tracker.GetRobotData(vframe.robots[t][i], t, i, predtime);
      } else
	vframe.robots[t][i].vision.conf = 0.0;
    }
  }

  /* work out the referee -- dumb for now */
  // need ot get this from radio server
	//  vframe.refstate = REF_GO;
 
  /* send it off */
  vision_s.send(&vframe, sizeof(vframe));
}

void VisionDaemon(camera_t *cam)
{
  int frame_index,field,ret;
  double timestamp;
  pixel *buf;
  image img;

  img.width  = IMAGE_WIDTH;
  img.height = IMAGE_HEIGHT;
  img.pitch  = IMAGE_WIDTH;

  THREAD_START;

  while (run_daemon){
    buf = (pixel*)cam->cap.captureFrame(frame_index,field);
    if (buf) {
      timestamp = cam->cap.getFrameTimeSec();
      // printf("%d %f\n",field,timestamp);

      // lock the CMVision class and process the frame
      sem_wait(&vision_mutex);
      img.buf = buf;
      vision.processFrame(img,field);

      // run detection and tracking
      vdetect->update(loc,vision,cam->model,timestamp);
      // detect.getLocations(loc);

      // do tracking update here
      do_tracking_update();

      // Send new information to clients
      do_vision_send();

      // any new information ?
      do_vision_recv();

      if (save_image){
	vision.saveThresholdImage(fname);
	save_image = false;
      }

      sem_post(&vision_mutex);
      cam->cap.releaseFrame((unsigned char*)buf,frame_index);
    }
  }

  ret = 0;
  pthread_exit(&ret);
}

/*
 * do_tracking_update -
 *
 * This function updates the tracker with the latest observations
 */
void do_tracking_update(void)
{
  vraw obs;

  /* we need to update the ball first */
  obs.pos = vdtof(loc.ball.cur.loc);
  obs.timestamp = loc.ball.timestamp;
  obs.angle = 0.0;
  obs.conf = loc.ball.conf;
  tracker.ball.observe(obs, loc.timestamp);

  /* now do all the robots we have -- if they are configured */
  for (int t = 0; t < NUM_TEAMS; t++) {
    for (int i = 0; i < MAX_TEAM_ROBOTS; i++) {
      if (vframe.config.teams[t].robots[i].id >= 0) {
	obs.pos = vdtof(loc.robot[t][i].cur.loc);
	obs.timestamp = loc.robot[t][i].timestamp;
	obs.angle = loc.robot[t][i].cur.angle;
	obs.conf = loc.robot[t][i].conf;
	tracker.robots[t][i].observe(obs, loc.timestamp);
      }
    }
  }
}

/*
 * Initialize -
 *
 * This function initializes the vision system, starts the threads and
 * gets things goign
 */
bool Initialize()
{
  char *configdir;
  char tmapf[256];
  int i;

  run_daemon = true;
  sem_init(&vision_mutex,0,1);

  printf("==== Initializing Vision ====\n");

  configdir = getenv("F180VISION");
  if(!configdir) configdir = ".";

  printf("  Vision Config Directory: %s\n",configdir);

  vdetect = new detect;
  if(!vdetect) exit(1);

  // initialize capture
  for(i=0; i<NUM_CAMERAS; i++){
    if(camera[i].cap.initialize(device_name[i],
         IMAGE_WIDTH,IMAGE_HEIGHT,PIXEL_FORMAT)){
      printf("  Initialized capture %d.\n",i+1);
    }else{
      printf("  ERROR: Could not initialize capture %d.\n",i+1);
      return(false);
    }
  }

  // load camera models
  for(i=0; i<NUM_CAMERAS; i++){
    sprintf(fname,"%s/camera%d.txt",configdir,i+1);
    if(camera[i].model.loadParam(fname)){
      printf("  Initialized camera model %d.\n",i+1);
      // sprintf(buf,"camera%d-out.txt",i+1);
      // camera[i].model.print(); // saveParam(buf);
    }else{
      printf("  ERROR: Could not initialize camera model %d.\n",i+1);
      return(false);
    }
  }

  // Init vision
  sprintf(fname,"%s/%s",configdir,"colors.txt");
  sprintf(tmapf,"%s/%s",configdir,"thresh.tmap");
  if(vision.initialize(fname,tmapf,IMAGE_WIDTH,IMAGE_HEIGHT)){
    printf("  CMVision initialized.\n");
  }else{
    printf("  ERROR: Could not initialize CMVision.\n");
    return(false);
  }

  // spawn update daemon thread(s)
  for(i=0; i<NUM_CAMERAS; i++){
    if(!pthread_create(&camera[i].thread,NULL,
          (pthread_start)VisionDaemon,(void*)&camera[i])){
      printf("  Started vision daemon %d.\n",i+1);
    }else{
      printf("  ERROR: Could not start vision daemon %d.\n",i+1);
      return(false);
    }
  }

  // initialize the vframe structure
  for (int t = 0; t < NUM_TEAMS; t++) {
    vframe.config.teams[t].cover_type = VCOVER_NONE;
    for (int i = 0; i < MAX_TEAM_ROBOTS; i++) {
      vframe.config.teams[t].robots[i].id = -1;
      vframe.config.teams[t].robots[i].type = ROBOT_TYPE_NONE;
    }
  }
  vdetect->updateParams(vframe.config);

  run_log_entry(true);

  return(true);
}

/*
 * Close -
 *
 * Pretty obvious. close it all down and deallocate semaphores
 */
void Close()
{
  int i;

  printf("==== Closing Vision ====\n");

  // Flag exit and join all threads
  run_daemon = false;
  for(i=0; i<NUM_CAMERAS; i++){
    pthread_join(camera[i].thread,NULL);
  }

  // close all capture classes
  for(i=0; i<NUM_CAMERAS; i++){
    camera[i].cap.close();
  }

  sem_destroy(&vision_mutex);

  // close CMVision
  vision.close();

  run_log_entry(false);
}

#define VALIDTEAM(x) (x==TEAM_BLUE || x==TEAM_YELLOW)


// currently unused
void SaveThresholdImage()
{
  image_num++;
  sprintf(fname,"cmap%03d.ppm",image_num);
  save_image = true;
}

// currently unused
void Reset()
{
  vdetect->reset();
}


/*
 * handle_stop -
 *
 * Signal to handle someone pressing Ctrl-C
 */
void handle_stop(int i)
{
  // fprintf(stderr,"caught break; stopping.\n");
  alarm(1);
  run_daemon = false;
  running = false;
}

void handle_alarm(int i)
// Signal to handle when the code hangs on exit
{
  // fprintf(stderr,"Forced exit.\n");
  exit(0);
}

char *date_str()
{
  char *month="Jan\0Feb\0Mar\0Apr\0May\0Jun\0Jul\0Aug\0Sep\0Oct\0Nov\0Dec";
  char *day  ="Sun\0Mon\0Tue\0Wed\0Thu\0Fri\0Sat";

  static char date[32];
  struct tm *tm;
  time_t t;

  t = time(NULL);
  tm = localtime(&t);
  snprintf(date,32,
	   "%04d-%s-%02d %s %02d:%02d:%02d",
	   1900+tm->tm_year,&month[tm->tm_mon*4],tm->tm_mday,
	   &day[tm->tm_wday*4],
	   tm->tm_hour,tm->tm_min,tm->tm_sec);

  return(date);
}

void run_log_entry(bool start)
{
  FILE *log = fopen(run_log_filename,"at");
  static time_t start_time;
  time_t t;

  if(!log) return;
  t = time(NULL);

  if(start){
    start_time = t;
    fprintf(log,"%s pid=%05d rserver started\n",date_str(),getpid());
  }else{
    fprintf(log,"%s pid=%05d rserver stopping (%dsec run time)\n",
	    date_str(),getpid(),(int)(t-start_time));
  }

  fclose(log);
}
