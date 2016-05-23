// logplay.cc
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
#include <getopt.h>
#include <sys/time.h>
#include <unistd.h>

#include <deque>

#include "../utils/socket.h"
#include "../reality/net_vision.h"
#include "../reality/net_radio.h"
#include "../soccer/net_gui.h"

#include "configreader.h"

#include "../include/commands.h"

// shift to config reader
#define FAST_RATE     5

struct DebugFrame {
  fpos_t vispos, socpos;
  deque<net_gdebug> messages;
  net_vframe frame;
};



/************************* PROTOTYPES ***********************************/
static void do_radio_recv(void);
static bool do_vision_send(net_vframe &vf);
static bool ReadFile(void);
static bool StepFrame(void);
static bool do_gui_send(deque<net_gdebug> &f);



/************************* GLOBALS ***********************************/

Socket vision_s(NET_VISION_PROTOCOL, NET_VISION_ACK_PERIOD);
Socket radio_s(NET_RADIO_PROTOCOL, NET_RADIO_ACK_PERIOD);
Socket gui_s(NET_GUI_PROTOCOL, NET_GUI_ACK_PERIOD);


FILE *logfile = NULL, *slogfile = NULL;

bool usesoccer = false;

enum RunMode {PAUSE = 0, STEP, STEP_BACK, SLOW, PLAY, PLAY_BACK, FFWD, RWND};
RunMode runmode = PAUSE;
long frames = 0;

deque<DebugFrame> debugframes;
uint currpos = 0;



/************************* CODE ***********************************/



int main(int argc, char *argv[])
{
  char *filename = NULL, *sfilename = NULL;
  char c;
  bool showpause = false;
  bool nowait = false;
  
  while((c = getopt(argc, argv, "f:s:phn")) != EOF) {
    switch (c) {
    case 'f': filename = optarg; break;
    case 's': sfilename = optarg; usesoccer = true; break;
    case 'p': showpause = true; break;
    case 'n': nowait = true; break;
    case 'h':
    default: 
      if (argc > 1)
	fprintf(stderr, "%s: Unknown option -%c.", argv[0], c);
      fprintf(stderr, "USAGE: logplay -f <filename> [-s <filename>]\n");
      fprintf(stderr, "\t-f <filename>\tload logfile\n");
      fprintf(stderr, "\t-s <filename>\tload soccer log file\n");
      fprintf(stderr, "\t-p \tsend halted information\n");
      fprintf(stderr, "\t-n \tnot at original frame rate\n");
      exit(1);
    }
  }

  if (!filename) { 
    fprintf(stderr, "%s: Must specify log file with -f.\n", argv[0]); 
    exit(1); 
  }

  // open sockets as server
  if (vision_s.connect_server(NET_VISION_PORT) != Socket::Server) {
    fprintf(stderr, "Cannot create vision server\n");
    exit(1);
  }

  if (radio_s.connect_server(NET_RADIO_PORT) != Socket::Server) {
    fprintf(stderr, "Cannot create radio port at %d\n", NET_RADIO_PORT);
    exit(1);
  }

  // if we have a soccer open it to
  if (usesoccer && (gui_s.connect_server(NET_GUI_PORT) != Socket::Server)) {
    fprintf(stderr, "Cannot setup gui server on port %d\n", NET_GUI_PORT);
    exit(1);
  }

  // open the file
  if ((logfile = fopen(filename, "r")) == NULL) {
    fprintf(stderr, "ERROR: Cannot read log fiel %s\n", filename);
    exit(1);
  }

  // open the soccer file
  if (usesoccer && ((slogfile = fopen(sfilename, "r")) == NULL)) {
    fprintf(stderr, "ERROR: Cannot read soccer log file %s\n", sfilename);
    exit(1);
  }
    

  /* Timing */
  struct timeval current, last;
  double secs, total = 0.0, rtotal = 0.0;
  long vframes = 0;

  gettimeofday(&current, NULL);
  last = current;
  
  while(1) {
    gettimeofday(&current, NULL);
    
    secs = ((current.tv_sec + current.tv_usec / 1000000.0) -
	    (last.tv_sec + last.tv_usec / 1000000.0));

    do_radio_recv();

    //    if ((runmode != PAUSE) && (nowait || (secs >= FRAME_PERIOD))) {
    if ((nowait || (secs >= FRAME_PERIOD))) {

      if (!StepFrame()) {
	//	if (feof(logfile)) {
	//	  fprintf(stderr, "reached end of log!\n");
	//	  continue;
	//	}
	runmode = PAUSE;
      }
      
      if (!debugframes.empty()) {
	if (showpause || (debugframes[currpos].frame.refstate == COMM_START)) {
	  do_vision_send(debugframes[currpos].frame);

	  if (usesoccer)
	    do_gui_send(debugframes[currpos].messages);
	  if ((runmode != PAUSE) && (frames % ((int) FRAME_RATE * 30) == 0))
	    fprintf(stderr, ".");

	} else if (frames % (30 * (int) FRAME_RATE) == 0) {
	  printf("Skipping...\n");
	}
      } 

      last = current;

      total += secs;
      rtotal += secs;
      if (!debugframes.empty() && (frames % ((int) FRAME_RATE * 30) == 0)) {
	double t = debugframes[currpos].frame.timestamp - debugframes[0].frame.timestamp;
	fprintf(stderr, "TIME %1d:%02d frate %f FPS **\n", (int) floor(t / 60.0),
		(int) t % 60, abs(frames - vframes) / rtotal);
	rtotal = 0.0;
	vframes = frames;
      }
    } else {
      if (nowait)
	usleep((unsigned long)(100));
      else
	usleep((unsigned long)(0));
    }
  }
}



static void do_radio_recv(void)
{
  /* check to see if there are new connections */
  while (radio_s.ready_for_accept()) 
    radio_s.accept();


  while (radio_s.ready_for_recv()) {
    static char msg[net_radio_in_maxsize];
    char msgtype;
    net_rcontrol *rctl = (net_rcontrol *) msg;

    // get the message and process it
    radio_s.recv_type(msg, net_radio_in_maxsize, msgtype);

    if (msgtype == NET_RADIO_CONTROL) {
      switch (rctl->control) {
      case VCR_PAUSE: runmode = PAUSE; break;
      case VCR_PLAY: runmode = PLAY; break;
      case VCR_PLAYBACK: runmode = PLAY_BACK; break;
      case VCR_STEP: runmode = STEP; break;
      case VCR_STEPBACK: runmode = STEP_BACK; break;
      case VCR_SLOW: runmode = SLOW; break;
      case VCR_FFWD: runmode = FFWD; break;
      case VCR_RWND: runmode = RWND; break;
      }
    } else {
      fprintf(stderr, "Unknown radio message type, %d!\n", msgtype);
    }
  }
}


static bool do_vision_send(net_vframe &vf)
{
  /* check to see if there are new connections */
  while (vision_s.ready_for_accept()) 
    vision_s.accept();

  while (!vision_s.ready_for_send())
    ;

  vision_s.send(&vf, sizeof(vf));
  return true;
}

static bool ReadFile(void)
{
  bool rval = false;

  if (!debugframes.empty() && (currpos < debugframes.size() - 1)) {
    currpos++;
    rval = true;
  } else {

    frames++;
    DebugFrame df;
    rval =  (fread(&df.frame, sizeof(char), net_vision_out_maxsize, logfile) ==
	     (int) net_vision_out_maxsize);

    if (usesoccer) {
      net_gdebug gdb;
      int size;
      do {
	fread((char *) &size, sizeof(int), 1, slogfile);
	rval = ((int) fread((char *) &gdb, sizeof(char), size, slogfile) == (int) size);
	
	if (gdb.timestamp == df.frame.timestamp)
	  df.messages.push_back(gdb);
      } while (rval & (gdb.timestamp <= df.frame.timestamp));
      
      if (rval)
	fseek(slogfile, -(size + sizeof(int)), SEEK_CUR);
      
    }
    debugframes.push_back(df);
    currpos = debugframes.size() - 1;
  }
  return (rval);
}


static bool StepFrame(void)
{
  bool rval = 0;
  int f;

  switch (runmode) {
  case STEP_BACK:
    runmode = PAUSE;
  case PLAY_BACK:
    if (currpos == 0) {
      runmode = PAUSE;
      return (false);
    }
    currpos--;
    return (true);
  case STEP:
    runmode = PAUSE;
  case PLAY:
    return (ReadFile());

  case FFWD:
    rval = true;
    for (f = 0; (f < FAST_RATE) && rval; f++)
      rval = ReadFile();
    return (rval);

  case RWND:
    if (currpos < FAST_RATE) {
      runmode = PAUSE;
      return (false);
    }

    currpos -= FAST_RATE;
    return (true);
    
  default:
    return (false);
    }
  return (rval);
}


static bool do_gui_send(deque<net_gdebug> &f)
{
  /* check to see if there are new connections */
  while (gui_s.ready_for_accept()) 
    gui_s.accept();

  for (uint i = 0; i < f.size(); i++) {
    while (!gui_s.ready_for_send())
      ;
    
    gui_s.send(&f[i], f[i].size());
  }

  return (true);
}

  
