// logrecord.cc
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

#include "socket.h"
#include "reality/net_vision.h"
#include "reality/net_radio.h"
#include "soccer/net_gui.h"
#include "include/commands.h"

Socket vision_s(NET_VISION_PROTOCOL, NET_VISION_ACK_PERIOD);
Socket radio_s(NET_RADIO_PROTOCOL, NET_RADIO_ACK_PERIOD);
Socket soccer_s(NET_GUI_PROTOCOL, NET_GUI_ACK_PERIOD);

char *hostname = "localhost";
char *soccerhost = "localhost";

int main(int argc, char *argv[])
{
  char *filename = NULL;
  char *soccerlog = NULL;
  bool soccer = false;
  bool recordstop = false;
  bool sendplay = false;
  char c;
  
  while((c = getopt(argc, argv, "V:S:f:s:hrp")) != EOF) {
    switch (c) {
      //    case 'p': port = atoi(optarg); break;
    case 'V': hostname = optarg; break;
    case 'S': soccerhost = optarg; break;
    case 'f': filename = optarg; break;
    case 's': soccerlog = optarg; soccer = true; break;
    case 'r': recordstop = true; break;
    case 'p': sendplay = true; break;
    case 'h':
    default: 
      if (argc != 1)
	fprintf(stderr, "%s: Unknown option -%c.", argv[0], c);

      fprintf(stderr, "USAGE: logrecord -[ph] -f <filename>\n");
      fprintf(stderr, "\t-p <port>\t: port address\n");
      fprintf(stderr, "\t-V <hostname>\t: vision hostname address\n");
      fprintf(stderr, "\t-S <hostname>\t: soccer hostname address\n");
      fprintf(stderr, "\t-f <filename>\t: filename\n");
      fprintf(stderr, "\t-s <filename>\t: soccer log filename\n");
      fprintf(stderr, "\t-r \t: record duing stoppages in play (default off)\n");
      fprintf(stderr, "\t-p \t: send play command to server (default off)\n");
      exit(1);
    }
  }

  if (!filename) { 
    fprintf(stderr, "%s: Must specify log file with -f.\n", argv[0]); 
    exit(1); 
  }

  fprintf(stderr, "Connecting  to Vision on %s\n", hostname);
  while (vision_s.connect_client(hostname, NET_VISION_PORT) != Socket::Client)
    ;
  fprintf(stderr, "Connected to Vision\n");

  if (sendplay) {
    fprintf(stderr, "Connecting  to radio on %s\n", hostname);
    while (radio_s.connect_client(hostname, NET_RADIO_PORT) != Socket::Client)
      ;
    fprintf(stderr, "Connected to Radio\n");
  }

  if (soccer) {
    fprintf(stderr, "Connecting to Soccer on host %s...\n", soccerhost);
    while (soccer_s.connect_client(soccerhost, NET_GUI_PORT) != Socket::Client)
      ;
  }

  FILE *logf, *soccerf = NULL;
  if ((logf = fopen(filename, "w")) == NULL) {
    fprintf(stderr, "ERROR: Cannot open file %s\n", filename);
    exit(1);
  }


  if (soccer) {
    if ((soccerf = fopen(soccerlog, "w")) == NULL) {
      fprintf(stderr, "ERROR: Cannot open soccer log file %s\n", soccerlog);
      exit(1);
    }
  }
    
  char buffer[4096];
  long fcount = 0, scount = 0, flast = 0;

  net_gdebug *gdb = (net_gdebug *) buffer;
  net_vframe *vf =  (net_vframe *) buffer;
  char fstate = COMM_STOP;
  double tstamp = 0;

  int sendcount = 0;

  while (1) {

    //    if (sendplay && (radio_s.get_status() == Socket::Client) &&
    //	(fcount % 10 == 0)) {
    if (sendplay && (radio_s.get_status() == Socket::Client) &&
	(sendcount++ % 100 == 0)) {
      while (!radio_s.ready_for_send())
	;

      net_rcontrol rc = {NET_RADIO_CONTROL, VCR_PLAY};
      radio_s.send(&rc, sizeof(rc));
    }

    // read vision
    if (vision_s.get_status() == Socket::Client) {
      while(vision_s.ready_for_recv()) {

	int bsize = vision_s.recv(buffer, net_vision_out_maxsize);
	fstate = vf->refstate;
	tstamp = vf->timestamp;
	if (recordstop || (fstate == COMM_START)) {
	  fwrite(buffer, sizeof(char), bsize, logf);
	  fcount++;
	}
      }
    }
    if (soccer) {
      int rv = 1;
      int myflast = flast;

      while (soccer_s.ready_for_recv() && (rv > 0)) {
	if (myflast < fcount) {
	  myflast = fcount;
	  scount++;
	}
	rv = soccer_s.recv(buffer, net_gui_out_maxsize);
	rv = gdb->size();
	if (recordstop || ((fstate == COMM_START) && (tstamp == gdb->timestamp))) {
	  fwrite(&rv, sizeof(int), 1, soccerf);
	  fwrite(buffer, sizeof(char), gdb->size(), soccerf);
	}
      }
    }

    if (flast < fcount) {
      if (fcount % ((long) FRAME_RATE * 30L) == 0)
	fprintf(stderr, ".");
      if (soccer && (scount % ((long) FRAME_RATE * 30L) == 0))
	fprintf(stderr, "+");
      
      if (fcount % ((int) FRAME_RATE * 60 * 2) == 0) {
	fprintf(stderr, "\n%i minutes recorded\n", (int) fcount / ((int) FRAME_RATE * 60));
      }
      fflush(stderr);
    }
    flast = fcount;
  }

  fprintf(stderr, "Closing...\n");
  fclose(logf);
  if (soccerlog)
    fclose(soccerf);
  fprintf(stderr, "Received %i frames\n", (int) fcount);
  return (0);
}


