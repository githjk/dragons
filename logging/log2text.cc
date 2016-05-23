// main.cc
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
#include <slist>
#include <list>

#include "../utils/socket.h"
#include "../reality/net_vision.h"
#include "../reality/net_radio.h"
#include "constants.h"

#define FIELD_VISION  0x0001
#define FIELD_TRACKER 0x0002

#define FIELD_BALL    0x0010
#define FIELD_BLUE    0x0020
#define FIELD_YELLOW  0x0040
#define FIELD_REF     0x0080
#define FIELD_ALL     0xFFFF

/*********************** PROTOTYPES **************************/
bool do_frame_write(FILE *f, net_vframe &vf);


/*********************** GLOBALS *****************************/
int fields = 0;
int robots[NUM_TEAMS][MAX_TEAM_ROBOTS];
int nr_robots[NUM_TEAMS];

int main(int argc, char *argv[])
{
  char *input_fname = NULL;
  char *output_fname = NULL;
  FILE *logfile = NULL;
  FILE *textfile = NULL;
  double tstart = 0, tend = 0;
  
  char c;
  int t;
  char str[256];
  for (int t = 0; t < NUM_TEAMS; t++) {
    nr_robots[t] = 0;
    for (int i = 0; i < MAX_TEAM_ROBOTS; i++)
      robots[t][i] = -1;
  }
  
  while((c = getopt(argc, argv, "Y:B:bvtrf:ho:")) != EOF) {
    switch (c) {
    case 'b': fields |= FIELD_BALL; break;
    case 'v': fields |= FIELD_VISION; break;
    case 't': fields |= FIELD_TRACKER; break;
    case 'r': fields |= FIELD_REF; break;

    case 'B': 
    case 'Y':
      strcpy(str, optarg);
      t = ((c == 'B') ? 0 : 1);
      if (t) {
	fields |= FIELD_YELLOW;
      } else {
	fields |= FIELD_BLUE;
      }

      nr_robots[t] = sscanf(str, "%i,%i,%i,%i,%i", &robots[t][0], 
			    &robots[t][1], &robots[t][2], 
			    &robots[t][3], &robots[t][4]);

      if (nr_robots[t] <= 0) {
	fprintf(stderr, "ERROR: No robot list!!!\n");
	fprintf(stderr, "Use a comma delimited format, no spaces e.g. -B 0,3,4,9\n");
	exit(1);
      } 
      break;

    case 'f': input_fname = optarg; break;
    case 'o': output_fname = optarg; break;
    case 'h':
    default: 
      if (argc > 1)
	fprintf(stderr, "%s: Unknown option -%c.", argv[0], c);
      fprintf(stderr, "USAGE: log2text -f <filename> -o <file>\n");
      fprintf(stderr, "\t-f <filename>\t input log file\n");
      fprintf(stderr, "\t-o <filename>\t output text file\n");

      fprintf(stderr, "\nOptions for selective fields\n");
      fprintf(stderr, "\t-v \t raw vision fields\n");
      fprintf(stderr, "\t-t \t tracker fields\n");
      fprintf(stderr, "\t-r \t referee field\n");
      fprintf(stderr, "\t-b \t ball field\n");
      fprintf(stderr, "\t-B \t Blue field\n");
      fprintf(stderr, "\t-Y \t Yellow field\n");

      exit(1);
    }
  }

  if (fields == 0) {
    fields = FIELD_ALL;
  }

  // print what optinos were
  fprintf(stderr, "\nConverting using the following options:\n");
  fprintf(stderr, "\tconverting %s -> to text file %s\n", input_fname, output_fname);
  if (fields == FIELD_ALL)
    fprintf(stderr, "using all fields\n");
  else {
    fprintf(stderr, "fields %x got here...\n", fields);

    if (fields & FIELD_VISION)
      fprintf(stderr, "raw vision, ");

    if (fields & FIELD_TRACKER)
      fprintf(stderr, "tracker, ");

    if (fields & FIELD_BALL)
      fprintf(stderr, "ball, ");

    if (fields & FIELD_BLUE) {
      fprintf(stderr, "\nblue team with %i robots: ", nr_robots[0]);
      for (int i = 0; i < nr_robots[0]; i++)
	fprintf(stderr, "%i, ", robots[0][i]);
    }

    if (fields & FIELD_YELLOW) {
      fprintf(stderr, "\nyellow team with %i robots: ", nr_robots[1]);
      for (int i = 0; i < nr_robots[1]; i++)
	fprintf(stderr, "%i, ", robots[1][i]);
    }

    fprintf(stderr, "\n");

  }
    
  if (!input_fname) { 
    fprintf(stderr, "%s: Must specify input log file with -f.\n", argv[0]); 
    exit(1); 
  }
  if (!output_fname) { 
    fprintf(stderr, "%s: Must specify outptu text file with -f.\n", argv[0]); 
    exit(1); 
  }

  if ((logfile = fopen(input_fname, "r")) == NULL) {
    fprintf(stderr, "%s: Cannot open logfile %s.\n", argv[0], input_fname); 
    exit(1); 
  }
  if ((textfile = fopen(output_fname, "wt")) == NULL) {
    fprintf(stderr, "%s: Cannot open output file %s.\n", 
	    argv[0], output_fname); 
    exit(1); 
  }

 
  int packet = 0;
  while(!feof(logfile)) {
    net_vframe vf;

    int bsize = fread((char *) &vf, sizeof(char), net_vision_out_maxsize, logfile);
    if (bsize != net_vision_out_maxsize) {
      fprintf(stderr, "Bad packet in file %d\n", packet + 1);
      break;
    }

    // write it to the file
    do_frame_write(textfile, vf);
    if (packet == 0) {
      tstart = vf.timestamp;
    }
    tend = vf.timestamp;

    packet++;
  }

  fclose(textfile);
  fclose(logfile);

  tend -= tstart;
  printf("Wrote %i packets, and %1.0f:%2.2f minutes\n", packet, 
	 floor(tend / 60), tend - (floor(tend/60) * 60.0));
  return (0);
}


bool do_frame_write(FILE *f, net_vframe &vf)
{
  fprintf(f, "%f ", vf.timestamp);

  // ball info
  if (fields & FIELD_BALL) {
    if (fields & FIELD_VISION) {
      fprintf(f, "%f %f %f %f ", vf.ball.vision.timestamp,
	      vf.ball.vision.pos.x, vf.ball.vision.pos.y, 
	      vf.ball.vision.conf);
    }
    if (fields & FIELD_TRACKER) {
      fprintf(f, "%f %f %f %f ", vf.ball.state.x, vf.ball.state.y,
	      vf.ball.state.vx, vf.ball.state.vy);
      for (int i = 0; i < 16; i++) {
	fprintf(f, "%f ", vf.ball.variances[i]);
      }
    }
  }

  // robot info
  // fmt: team cover [robot info] x MAX_TEAM_ROBOT
  // robot: id type vraw state
  for (int t = 0; t < NUM_TEAMS; t++) {
    if (((t == TEAM_BLUE) && (fields & FIELD_BLUE)) ||
	((t == TEAM_YELLOW) && (fields & FIELD_YELLOW))) {

      fprintf(f, "%d ", vf.config.teams[t].cover_type);
      for (int i = 0; i < nr_robots[t]; i++) {

	// find teh robot with the right id in the array
	// must do everytime in case config changes
	int id = robots[t][i];
	int aid = -1;
	for (int j = 0; (aid < 0) && (j < MAX_TEAM_ROBOTS); j++) {
	  if (vf.config.teams[t].robots[j].id == id) {
	    aid = j;
	  }
	}

	if (aid >= 0) {
	  fprintf(f, "%d %d ", vf.config.teams[t].robots[aid].id,
		  vf.config.teams[t].robots[aid].type);
	  if (fields & FIELD_VISION) {
	    fprintf(f, "%f %f %f %f %f ", vf.robots[t][i].vision.timestamp,
		    vf.robots[t][i].vision.pos.x, vf.robots[t][i].vision.pos.y, 
		    vf.robots[t][i].vision.angle, vf.robots[t][i].vision.conf);
	  }
	  if (fields & FIELD_TRACKER) {
	    fprintf(f, "%f %f %f %f %f %f %f ", 
		    vf.robots[t][i].state.x, vf.robots[t][i].state.y, 
		    vf.robots[t][i].state.theta,
		    vf.robots[t][i].state.vx, vf.robots[t][i].state.vy, 
		    vf.robots[t][i].state.vtheta,
		    vf.robots[t][i].state.stuck);
	  }
	}
      }
    }
  }

  // do we store the refstate or not?
  if (fields & FIELD_REF)
    fprintf(f, "%i", vf.refstate);
  fprintf(f, "\n");

  return true;
}
