/*
 * TITLE:	VClient.cc
 *
 * PURPOSE:	This is the main entry point for the vision client text test program
 *              The file connects to the vision server and dumps the frame output to the
 *              screen in text format
 *
 * COMMAND LINE: the command line arguments are:
 *
 * WRITTEN BY: James R Bruce
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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "constants.h"
#include "client.h"

// 5b1b 1b48 325b 004a


void reset_cursor()
{
  printf("%c%c%c",
         0x1b,0x5b,0x48);
  //         0x1b,0x5b,0x32,0x4a);
}

void clear_screen()
{
  printf("%c%c%c%c%c%c%c",
         0x1b,0x5b,0x48,
         0x1b,0x5b,0x32,0x4a);
}


int main(int argc,char *argv[])
{
  Client client;
  net_vframe vf;
  net_vconfig vc;

  char *hostname;
  int i;

  hostname = "calvin.prodigy.cs.cmu.edu";
  if(argc >= 2) hostname = argv[1];

  printf("using %s.\n",hostname);
  client.Initialize(hostname);

  // test code
  if (true) {
    memset(&vc, 0, sizeof(net_vconfig));
    vc.msgtype = NET_VISION_CONFIG;

    vc.teams[TEAM_BLUE].robots[0].id = 0;
    vc.teams[TEAM_BLUE].robots[1].id = 1;
    vc.teams[TEAM_BLUE].robots[2].id = 2;
    vc.teams[TEAM_BLUE].robots[3].id = 3;
    vc.teams[TEAM_BLUE].robots[4].id = 5;
    vc.teams[TEAM_BLUE].cover_type = VCOVER_NORMAL;

    vc.teams[TEAM_YELLOW].robots[0].id = 10;
    vc.teams[TEAM_YELLOW].robots[1].id = 11;
    vc.teams[TEAM_YELLOW].robots[2].id = 12;
    vc.teams[TEAM_YELLOW].robots[3].id = 13;
    vc.teams[TEAM_YELLOW].robots[4].id = 15;
    vc.teams[TEAM_YELLOW].cover_type = VCOVER_NONE;
    client.Configure(vc);
  }

  // Get vision updates for awhile and print them
  memset(&vf,0,sizeof(net_vframe));

  for(i=0; i<60*60*5; i++){
    if(i % 60){
      reset_cursor();
    }else{
      clear_screen();
    }

    client.GetUpdate(vf);

    // printf("%f\n",loc.timestamp-loc.ball.timestamp);
#if 0
    printf("Time: %f \n",vf.timestamp);
    printf("Ball: (%8.2f,%8.2f):%6.4f t%6.4f \n",
    	   vf.ball.cur.loc.x,loc.ball.cur.loc.y,loc.ball.conf,
	   loc.ball.timestamp-loc.timestamp);
    
    for(team=0; team<NUM_TEAMS; team++){
      printf("Team %s:\n",(team==TEAM_BLUE)?"Blue":"Yellow");
      for(j=0; j<MAX_TEAM_ROBOTS; j++){
	printf("  Robot %d: (%8.2f,%8.2f),%8.2fr : %6.4f t%6.4f \n",
	       j+1,
	       loc.robot[team][j].cur.loc.x,
	       loc.robot[team][j].cur.loc.y,
	       loc.robot[team][j].cur.angle,
	       loc.robot[team][j].conf,
	       loc.robot[team][j].timestamp-loc.timestamp);
      }
    }
#endif
  }

  client.Close();
  return(0);

  // Vision::SaveThresholdImage("/tmp/cmap.ppm");
}
