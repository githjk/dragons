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
#include "robocomm.h"

double vel[8][3] = {
  {  250,   0,   0},
  {    0, 250,   0},
  { -250,   0,   0},
  {    0,-250,   0},
  {    0,   0,   2},
  {    0,   0,   0},
  {    0,   0,  -2},
  {    0,   0,   0}
};

int main(int argc,char *argv[])
{
  RoboComm robocomm;
  char *hostname;
  int rid,type;
  int i,j;

  if(argc < 3) return(1);
  type = ROBOT_TYPE_NONE;
  if(strcasecmp(argv[1],"diff") == 0) type = ROBOT_TYPE_DIFF;
  if(strcasecmp(argv[1],"omni") == 0) type = ROBOT_TYPE_OMNI;
  rid = atoi(argv[2]);
  printf("type=%d, rid=%d\n",type,rid);
  if(!type || rid<0 || rid>5) return(2);

  hostname = "calvin.prodigy.cs.cmu.edu";
  if(argc >= 4) hostname = argv[3];

  printf("using %s.\n",hostname);
  robocomm.init(hostname);

  for(i=0; i<8; i++){
    printf("%g %g %g\n",vel[i][0],vel[i][1],vel[i][2]);
    for(j=0; j<4; j++){
      robocomm.setCommand(rid,type,
			  vel[i][0]*2,vel[i][1]*2,vel[i][2]);
      robocomm.send();
    }
    usleep(500*1000);
  }
  robocomm.close();

  return(0);
}
