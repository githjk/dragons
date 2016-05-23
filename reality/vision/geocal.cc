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

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include <string.h>

#include "camera.h"

// 5b1b 1b48 325b 004a
// 0x1b,0x5b,0x32,0x4a

/*
void reset_cursor()
{
  printf("%c%c%c",
         0x1b,0x5b,0x48);
}

void clear_screen()
{
  printf("%c%c%c%c%c%c%c",
         0x1b,0x5b,0x48,
         0x1b,0x5b,0x32,0x4a);
}
*/

int main(int argc,char **argv)
{
  camera cam;
  const char *configdir,*camera,*params;
  char buf[256];

  // geocal <configdir> <camera> <params>

  if(argc >= 2){
    configdir = argv[1];
  }else{
    configdir = getenv("F180CONFIG");
    if(!configdir) configdir = ".";
  }

  camera = (argc >= 3)? argv[2] : "camera1.txt";
  params = (argc >= 4)? argv[3] : "params1.txt";

  printf("calibrating...\n");

  snprintf(buf,256,"%s/%s",configdir,camera);
  cam.loadParam(buf);

  snprintf(buf,256,"%s/%s",configdir,params);
  cam.calibrate(buf);

  return(0);
}
