/*
 * TITLE:	vtypes.h
 *
 * PURPOSE:	This is file encapsulates the vision types that are required
 *              by all the users. It was made from vision_client.h in the 
 *              2001 code base.
 *
 * WRITTEN BY:	James R Bruce, Brett Browning
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

#ifndef __VTYPES_H__
#define __VTYPES_H__

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "geometry.h"
#include "include/constants.h"
#include "utils/socket.h"

// detection attributes
#define VISION_DETECT_ORIENTATION 1
#define VISION_DETECT_IDENTIFY    3

//==== Vision Data ====//

struct rparam{
  char enabled;  // is this robot present?
  char marker;   // vision marker number on top
  char radio_id; // radio ID number
  char type;     // type of robot diff/omni
public:
  void set(char nenabled,char nmarker,char nradio_id,char ntype)
    {enabled=nenabled; marker=nmarker; radio_id=nradio_id; type=ntype;}
};

struct tparam{
  char team;         // which side we are playing
  char num_robots;   // number of robots on the team
  char detect_attr;  // VISION_DETECT flags
  char marker_color; // which team's marker is used (normally same as team)
  rparam robot[MAX_TEAM_ROBOTS];  // individual robot parameters
};

struct vposition{
  vector2d loc;  // (x,y) location on the field
  double angle;  // angle (if known)
};

struct vlocation{
  double conf;       // Confidence [0,1] of detection accuracy
  double timestamp;  // timestamp of last detection
  vposition cur;     // last detected position
  char enabled;      // 1 if trying to detect this object, 0 if not
  char jumped;       // Changed which object we are tracking since last report
  char orientation;  // nonzero if orientation has successfully been detected
};

struct vlocations{
  vlocation robot[NUM_TEAMS][MAX_TEAM_ROBOTS];
  vlocation ball;
  double timestamp;
  tparam param[NUM_TEAMS];
};



#endif /* __VTYPES_H__ */

