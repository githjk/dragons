// net_radio.h
//
// Network interface for radio socket.  See socket.h for info
// on connecting a client. 
//
// Given a connected client, here's some example code...
//
// rcommand cmd = { NET_RADIO_COMMMAND, id, 1.0, 0.0, 0.0, false };
// client->send(&cmd, sizeof(cmd));
// ...
// rcontrol ctl = { NET_RADIO_CONTROL, VCR_PAUSE };
// client->send(&ctl, sizeof(ctl));
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

#ifndef __net_radio_h__
#define __net_radio_h__

#include "../include/rtypes.h"
#include "../include/constants.h"
#include "../utils/macros.h"

/* port addresses, protocol and acknowledgement period */
#define NET_RADIO_PORT         32882
#define NET_RADIO_PROTOCOL     Socket::UDP
#define NET_RADIO_ACK_PERIOD   30

// ------------------------------------------------------------------
// Input Messages
// ------------------------------------------------------------------

#define NET_RADIO_COMMAND  1
#define NET_RADIO_CONTROL  2

#define NET_RADIO_COMMANDS  3

//struct net_rcommand;
struct net_rcontrol;
struct net_rcommand;
struct net_rcommands;

//
// rcommand
//

//struct net_rcommand {
//  char msgtype;         // = NET_RADIO_COMMAND
  
//  uchar team;    // this wasn't added in
//  uchar id;             // Radio id of robot.
//  double vx, vy, va;    // Velocities in mm/s and radians/s.
//  bool kick, dribble;
//  char priority;
//};

struct net_rcommand {
  uchar team;     // what team it is on
  uchar mode;     // 0=sleep,1=run
  uchar priority; // priority level of command (higher number = higher prio)
  uchar id,type;  // radio ID and type of robot
  short dx,dy,da; // velocity in mm/sec + mrad/s
  bool kick, drib;
};


struct net_rcommands {
  char msgtype; // = NET_RADIO_COMMANDS
  double time_stamp;
  uchar nr_commands;
  net_rcommand cmds[1];
};


//
// rcontrol
//

#define VCR_PAUSE         0
#define VCR_PLAY          1
#define VCR_STEP          2
#define VCR_SLOW          3

// currently unimplemented
#define VCR_FFWD     4
#define VCR_RWND     5

#define VCR_PLAYBACK     6
#define VCR_STEPBACK     7

struct net_rcontrol {
  char msgtype;         // = NET_RADIO_CONTROL
  char control;         // Control command, see above.
};

//const int net_radio_in_maxsize = MAX(sizeof(net_rcommand), 
//				     sizeof(net_rcontrol));

const int net_radio_in_maxsize = MAX(sizeof(net_rcommands) + 
				     sizeof(net_rcommand) * (MAX_TEAM_ROBOTS - 1), 
				     sizeof(net_rcontrol));

#endif
