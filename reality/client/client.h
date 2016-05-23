/*
 * TITLE:	Client.h
 *
 * PURPOSE:	This file encapsulates the radio/vision client interfaces that 
 *              hide the socket interface to the vision/radio servers
 *
 * WRITTEN BY:	      James R Bruce, Scott Lenser, Michael Bowling, Brett Browning
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

#ifndef __CLIENT_H__
#define __CLIENT_H__

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <stdlib.h>
#include <stdio.h>
//#include <unistd.h>

#include "utils/socket.h"
#include "reality/net_vision.h"
#include "reality/net_radio.h"
#include "../vision/vtypes.h"

#include "reality/radio/robocomms.h"


//==== Vision/Radio  External Interface ====//

class Client {
private:
  
  struct RCommands {
    char msgtype; // = NET_RADIO_COMMANDS
    double timestamp;
    char nr_commands;
    net_rcommand commands[MAX_TEAM_ROBOTS];

    uint size(void) {
      return (sizeof(struct RCommands) - (MAX_TEAM_ROBOTS - nr_commands - 1) *
	      sizeof(net_rcommand));
    }

  } rcommands;

public:
  Socket vision_s;
  Socket radio_s;

  // intialization stuff
  Client(void) {
    vision_s.set(NET_VISION_PROTOCOL, NET_VISION_ACK_PERIOD);
    radio_s.set(NET_RADIO_PROTOCOL, NET_RADIO_ACK_PERIOD);
    rcommands.msgtype = NET_RADIO_COMMANDS;
    rcommands.nr_commands = 0;
  }

  bool Initialize(char *hostname, int vport = NET_VISION_PORT, int rport = NET_RADIO_PORT);
  void Close(void);
  bool IsConnected(void) {
    return ((vision_s.get_status() == Socket::Client) &&
	    (radio_s.get_status() == Socket::Client));
  }


  // Vision wrappers
  bool Configure(struct net_vconfig &vc);
  bool SendRef(char state);
  bool MoveBall(double x, double y, double vx, double vy);
  bool MoveRobot(uchar team, uchar id, double x, double y, double angle);

  bool GetUpdate(net_vframe &vf);

  // Radio commands
  bool EnableRadio(bool en);
  bool RadioControl(char ctrl);

  // function to set a command in teh command list...will not send it
  bool SetCommand(double tstamp, char team, char rid, double vx, double vy, double va, 
		  bool kick = false, bool drib = false, char priority = 1);
  bool SetCommand(double tstamp, char team, char rid, vector3d vcmd,
		  bool kick = false, bool drib = false, char priority = 1) {
    return SetCommand(tstamp, team, rid, vcmd.x, vcmd.y, vcmd.z, kick, drib, priority);
  }
  bool SetCommand(double tstamp, int team, int rid, vector3d vcmd,
		  bool kick = false, bool drib = false, char priority = 1) {
    return SetCommand(tstamp, (char) team, (char) rid, vcmd.x, vcmd.y, vcmd.z, 
		      kick, drib, priority);
  }

  bool SetHalt(double tstamp, char team, char rid, char priority = 1);
  bool SetHalt(double tstamp, int team, int rid, int priority = 1) {
    return(SetHalt(tstamp, (char) team, (char) rid, (char) priority)); }

  // set a single command and send it
  bool SendCommand(double tstamp, char team, char rid, double vx, double vy, double va, 
		   bool kick = false, bool drib = false, char priority = 1) {
    if (SetCommand(tstamp, team, rid, vx, vy, va, kick, drib, priority))
      return Send();
    return false;
  }
  bool SendCommand(double tstamp, char team, char rid, vector3d vcmd,
		   bool kick = false, bool drib = false, char priority = 1) {
    return SendCommand(tstamp, team, rid, vcmd.x, vcmd.y, vcmd.z, kick, drib, priority);
  }

  // send all the commands 
  bool Send(void);
};

#endif /* __CLIENT_H__ */

