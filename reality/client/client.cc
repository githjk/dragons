/*
 * TITLE:	Client.cc
 *
 * PURPOSE:	This file encapsulates the radio/vision client interfaces that 
 *              hide the socket interface to the vision/radio servers
 *
 * WRITTEN BY:  James R Bruce, Scott Lenser, Michael Bowling, Brett Browning
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

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "util.h"
#include "constants.h"
#include "utils/socket.h"
#include "reality/net_vision.h"

#include "client.h"


bool Client::Initialize(char *hostname, int vport, int rport)
{
 if (vision_s.connect_client(hostname, vport) != Socket::Client)
   return (false);
 if (radio_s.connect_client(hostname, rport) != Socket::Client)
   return (false);
 return (true);
}

void Client::Close(void)
{
  vision_s.disconnect();
  radio_s.disconnect();
}

bool Client::Configure(struct net_vconfig &vc)
{
  vc.msgtype = NET_VISION_CONFIG;
  vision_s.send(&vc, sizeof(vc));
  return(true);
}

bool Client::GetUpdate(net_vframe &vf)
{
  static char msg[net_vision_out_maxsize];
  char msgtype;

  if (vision_s.get_status() != Socket::Client)
    return (false);
	
  /* get the message and block until we do */
  if (vision_s.recv_type(msg, net_vision_out_maxsize, msgtype) <= 0)
    return (false);

  if (msgtype == NET_VISION_FRAME)
    memcpy((char *) &vf, msg, sizeof(vf));
  return(true);
}

bool Client::SendRef(char state) 
{
  net_vref c = {NET_VISION_REF, state};
  if (!vision_s.ready_for_send()) {
    fprintf(stderr, "Cannot send yet\n");
    return false;
  }
  vision_s.send(&c, sizeof(c));
  return true;
}

bool Client::MoveBall(double x, double y, double vx, double vy) 
{
  net_vsim c = {NET_VISION_SIM, VSIM_MOVEBALL};
  c.info.moveball = (vsim_moveball) {{x, y}, {vx, vy}};
  vision_s.send(&c, sizeof(c));
  return true;
}

bool Client::MoveRobot(uchar team, uchar id, double x, double y, double angle) 
{
  net_vsim c = {NET_VISION_SIM, VSIM_MOVEROBOT};
  c.info.moverobot =  (vsim_moverobot) {team, id, {x, y}, angle};
  vision_s.send(&c, sizeof(c));
  return true;
}


bool Client::EnableRadio(bool en)
{
  char cmd;
  if (en)
    cmd = VCR_PLAY;
  else
    cmd = VCR_PAUSE;
    
  net_rcontrol rc = (net_rcontrol) {NET_RADIO_CONTROL, cmd};

  radio_s.send(&rc, sizeof(rc));
  return (true);
}

bool Client::RadioControl(char ctrl)
{
  net_rcontrol rc = (net_rcontrol) {NET_RADIO_CONTROL, ctrl};
  radio_s.send(&rc, sizeof(rc));
  return (true);
}


bool Client::SetCommand(double tstamp, char team, char rid, double vx, double vy, double va, 
		bool kick = false, bool drib = false, char priority = 1)
{
  // need to add command if we can...
  if (rcommands.nr_commands >= MAX_TEAM_ROBOTS) {
    fprintf(stderr, "ERROR: Cannot add any more commands!!!\n");
    fprintf(stderr, "DBG INFO: time %f team %i rid %i vel (%f, %f, %f), k/d (%i, %i) priority %i\n",
	    tstamp, team, rid, vx, vy, va, kick, drib, priority);
    return false;
  }

  // do a nan check first
  if (isnan(vx) || isnan(vy) || isnan(va)) {
    fprintf(stderr, "NAN's in velocity command: %f %f %f\n", vx, vy, va);
    return false;
  }

  // do an id sanity check
  if ((rid < 0) || (rid > MAX_ROBOT_ID)) {
    fprintf(stderr, "Bad Robot id %i\n", rid);
    return false;
  }

  //  fprintf(stderr, "\tAdding command t %i, r %i v(%f %f %f)\n", team, rid, vx, vy, va);

  int i = rcommands.nr_commands;

  rcommands.commands[i].mode     = RF_MODE_RUN;
  rcommands.commands[i].priority = priority;
  rcommands.commands[i].id       = rid;
  rcommands.commands[i].team     = team;
  rcommands.commands[i].type     = ROBOT_TYPE_ID(rid);
  rcommands.commands[i].dx       = (int) vx;
  rcommands.commands[i].dy       = (int) vy;
  rcommands.commands[i].da       = (int) (va*1000); // convert to mrad/sec
  rcommands.commands[i].kick     = kick;
  rcommands.commands[i].drib     = drib;

  // what do we do withthe timestamp??? For now set it to 
  // latest timestamp available
  rcommands.timestamp = max(rcommands.timestamp, tstamp);

  // add it to the list
  rcommands.nr_commands++;

  // all done
  return true;
}

bool Client::SetHalt(double tstamp, char team, char rid, char priority)
{
  // need to add command if we can...
  if (rcommands.nr_commands >= MAX_TEAM_ROBOTS) {
    fprintf(stderr, "ERROR: Cannot add any more commands!!!\n");
    fprintf(stderr, "DBG INFO: time %f team %i rid %i HALT\n",
	    tstamp, team, rid);
    return false;
  }

  // do an id sanity check
  if ((rid < 0) || (rid > MAX_ROBOT_ID)) {
    fprintf(stderr, "Bad Robot id %i\n", rid);
    return false;
  }

  int i = rcommands.nr_commands;

  rcommands.commands[i].mode     = RF_MODE_SLEEP;
  rcommands.commands[i].priority = priority;
  rcommands.commands[i].id       = rid;
  rcommands.commands[i].team     = team;
  rcommands.commands[i].type     = ROBOT_TYPE_ID(rid);
  rcommands.commands[i].dx       = (int) 0;
  rcommands.commands[i].dy       = (int) 0;
  rcommands.commands[i].da       = (int) 0;
  rcommands.commands[i].kick     = false;
  rcommands.commands[i].drib     = false;

  // what do we do withthe timestamp??? For now set it to 
  // latest timestamp available
  rcommands.timestamp = max(rcommands.timestamp, tstamp);

  // add it to the list
  rcommands.nr_commands++;

  // all done
  return true;
}

// send all the commands 
bool Client::Send(void)
{
  // check if there is anything to send
  if (rcommands.nr_commands == 0)
    return false;

  if (radio_s.get_status() != Socket::Client) {
    fprintf(stderr, "ERROR: Socket disconnected so cannot send!!!\n");
    return false;
  }

  bool rval = radio_s.send((net_rcommands *) &rcommands, rcommands.size());

  // refreshteh command structure so we can add more things to it
  rcommands.nr_commands = 0;
  return rval;
}

