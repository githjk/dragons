/*
 * TITLE:	RoboComms.cc
 *
 * PURPOSE:	This class encapsulates the robocomms interface. It makes
 *              up radio packets from the velocity commands and sends them
 *              to the robots via the serial port.
 *
 * WRITTEN BY:	      James R Bruce, Brett Browning
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

#ifndef __ROBOCOMMS_H__
#define __ROBOCOMMS_H__

#include "vector.h"
#include "constants.h"
#include "rtypes.h"
#include "serial.h"

#include "reality/net_radio.h"

// Packet Flags
#define RF_REVERSE_DX 0x01
#define RF_REVERSE_DY 0x02
#define RF_REVERSE_DA 0x04
#define RF_HALT       0x08
#define RF_KICK	      0x10
#define RF_DRIBBLE    0x20
#define RF_ALL        0x0F

// Robot Modes
#define RF_MODE_SLEEP 0
#define RF_MODE_RUN   1
#define RF_NUM_MODES  2


class RoboComms {
public:
  struct packet {
    uchar start;    // always 0x33
    uchar robot;    // from identifier[]
    uchar dx,dy,da; // velocities in cm/sec
    uchar flags;    // flag bits
    uchar checksum; // checksum for 5 data fields
    uchar stop;     // always 0x55
  };

//  struct command{
//    uchar mode;     // 0=sleep,1=run
//    uchar priority; // priority level of command (higher number = higher prio)
//    uchar id,type;  // radio ID and type of robot
//    short dx,dy,da; // velocity in mm/sec + mrad/s
//    bool kick, drib;
//  };

protected:
  net_rcommand cmd[MAX_ROBOTS];
  double cmd_time[MAX_ROBOTS];
  int current_id;
  serial ser;

  bool sleeping;
  int bytes_sent;
  int one_bits_sent;
protected:
  unsigned char checksum(packet &rp);
  bool makeRadioPacket(packet &rp, net_rcommand &cmd);
  bool verifyPacket(packet &rp);
public:
  bool init();
  void close();

  bool run();

  bool verifyCommand(net_rcommand &cmd);
  bool setCommand(net_rcommand &ncmd, double time);
  int setCommands(net_rcommand *ncmd, int num);

  // should nt be called any more
  int setCommand(char rid, double vx, double vy, double va, 
		 bool kick = false, bool drib = false);

  void setSleep(bool _sleeping) {sleeping=_sleeping;}
};

#endif /* __ROBOCOMMS_H__ */

