/*
 * TITLE:	RoboComms.cc
 *
 * PURPOSE:	This class encapsulates the robocomms interface. It makes
 *              up radio packets from the velocity commands and sends them
 *              to the robots via the serial port.
 *
 * WRITTEN BY:  James R Bruce, Brett Browning
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

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "util.h"
#include "serial.h"
#include "constants.h"
#include "robocomms.h"

// #define DEBUG

const bool print_serial_rate = false;
const bool print_packet_sent = false;

/****************************** CONSTANTS ******************************/

// how long to wait in sec before not sending on the radio
#define ROBOT_TIMEOUT 2.000

// how long to wait in sec before letting a lower priority take over
#define ROBOT_PRIORITY_TIMEOUT 1.000

// Serial parameters
#define SERIAL_DEVICE  "/dev/ttyS0"
#define BAUD_RATE      38400

#define ROBOT_WHEELBASE 120


const bool dump_command = true;

// must be at least MAX_ROBOTS entries
const uchar identifier[12] = {
  0xAA,  0xAB,  0xBA,
  0xBB,  0xBC,  0xCB,
  0xCC,  0xCD,  0xDC,
  0xDD,  0xDE,  0xED
};


/****************************** CODE ************************************/

// should export this for global use as a util
double gettime()
{
  timeval tv;
  gettimeofday(&tv,NULL);
  return(tv.tv_sec + tv.tv_usec/1.0E6);
}

int count_bits(unsigned u)
{
  u = ((u & 0xAAAAAAAAL) >>  1) + (u & 0x55555555L);
  u = ((u & 0xCCCCCCCCL) >>  2) + (u & 0x33333333L);
  u = ((u & 0xF0F0F0F0L) >>  4) + (u & 0x0F0F0F0FL);
  u = ((u & 0xFF00FF00L) >>  8) + (u & 0x00FF00FFL);
  u = ((u & 0xFFFF0000L) >> 16) + (u & 0x0000FFFFL);

  return((int)u);
}

int count_bits(void *arr,size_t num)
{
  unsigned char *s = (unsigned char*)arr;
  unsigned i,sum;

  sum = 0;
  for(i=0; i<num; i++){
    sum += count_bits(s[i]);
  }

  return(sum);
}

void memchecker(void *arr, size_t num)
{
  unsigned char *s = (unsigned char*)arr;
  unsigned i;

  for(i=0; i<num; i++){
    s[i] = (i%2)? 255 : 0;
  }
}

/****************************** ROBOCOMM CLASS **************************/

unsigned char RoboComms::checksum(packet &rp)
{
  unsigned u;

  u  = (rp.robot     );
  u ^= (rp.dx    << 1);
  u ^= (rp.dy    << 2);
  u ^= (rp.da    << 3);
  u ^= (rp.flags << 4);

  return((u & 0xFF) ^ (u >> 8));
}

bool RoboComms::makeRadioPacket(RoboComms::packet &rp, net_rcommand &cmd)
{
  // Make low level packet
  rp.start = 0x33;
  rp.robot = identifier[cmd.id];
  rp.stop  = 0x55;

  // scale to cm/sec for dx,dy, and diff cm/sec for da
  rp.dx = bound(abs((int)(cmd.dx/10.0)),0,255);
  rp.dy = bound(abs((int)(cmd.dy/10.0)),0,255);
  //  rp.da = bound(abs((int)(cmd.da/1000.0*DIFF_WHEELBASE_RADIUS)),0,255);
  rp.da = bound(abs((int)(cmd.da/1000.0 * 15.0)),0,255);

  //    rp.flags = (cmd.mode == RF_MODE_RUN)? 0 : RF_HALT;
  rp.flags = (sleeping || cmd.mode == RF_MODE_SLEEP) ? RF_HALT : 0;
  if(cmd.dx < 0) 
    rp.flags |= RF_REVERSE_DX;
  if(cmd.dy < 0) 
    rp.flags |= RF_REVERSE_DY;
  if(cmd.da < 0) 
    rp.flags |= RF_REVERSE_DA;

  if (cmd.kick) {
    rp.flags |= RF_KICK;
  }

  if (cmd.drib) {
    rp.flags |= RF_DRIBBLE;
  }

  rp.checksum = checksum(rp);

  //  printf("mrp(%3d,%3d,%3d) k %d d %d\n",rp.dx,rp.dy,rp.da, 
  //	 cmd.kick, cmd.drib);

  return(true);
}

bool RoboComms::verifyPacket(RoboComms::packet &rp)
{
  int i;

  // check for valid stop
  if(rp.stop != 0x55) return(false);

  // check for valid id
  i = 0;
  while ((i < MAX_ROBOTS) && (identifier[i] != rp.robot))
    i++;
  if (i >= MAX_ROBOTS) 
    return(false);

  // check for valid flags
  if (rp.flags & ~RF_ALL) 
    return(false);

  /*
  // check if sleep flag is consistent
  if(rp.flags&RF_SLEEP){
    if(rp.flags&~RF_SLEEP) return(false);
    if(rp.dx!=0 || rp.dy!=0 || rp.da!=0) return(false);
  }
  */

  return(true);
}

bool RoboComms::init()
{
  memset(cmd,0,sizeof(cmd));
  memset(cmd_time,0,sizeof(cmd_time));
  current_id = 0;

  // init serial port
  if(!ser.open(SERIAL_DEVICE,BAUD_RATE)){
    printf("ROBOCOMM ERROR: Could not open serial port!\n");
    return(false);
  }

  bytes_sent = 0;
  one_bits_sent = 0;

  return(true);
}

void RoboComms::close()
{
  ser.close();
}

bool RoboComms::verifyCommand(net_rcommand &cmd)
{
  if (cmd.mode >= RF_NUM_MODES) 
    return(false);
  if (cmd.id >= MAX_ROBOTS) 
    return(false);
  if ((cmd.type == ROBOT_TYPE_NONE) || (cmd.type >= NUM_ROBOT_TYPES)) 
    return(false);
  return(true);
}

bool RoboComms::setCommand(net_rcommand &ncmd,double time)
{
  int id = ncmd.id;

  if(!verifyCommand(ncmd)) return(false);

#ifdef DEBUG
  printf("cmd: %d (%d,%d,%d)\n",ncmd.id,ncmd.dx,ncmd.dy,ncmd.da);
#endif

  if(ncmd.priority>=cmd[id].priority ||
     (time - cmd_time[id])>ROBOT_PRIORITY_TIMEOUT){
    cmd[ncmd.id] = ncmd;
    cmd_time[ncmd.id] = time;
    return(true);
  }else{
    return(false);
  }
}

int RoboComms::setCommands(net_rcommand *ncmd,int num)
{
  double t;
  int i,n;


  t = gettime();
  n = 0;

  for(i=0; i<num; i++){
    n += setCommand(ncmd[i],t);
  }

  return(n);
}

int RoboComms::setCommand(char rid, double vx, double vy, double va,
		 bool kick = false, bool drib = false)
{
  net_rcommand ncmd;

  ncmd.mode     = RF_MODE_RUN;
  ncmd.priority = 1;
  ncmd.id       = rid;
  ncmd.type     = ROBOT_TYPE_ID(rid);
  ncmd.dx       = (int) vx;
  ncmd.dy       = (int) vy;
  ncmd.da       = (int) (va*1000); // convert to mrad/sec
  ncmd.kick     = kick;
  ncmd.drib     = drib;

  cmd[rid] = ncmd;
  //  cmd_new[rid] = true;

  return (setCommand(ncmd, gettime()));
}


bool RoboComms::run()
{
  packet rp[2];
  double t;
  int id;
  bool send;

  // memset(rp,0,sizeof(packet)*2);
  memchecker(rp,sizeof(packet)*2);
  t = gettime();
  id = current_id;

  do {
    id = (id + 1) % MAX_ROBOTS;
    send = ((t - cmd_time[id]) < ROBOT_TIMEOUT);
  } while(!send && (id != current_id));

  // printf("id=%d send=%d\n",id,send);
  // if(send) printf("t=%f\n",(t - cmd_time[id]));

  if(send) {
    makeRadioPacket(rp[0],cmd[id]);
    int n;
    n = ser.write(rp,sizeof(packet) + 4);
    if(print_packet_sent){
      printf("packet sent (r%d).\n",id);
    }

    bytes_sent += n;
    one_bits_sent += count_bits((unsigned char*)&rp,n);

    if(print_serial_rate && ((bytes_sent+n)%5000 < bytes_sent%5000)){
      printf(" %db/%dB %f\n",one_bits_sent,bytes_sent,
	     (double)one_bits_sent/(8*bytes_sent));
    }
    //    printf("wrote %i bytes\n", n);
    // printf("."); fflush(stdout);
  }

  current_id = id;
  return(send);
}
