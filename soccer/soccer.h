// soccer.cc
//
// Defines global functions for all of the soccer code.
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

#ifndef __soccer_h__
#define __soccer_h__

#include "net_gui.h"

enum Status { Failed, Aborted, InProgress, Completed, Succeeded, Busy};

inline const char *status_as_string(Status s) {
  switch(s) {
  case Failed: return "Failed";
  case Aborted: return "Aborted";
  case InProgress: return "InProgress";
  case Completed: return "Completed";
  case Succeeded: return "Succeeded";
  case Busy: return "Busy";
  }

  return "";
}

void gui_debug(const net_gdebug &g);
void gui_debug_line(const char robot, const char level, 
		    vector2d p1, vector2d p2, char flags = 0);
void gui_debug_arc(const char robot, const char level,
		   vector2d p1, vector2d dimens,
		   double start_angle, double stop_angle, 
		   char flags = 0);
void gui_debug_x(const char robot, const char level, vector2d p);
void gui_debug_printf(const char robot, const char level,
		      const char *fmt, ...);


void radio_send(const char robot, double vx, double vy, double va,
		bool kicker_on = false, bool dribbler_on = false);
void radio_halt(const char robot);

#endif
