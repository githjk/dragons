// goalie.h
// 
// Goalie tactic.
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

#ifndef __goalie_h__
#define __goalie_h__

#include "world.h"
#include "robot.h"
#include "tactic.h"

class TGoalie : public RobotTactic {
public:
  double penalty_tend;
  bool penalty_play;

  TGoalie(void);

  static Tactic *parser(const char *param_string);
  virtual Tactic *clone() const { return new TGoalie(*this); }

  virtual void command(World &world, int me, Robot::RobotCommand &command,
		       bool debug);
  bool penaltyCharge(World &world, int me, Robot::RobotCommand &command,
		     bool debug);
  void penaltyPlay(World &world, int me, Robot::RobotCommand &command,
		   bool debug);

};



#endif
