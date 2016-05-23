// special_tactics.h
// 
// Very basic tactics.
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

#ifndef __special_tactics_h__
#define __special_tactics_h__

#include "world.h"
#include "tactic.h"
#include "simple_tactics.h"

class TPositionForStart : public TPosition {
public:
  TPositionForStart(TCoordinate _position, TCoordinate _faceto) :
    TPosition(_position, _faceto) { active = true; }

  static Tactic *parser(const char *param_string);
    
  virtual Tactic *clone() const { return new TPositionForStart(*this); }
  virtual Status isDone(World &world, int me);
};

class TPositionForKick : public RobotTactic {
private:
  vector2d prev_target;
  bool prev_target_set;

public:
  TPositionForKick();

  static Tactic *parser(const char *param_string) {
    return new TPositionForKick(); }

  virtual Tactic *clone() const { return new TPositionForKick(*this); }
  virtual Status isDone(World &world, int me) {
    return (world.game_state == 's') ? Succeeded : InProgress;
  }

  virtual int selectRobot(World &world, bool candidates[], double bias[]) {
    int best_i = -1;
    double best = 0;

    for(uint i=0; i<MAX_TEAM_ROBOTS; i++) {
      if (!candidates[i]) 
	continue;
      if (bias[i] < 0.0) 
	return i;
      double d = (world.teammate_position(i) - world.ball_position()).sqlength();
      if ((best_i < 0) || (d < best)) { 
	best = d; 
	best_i = i; 
      }
    }

    return best_i;
  }

  virtual void command(World &world, int me, Robot::RobotCommand &command,
		       bool debug);
};


// position the non-kicking players behind the line when a penalty occurs
class TPositionForPenalty : public RobotTactic {
public:
  TPositionForPenalty();
  
  static Tactic *parser(const char *param_string) {
    return new TPositionForPenalty(); }
  
  virtual Tactic *clone() const { return new TPositionForPenalty(*this); }
  virtual Status isDone(World &world, int me) {
    return (world.game_state == COMM_START) ? Succeeded : InProgress;
  }
  virtual void command(World &world, int me, Robot::RobotCommand &command,
		       bool debug);
};



class TChargeBall : public RobotTactic {
public:
  TChargeBall() { }

  static Tactic *parser(const char *param_string) {
    return new TChargeBall(); }
  virtual Tactic *clone() const { return new TChargeBall(*this); }

  virtual void command(World &world, int me, Robot::RobotCommand &command,
		       bool debug);
};

class TSuccess : public Tactic {
private:
  double start_time;
  double wait;

public:
  TSuccess(double _wait = 0) { wait = _wait; start_time = -1; }

  static Tactic *parser(const char *param_string) {
    return new TSuccess(); }
  virtual Tactic *clone() const { return new TSuccess(*this); }

  virtual Status isDone(World &world, int me) {
    if (world.time - start_time > wait) return Succeeded;
    else return InProgress; }
  virtual void run(World &world, int me) {
    if (start_time < 0) start_time = world.time;
    world.halt(me); }
};

class TComplete : public Tactic {
private:
  double start_time;
  double wait;

public:
  TComplete(double _wait = 0) { wait = _wait; start_time = -1; }

  static Tactic *parser(const char *param_string) {
    return new TComplete(); }
  virtual Tactic *clone() const { return new TComplete(*this); }

  virtual Status isDone(World &world, int me) { 
    if (world.time - start_time > wait) return Completed; 
    else return InProgress; }
  virtual void run(World &world, int me) { 
    if (start_time < 0) start_time = world.time;
    world.halt(me); }
};

#endif
