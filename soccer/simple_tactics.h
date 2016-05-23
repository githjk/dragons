// simple_tactics.h
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

#ifndef __simple_tactics_h__
#define __simple_tactics_h__

#include "world.h"
#include "tactic.h"

class TStop : public Tactic {
public:
  TStop() : Tactic(false) { }

  static Tactic *parser(const char *param_string);
  virtual Tactic *clone() const { return new TStop(*this); }

  virtual void run(World &world, int me);
};

class TVelocity : public Tactic {
  double vx, vy, va;

public:
  TVelocity(double _vx, double _vy, double _va) : Tactic(false) {
    vx = _vx; vy = _vy; va = _va; }

  static Tactic *parser(const char *param_string);
  virtual Tactic *clone() const { return new TVelocity(*this); }
  
  virtual void run(World &world, int me);
};


class TPosition : public RobotTactic {
public:
  TCoordinate position;
  TCoordinate faceto;
  int obs;
  bool use_obsflags;

  TPosition(TCoordinate _position, TCoordinate _faceto) : RobotTactic(false) {
    position = _position; faceto = _faceto; use_obsflags = false; }
  TPosition(TCoordinate _position, double angle) : RobotTactic(false) {
    position = _position; faceto = vector2d(1, 0).rotate(angle); use_obsflags = false;}

  TPosition(TCoordinate _position, int _obs) : RobotTactic(false) {
    position = _position; faceto = vector2d(1, 0); 
    obs = _obs; use_obsflags = true; }

  static Tactic *parser(const char *param_string);
  virtual Tactic *clone() const { return new TPosition(*this); }

  virtual void command(World &world, int me, Robot::RobotCommand &command,
		       bool debug);
};


class TDribbleToPosition : public RobotTactic {
public:
  TCoordinate position;
  TCoordinate faceto;

  TDribbleToPosition(TCoordinate _position, TCoordinate _faceto) 
    : RobotTactic(false) {
    position = _position; faceto = _faceto; }
  TDribbleToPosition(TCoordinate _position, double angle) 
    : RobotTactic(false) {
    position = _position; faceto = vector2d(1, 0).rotate(angle); }

  static Tactic *parser(const char *param_string);
  virtual Tactic *clone() const { return new TDribbleToPosition(*this); }

  virtual void command(World &world, int me, Robot::RobotCommand &command,
		       bool debug);
};

#endif
