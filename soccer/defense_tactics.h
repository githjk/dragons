// defense_tactics.h
// 
// Defense tactics.
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

#ifndef __defense_tactics_h__
#define __defense_tactics_h__

#include "world.h"
#include "tactic.h"

class TDefendLine : public RobotTactic {
public:
  virtual const char *name() const { return "defend_line"; }

private:
  TCoordinate p[2];
  double distmin, distmax;
  
  bool intercepting;

public:
  TDefendLine(TCoordinate p1, TCoordinate p2, 
	      double _distmin, double _distmax);

  static Tactic *parser(const char *param_string);
  virtual Tactic *clone() const { return new TDefendLine(*this); }

  virtual void command(World &world, int me, Robot::RobotCommand &command,
		       bool debug);
  
  virtual Status isDone(World &world, int me) {
    return intercepting ? Busy : the_status; }
};

class TDefendPoint : public RobotTactic {
public:
  virtual const char *name() const { return "defend_point"; }

private:
  TCoordinate center;
  double distmin, distmax;

  bool intercepting;

public:
  TDefendPoint(TCoordinate _center, double _distmin, double _distmax);

  static Tactic *parser(const char *param_string);
  virtual Tactic *clone() const { return new TDefendPoint(*this); }

  virtual void command(World &world, int me, Robot::RobotCommand &command,
		       bool debug);

  virtual Status isDone(World &world, int me) {
    return intercepting ? Busy : the_status; }
};

class TDefendLane : public RobotTactic {
public:
  virtual const char *name() const { return "defend_lane"; }

private:
  TCoordinate p[2];

  bool intercepting;

public:
  TDefendLane(TCoordinate _p1, TCoordinate _p2);

  static Tactic *parser(const char *param_string);
  virtual Tactic *clone() const { return new TDefendLane(*this); }

  virtual void command(World &world, int me, Robot::RobotCommand &command,
		       bool debug);

  virtual Status isDone(World &world, int me) {
    return intercepting ? Busy : the_status; }
};

class TBlock : public RobotTactic {
public:
  virtual const char *name() const { return "block"; }

  bool intercepting;

private:
  double distmin, distmax;
  int prefside;

  vector2d pref_point;
  bool pref_point_set;

public:
  TBlock(double _distmin, double _distmax, int _prefside);

  static Tactic *parser(const char *param_string);
  virtual Tactic *clone() const { return new TBlock(*this); }

  virtual void command(World &world, int me, Robot::RobotCommand &command,
		       bool debug);

  virtual Status isDone(World &world, int me) {
    Robot::RobotCommand c;
    command(world, me, c, false);
    return intercepting ? Busy : the_status; }
};

class TMark : public RobotTactic {
public:
  virtual const char *name() const { return "mark"; }

public:
  enum Type { FromBall, FromOurGoal, FromTheirGoal, FromShot };

private:
  int target;
  Type type;

public:
  TMark(int _target, Type _type);

  static Tactic *parser(const char *param_string);

  virtual Tactic *clone() const { return new TMark(*this); }

  virtual void command(World &world, int me, Robot::RobotCommand &command,
		       bool debug);
};
   
#endif
