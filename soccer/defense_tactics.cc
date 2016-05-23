//
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

#include <configreader.h>

#include "parse.h"
#include "evaluation.h"
#include "defense_tactics.h"

CR_DECLARE(DEFENSE_OFF_BALL);
CR_DECLARE(MARK_OFF_OPPONENT);

inline static void cr_do_setup()
{
  static bool cr_setup = false;

  if (!cr_setup) {
    CR_SETUP(tactic, DEFENSE_OFF_BALL, CR_DOUBLE);
    CR_SETUP(tactic, MARK_OFF_OPPONENT, CR_DOUBLE);

    cr_setup = true;
  } 
}

bool tdefend_line_registered = 
Tactic::registerParser("defend_line", TDefendLine::parser);

TDefendLine::TDefendLine(TCoordinate p1, TCoordinate p2, 
			 double _distmin, double _distmax) 
  :  RobotTactic(false) 
{
  p[0] = p1; p[1] = p2; distmin = _distmin; distmax = _distmax; 
  cr_do_setup();
}

void TDefendLine::command(World &world, int me, Robot::RobotCommand &command,
			  bool debug)
{
  intercepting = true;

  vector2d ball = world.ball_position(me);

  vector2d target, velocity;
  double angle;

  vector2d v[2] = { p[0].asVector(world), p[1].asVector(world) };

  // Position
  if (!evaluation.defend_line(world, world.now, v[0], v[1],
			      distmin, distmax, DVAR(DEFENSE_OFF_BALL), 
			      intercepting, target, velocity)) {
    if (debug) {
      gui_debug_printf(me, GDBG_TACTICS, 
        "DefendLine: WARNING evaluation.defend_line() returned false.");
    }
    target = ball;
    velocity = vector2d(0, 0);
  }
  
  // Obstacles... don't avoid the ball if away from the line.
  int obs_flags = OBS_EVERYTHING_BUT_ME(me);

  vector2d mypos = world.teammate_position(me);

  if (distance_to_line(v[0], v[1], mypos) <
      distance_to_line(v[0], v[1], ball))
    obs_flags &= ~OBS_BALL;

  if (debug) 
    gui_debug_printf(me, GDBG_TACTICS, "Intercepting = %d\n", intercepting);

  target = evaluation.findOpenPositionAndYield(world, target, 
					       (v[0] + v[1]) / 2.0,
					       getObsFlags() | OBS_OPPONENTS);

  // Angle
  if (world.teammate_type(me) == ROBOT_TYPE_DIFF)
    angle = world.teammate_nearest_direction(me, (v[0] - v[1]).angle());
  else
    angle = (ball - target).angle();

  // Debug output
  if (debug) {
    gui_debug_line(me, GDBG_TACTICS, v[0], target); 
    gui_debug_line(me, GDBG_TACTICS, v[1], target); 
    gui_debug_line(me, GDBG_TACTICS, 
		   world.teammate_position(me), target, G_ARROW_FORW);
    gui_debug_line(me, GDBG_TACTICS, 
		   target, target + velocity, G_ARROW_FORW);
  }
  
  command.cmd = Robot::CmdPosition;
  command.target = target;
  command.velocity = velocity;
  command.angle = angle;
  command.obs = obs_flags;
}

Tactic *TDefendLine::parser(const char *param_string)
{
  TCoordinate c[2];
  double mind, maxd;

  param_string += Parse::pTCoordinate(param_string, c[0]);
  param_string += Parse::pTCoordinate(param_string, c[1]);
  param_string += Parse::pDouble(param_string, mind);
  param_string += Parse::pDouble(param_string, maxd);

  return new TDefendLine(c[0], c[1], mind, maxd);
}

bool tdefend_point_registered = 
Tactic::registerParser("defend_point", TDefendPoint::parser);

TDefendPoint::TDefendPoint(TCoordinate _center, 
			   double _distmin, double _distmax) 
    : RobotTactic(false) 
{
  center = _center; distmin = _distmin; distmax = _distmax; 
  cr_do_setup();
}

void TDefendPoint::command(World &world, int me, Robot::RobotCommand &command,
			   bool debug)
{
  intercepting = true;

  vector2d ball = world.ball_position(me);
  vector2d centerv = center.asVector(world);

  vector2d target, velocity;
  double angle;

  // Position
  if (!evaluation.defend_point(world, world.now, centerv, 
			       distmin, distmax, DVAR(DEFENSE_OFF_BALL),
			       intercepting, target, velocity)) {
    if (debug) {
      gui_debug_printf(me, GDBG_TACTICS, 
        "DefendCircle: WARNING evaluation.defend_line() returned false.");
    }
    target = ball;
    velocity = vector2d(0, 0);
  }
  
  // Angle
  angle = (target - centerv).angle();
  
  if (world.teammate_type(me) == ROBOT_TYPE_DIFF)
    angle = world.teammate_nearest_direction(me, anglemod(angle + M_PI_2));
  
  // Obstacles... don't avoid the ball if away from the point.
  int obs_flags = OBS_EVERYTHING_BUT_ME(me);

  vector2d mypos = world.teammate_position(me);

  if ((mypos - centerv).dot(ball - mypos) > 0)
    obs_flags &= ~OBS_BALL;

  target = evaluation.findOpenPositionAndYield(world, target, centerv,
					       getObsFlags() | OBS_OPPONENTS);

  // Debug output
  double r = (target - centerv).length();
  if (debug) {
    gui_debug_arc(me, GDBG_TACTICS, centerv,
		  vector2d(2 * r, 2 * r), 0.0, M_2PI);
    gui_debug_line(me, GDBG_TACTICS, 
		   world.teammate_position(me), target, G_ARROW_FORW);
  }

  command.cmd = Robot::CmdPosition;
  command.target = target;
  command.velocity = velocity;
  command.angle = angle;
  command.obs = obs_flags;
}

Tactic *TDefendPoint::parser(const char *param_string)
{
  TCoordinate center;
  double mind, maxd;

  param_string += Parse::pTCoordinate(param_string, center);
  param_string += Parse::pDouble(param_string, mind);
  param_string += Parse::pDouble(param_string, maxd);

  return new TDefendPoint(center, mind, maxd);
}

bool tdefend_lane_registered = 
Tactic::registerParser("defend_lane", TDefendLane::parser);

TDefendLane::TDefendLane(TCoordinate _p1, TCoordinate _p2) 
  : RobotTactic(false) 
{
  p[0] = _p1, p[1] = _p2; 
  cr_do_setup();
}

void TDefendLane::command(World &world, int me, Robot::RobotCommand &command,
			  bool debug)
{
  intercepting = true;

  vector2d target, velocity;
  double angle;

  vector2d v0 = p[0].asVector(world);
  vector2d v1 = p[1].asVector(world);

  evaluation.defend_on_line(world, world.now, v0, v1,
			    intercepting, target, velocity);

  if (world.teammate_type(me) == ROBOT_TYPE_DIFF) {
    angle = (v0 - v1).angle();
    angle = world.teammate_nearest_direction(me, angle);
  } else angle = (world.ball_position() - target).angle();

  vector2d opt0, opt1;
  opt0 = evaluation.findOpenPositionAndYield(world, target, v0,
					     getObsFlags() | OBS_OPPONENTS);
  opt1 = evaluation.findOpenPositionAndYield(world, target, v1,
					     getObsFlags() | OBS_OPPONENTS);
  target = ((target - opt0).length() < (target - opt1).length()) ? opt0 : opt1;

  if (debug) {
    gui_debug_line(me, GDBG_TACTICS, v0, v1);
    gui_debug_line(me, GDBG_TACTICS, 
		   world.teammate_position(me), target, G_ARROW_FORW);
  }

  command.cmd = Robot::CmdPosition;
  command.target = target;
  command.velocity = velocity;
  command.angle = angle;
  command.obs = OBS_EVERYTHING_BUT_ME(me) & ~OBS_BALL;
}

Tactic *TDefendLane::parser(const char *param_string)
{
  TCoordinate p1, p2;

  param_string += Parse::pTCoordinate(param_string, p1);
  param_string += Parse::pTCoordinate(param_string, p2);

  return new TDefendLane(p1, p2);
}

bool tblock_registered = 
Tactic::registerParser("block", TBlock::parser);

TBlock::TBlock(double _distmin, double _distmax, int _prefside)
  : RobotTactic(false) 
{
  distmin = _distmin; distmax = _distmax; prefside = _prefside; 
  cr_do_setup();
}

void TBlock::command(World &world, int me, Robot::RobotCommand &command,
		     bool debug) 
{
  intercepting = true;

  vector2d ball = world.ball_position(me);
  vector2d mypos = world.teammate_position(me);
  vector2d v[2] = { world.our_goal_l, world.our_goal_r };
  double mydist = distance_to_line(v[0], v[1], mypos);

  vector2d target, velocity;
  double angle;

  vector2d pref_point;

  // Side preference
  if (prefside != 0) 
    pref_point = vector2d(world.our_goal_l.x, 
			  world.our_goal_l.y * prefside * world.sideBall());
  else if (!pref_point_set) pref_point = world.our_goal;

  // Take into account teammates behind us.
  int obs_flags = 0;
  for(int i=0; i < world.n_teammates; i++) {
    if (i == me) continue;
    if (distance_to_line(v[0], v[1], world.teammate_position(i)) < mydist)
      obs_flags |= OBS_TEAMMATE(i);
  }

  // Position
  if (!evaluation.defend_line(world, world.now, v[0], v[1],
			      distmin, distmax, DVAR(DEFENSE_OFF_BALL),
			      intercepting, obs_flags, pref_point, 0.1221,
			      target, velocity) &&
      !evaluation.defend_line(world, world.now, v[0], v[1],
			      distmin, distmax, DVAR(DEFENSE_OFF_BALL),
			      intercepting, target, velocity)) {
    if (debug) 
      gui_debug_printf(me, GDBG_TACTICS, 
		       "Block: WARNING evaluation.defend_line() returned false.");

    target = mypos;
    velocity = vector2d(0, 0);
			   
  } else {
    pref_point = target;
    pref_point_set = true;
  }

  if (debug) 
    gui_debug_printf(me, GDBG_TACTICS, "Intercepting = %d\n", intercepting);

  target = evaluation.findOpenPositionAndYield(world, target, world.our_goal, 
					       getObsFlags() | OBS_OPPONENTS);
  
  // Obstacles... don't avoid the ball if away from the line.
  obs_flags = OBS_EVERYTHING_BUT_ME(me);

  if (distance_to_line(v[0], v[1], mypos) <
      distance_to_line(v[0], v[1], ball))
    obs_flags &= ~OBS_BALL;

  // Angle
  if (world.teammate_type(me) == ROBOT_TYPE_DIFF)
    angle = world.teammate_nearest_direction(me, (v[0] - v[1]).angle());
  else
    angle = (ball - target).angle();

  command.cmd = Robot::CmdPosition;
  command.target = target;
  command.velocity = velocity;
  command.angle = angle;
  command.obs = obs_flags;
}

Tactic *TBlock::parser(const char *param_string)
{
  double mindist, maxdist;
  int prefside;

  param_string += Parse::pDouble(param_string, mindist);
  param_string += Parse::pDouble(param_string, maxdist);
  param_string += Parse::pInt(param_string, prefside);

  return new TBlock(mindist, maxdist, prefside);
}

bool tmark_registered = 
Tactic::registerParser("mark", TMark::parser);

TMark::TMark(int _target, Type _type) { 
  target = _target; 
  type = _type;

  cr_do_setup();
}

Tactic *TMark::parser(const char *param_string)
{
  int target;
  Type type;
  
  char *word;
  
  param_string += Parse::pInt(param_string, target);
  param_string += Parse::skipWS(param_string, " \t");
  
  if (param_string[0] != '\n') {
    param_string += Parse::pWord(param_string, &word);
    
    if (strcmp(word, "ball") == 0) type = FromBall;
    else if (strcmp(word, "our_goal") == 0) type = FromOurGoal;
    else if (strcmp(word, "their_goal") == 0) type = FromTheirGoal;
    else if (strcmp(word, "shot") == 0) type = FromShot;
    else type = FromBall;
    
    if (word) delete [] word;
  } else type = FromBall;
  
  return new TMark(target, type); 
}

void TMark::command(World &world, int me, Robot::RobotCommand &command,
		    bool debug) 
{
  int targeti = getOpponentId(target);
  vector2d targetp = world.opponent_position(targeti);
  vector2d targetv = world.opponent_velocity(targeti);

  vector2d mark_from;

  switch(type) {
  case FromOurGoal:
    mark_from = world.our_goal; break;
  case FromTheirGoal:
    mark_from = world.their_goal; break;
  case FromBall:
    mark_from = world.ball_position(); break;
  case FromShot: {
    mark_from = point_on_segment(world.ball_position(), world.their_goal,
				 targetp);
    if (world.obsPosition(mark_from, OBS_OPPONENT(targeti)))
      mark_from = vector2d(targetp.x, copysign(FIELD_WIDTH_H, -targetp.y));
  }
  }

  targetp += (mark_from - targetp).norm(DVAR(MARK_OFF_OPPONENT));

  targetp = evaluation.findOpenPositionAndYield(world, targetp, 
						mark_from,
						getObsFlags() | OBS_OPPONENTS);

  targetv = targetv.rotate(-(mark_from - targetp).angle());
  if (targetv.x > 0.0) targetv.x = 0.0;
  targetv = targetv.rotate((mark_from - targetp).angle());

  if (targetv.length() < 80.0) targetv = vector2d(0, 0);

  double angle = (targetp - mark_from).angle();
  if (world.teammate_type(me) == ROBOT_TYPE_DIFF)
    angle = world.teammate_nearest_direction(me, angle + M_PI_2);

  int obs_flags = OBS_EVERYTHING_BUT_ME(me);

#if 0
  if ((mark_from - targetp).dot(world.teammate_position(me) - targetp) > 0.0)
    obs_flags &= ~OBS_OPPONENT(targeti);
#else
  if (point_on_segment(world.teammate_position(me), targetp, 
		       world.opponent_position(targeti)) == targetp)
    obs_flags &= ~OBS_OPPONENT(targeti);

#endif

  gui_debug_arc(me, GDBG_TACTICS, 
		world.opponent_position(targeti), 
		vector2d(200, 200), 0, M_2PI);

  command.cmd = Robot::CmdPosition;
  command.target = targetp;
  command.velocity = targetv;
  command.angle = angle;
  command.obs = obs_flags;
}


