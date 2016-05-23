// anticipation_tactics.cc
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

#include <stdio.h>
#include <configreader.h>

#include "parse.h"
#include "evaluation.h"
#include "anticipation_tactics.h"

bool tposition_for_loose_ball_registered = 
Tactic::registerParser("position_for_loose_ball", 
		       TPositionForLooseBall::parser);

Tactic *TPositionForLooseBall::parser(const char *param_string)
{
  TRegion r;

  param_string += Parse::pTRegion(param_string, r);

  return new TPositionForLooseBall(r);
}

void TPositionForLooseBall::command(World &world, int me, 
				    Robot::RobotCommand &command,
				    bool debug)
{
  vector2d ball = world.ball_position();
  vector2d p1, p2;

  region.diagonal(world, ball, p1, p2);
  
  vector2d target;
  double tolerance;
  double angle;

  int obs_flags = getObsFlags() | 
    OBS_OPPONENTS | OBS_OUR_DZONE | OBS_THEIR_DZONE;

  evaluation.aim(world, world.now, ball, p1, p2, 
		 obs_flags, target, tolerance);

  if (!world.obsPosition(target + vector2d(-ROBOT_DEF_WIDTH_H, 0), obs_flags))
    target -= 
      (world.their_goal - target).norm(2 * ROBOT_DEF_WIDTH_H + BALL_RADIUS);

  angle = ((world.their_goal - target).norm() + 
	   (ball - target).norm()).angle();

  command.cmd = Robot::CmdPosition;
  command.target = target;
  command.velocity = region.centerVelocity(world);
  command.angle = angle;
  command.obs = OBS_EVERYTHING_BUT_ME(me);
  
  if (debug) { 
    region.gui_draw(me, GDBG_TACTICS, world);
    gui_debug_line(me, GDBG_TACTICS, p1, p2);
  }
}

bool tposition_for_rebound_registered = 
Tactic::registerParser("position_for_rebound", TPositionForRebound::parser);

double TPositionForRebound::eval_fn(World &world, const vector2d p, 
				    int obs_flags, double &a)
{
  vector2d target;
  double tolerance;
  
  obs_flags |= OBS_OPPONENTS | OBS_OUR_DZONE | OBS_THEIR_DZONE;

  if (world.obsBlocksShot(p)) { a = 0.0; return -1; }
  
  if (world.obsPosition(p, obs_flags)) { a = 0.0; return -1; } 

  if (evaluation.aim(world, world.now, p, 
		     world.their_goal_r, world.their_goal_l, 
		     OBS_OPPONENTS, target, tolerance)) {
    a = (target - p).angle();

    return tolerance;
  } else return 0.0;
}

Tactic *TPositionForRebound::parser(const char *param_string)
{
  TRegion r;

  param_string += Parse::pTRegion(param_string, r);

  return new TPositionForRebound(r);
}


bool tposition_for_pass_registered = 
Tactic::registerParser("position_for_pass", TPositionForPass::parser);

double TPositionForPass::eval_fn(World &world, const vector2d p, 
				 int obs_flags, double &a)
{
  vector2d target;
  double pass_tolerance, tolerance;
  
  obs_flags |= OBS_OPPONENTS | OBS_OUR_DZONE | OBS_THEIR_DZONE;

  if (world.obsBlocksShot(p)) { a = 0.0; return -1; }
  
  if (world.obsPosition(p, obs_flags)) return -1;

  vector2d ball = world.ball_position();
  vector2d ball2p = (p - ball);

  a = (ball - p).angle();

  if (!evaluation.aim(world, world.now, ball, 
		      ball + ball2p + ball2p.perp().norm(30.0),
		      ball + ball2p + ball2p.perp().norm(-30.0), obs_flags,
		      target, pass_tolerance)) return -1;

  if (pass_tolerance < sin(25.0 / ball2p.length())) 
    return -1;
    
  if (evaluation.aim(world, world.now, p, 
		     world.their_goal_r, world.their_goal_l, 
		     obs_flags, target, tolerance)) {
    return pass_tolerance + tolerance;
  } else return 0.0;
}

Tactic *TPositionForPass::parser(const char *param_string)
{
  TRegion r;

  param_string += Parse::pTRegion(param_string, r);

  return new TPositionForPass(r);
}

bool tposition_for_deflection =
Tactic::registerParser("position_for_deflection", 
		       TPositionForDeflection::parser);

double TPositionForDeflection::eval_fn(World &world, const vector2d p, 
				       int obs_flags, double &a)
{
  vector2d target;
  double deflection_tolerance, tolerance;
  
  obs_flags |= OBS_OPPONENTS | OBS_OUR_DZONE | OBS_THEIR_DZONE;

  if (world.obsPosition(p, obs_flags)) { a = 0.0; return -1; }

  if (world.obsBlocksShot(p)) { a = 0.0; return -1; }
  
  vector2d ball = world.ball_position();
  vector2d ball2p = (p - ball);

  a = (ball - p).angle() / 2.0;

  if (!evaluation.aim(world, world.now, ball, 
		      ball + ball2p + ball2p.perp().norm(30.0),
		      ball + ball2p + ball2p.perp().norm(-30.0), obs_flags,
		      target, deflection_tolerance)) return -1;

  if (deflection_tolerance < sin(25.0 / ball2p.length())) 
    return -1;
    
  if (evaluation.aim(world, world.now, p, 
		     world.their_goal_r, world.their_goal_l, 
		     obs_flags, target, tolerance)) {
    a = ((ball - p) + (target - p)).angle();
    return deflection_tolerance + tolerance;
  } else return 0;
}

Tactic *TPositionForDeflection::parser(const char *param_string)
{
  TRegion r;

  param_string += Parse::pTRegion(param_string, r);

  return new TPositionForDeflection(r);
}
