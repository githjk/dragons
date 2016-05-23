// special_tactics.cc
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
#include "special_tactics.h"
#include "evaluation.h"

CR_DECLARE(SHOOT_AIM_PREF_AMOUNT);
CR_DECLARE(DISTANCE_FROM_PENALTY_LINE);


inline void cr_do_setup()
{
  static bool cr_setup = false;

  if (!cr_setup) {
    CR_SETUP(tactic, SHOOT_AIM_PREF_AMOUNT, CR_DOUBLE);
    CR_SETUP(tactic, DISTANCE_FROM_PENALTY_LINE, CR_DOUBLE);
    cr_setup = true;
  }
}

// TPositionForStart
//
// Sends the robot to a position on the field.
//

bool tpositionForStart_registered = 
Tactic::registerParser("position_for_start", TPositionForStart::parser);

Tactic *TPositionForStart::parser(const char *param_string)
{
  TCoordinate c;
  TCoordinate f;

  param_string += Parse::pTCoordinate(param_string, c);
  param_string += Parse::pTCoordinateDir(param_string, f);

  return new TPositionForStart(c, f);
}

Status TPositionForStart::isDone(World &world, int me)
{
  return (world.game_state == 's') ? Succeeded : InProgress;
}

bool tposition_for_kick_registered = 
Tactic::registerParser("position_for_kick", TPositionForKick::parser);

TPositionForKick::TPositionForKick()
{
  cr_do_setup();
}

void TPositionForKick::command(World &world, int me, 
			       Robot::RobotCommand &command,
			       bool debug)
{
  vector2d ball = world.ball_position();

  vector2d target;
  double angle_tolerance;

  if (!prev_target_set) {
    prev_target = world.their_goal; prev_target_set = true; 
  }

  // (1) Try shooting on goal.
  if (!evaluation.aim(world, world.now, world.ball_position(),
		      world.their_goal_r, 
		      world.their_goal_l,
		      OBS_EVERYTHING_BUT_US,
		      prev_target, DVAR(SHOOT_AIM_PREF_AMOUNT),
		      target, angle_tolerance)) {
    vector2d downfield[2];
  
    downfield[0].set(ball.x + 180.0, -FIELD_WIDTH_H);
    downfield[1].set(ball.x + 180.0, FIELD_WIDTH_H);

    // (2) Try clearing the ball
    if (!evaluation.aim(world, world.now, world.ball_position(),
			downfield[0], downfield[1],
			OBS_EVERYTHING_BUT_ME(me),
			prev_target, DVAR(SHOOT_AIM_PREF_AMOUNT),
			target, angle_tolerance)) {
      // Guaranteed to return true and fill in the parameters when 
      // obs_flags is empty.
      // (3) Just shoot downfield.
      evaluation.aim(world, world.now, world.ball_position(),
		     downfield[0], downfield[1],
		   0, target, angle_tolerance);
    }
  }

  if (debug) {
    gui_debug_line(me, GDBG_TACTICS, ball, target);
    gui_debug_line(me, GDBG_TACTICS, ball,
		   (target - ball).rotate(angle_tolerance) + ball);
    gui_debug_line(me, GDBG_TACTICS, ball,
		   (target - ball).rotate(-angle_tolerance) + ball);

  }
    
  prev_target = target;

  double balldist = (world.teammate_position(me) - ball).length();

  if (world.teammate_velocity(me).length() < 20.0)
    balldist -= 20.0;

  // put this in config
  double closest = 85.0;

  balldist = bound(balldist, closest, 150.0);

  vector2d targetp = ball - (target - ball).norm(balldist);
  double angle = (ball - targetp).angle();
  int obs = OBS_EVERYTHING_BUT_ME(me);

  vector2d r2target = (targetp - world.teammate_position(me));
  double d2target = r2target.sqlength();
  if ((d2target < 150.0 * 150.0) && (d2target > 20.0 * 20.0) &&
      (fabs(angle_mod(angle - r2target.angle())) < M_PI_4)) {
    //    obs = OBS_WALLS;
    obs = 0;

    //    printf("turning off obstacle avoidance!!!\n");
    
    if (debug)
      gui_debug_printf(me, GDBG_TACTICS, "turning off obstacle avoidance!!!\n");
  
  }

  command.cmd = Robot::CmdPosition;
  command.target = targetp;
  command.velocity = vector2d(0, 0);
  command.angle = angle;
  //  command.obs = OBS_EVERYTHING_BUT_ME(me);
  command.obs = obs;
  command.goto_point_type = Robot::GotoPointMoveForw;
}

////////////////// Penalty ////////////////////

TPositionForPenalty::TPositionForPenalty(void)
{
  cr_do_setup();
}

bool tposition_for_penalty_registered = 
Tactic::registerParser("position_for_penalty", TPositionForPenalty::parser);


void TPositionForPenalty::command(World &world, int me, 
			       Robot::RobotCommand &command,
			       bool debug)
{
  vector2d mypos = world.teammate_position(me);

  vector2d target;

  double linex;
  double penalty_goal_dir;

  // figure out which penalty is going
  if (world.restartWhoseKick() == World::OurBall) {
    linex = PENALTY_SPOT - (DVAR(DISTANCE_FROM_PENALTY_LINE) 
			    + ROBOT_DEF_WIDTH_H);
    penalty_goal_dir = 1.0;
  } else {
    linex = -PENALTY_SPOT + (DVAR(DISTANCE_FROM_PENALTY_LINE) 
			     + ROBOT_DEF_WIDTH_H);
    penalty_goal_dir = -1.0;
  }

  if (debug) {
    gui_debug_line(me, GDBG_TACTICS, vector2d(linex, -100), vector2d(linex, 100));
  }


  if ((linex - mypos.x) * penalty_goal_dir > 0) {
    // need to halt here
    gui_debug_printf(me, GDBG_TACTICS, "we are in position\n");

    command.cmd = Robot::CmdPosition;
    command.target = mypos;
    command.velocity = vector2d(0, 0);
    command.angle = 0; //((penalty_goal_dir > 0) ? M_PI : 0.0);
    command.obs = OBS_EVERYTHING_BUT_ME(me);
    command.goto_point_type = Robot::GotoPointMove;

  } else {

    gui_debug_printf(me, GDBG_TACTICS, "we need to move\n");

    // choose our target point
    target.set(linex, mypos.y);

    if (debug) {
      gui_debug_line(me, GDBG_TACTICS, mypos, target);
    }
    
    command.cmd = Robot::CmdPosition;
    command.target = target;
    command.velocity = vector2d(0, 0);
    command.angle = 0; //((penalty_goal_dir > 0) ? M_PI : 0.0);
    command.obs = OBS_EVERYTHING_BUT_ME(me);
    command.goto_point_type = Robot::GotoPointMove;
  }
}


bool tcharge_ball_registered = 
Tactic::registerParser("charge_ball", TChargeBall::parser);

void TChargeBall::command(World &world, int me, Robot::RobotCommand &command,
			  bool debug)
{
  vector2d ball = world.ball_position();
  vector2d mypos = world.teammate_position(me);

  command.cmd = Robot::CmdMoveBall;

  if ((ball - mypos).x > 0) {
    command.target = ball + (ball - mypos).norm(500);
    command.angle_tolerance = M_PI_2 - fabs((ball - mypos).angle());
  } else {
    command.target = ball + vector2d(500, 0);
    command.angle_tolerance = M_PI_4;
  }

  command.ball_target = command.target;
  command.ball_shot_type = Robot::BallShotClear;
}

bool tsuccess_registered = Tactic::registerParser("success", TSuccess::parser);
bool tcomplete_registered = Tactic::registerParser("complete", TComplete::parser);

