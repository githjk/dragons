// ball_tactics.h
// 
// Ball tactics.
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

#include "soccer.h"
#include "parse.h"
#include "evaluation.h"
#include "ball_tactics.h"

CR_DECLARE(SHOOT_AIM_PREF_AMOUNT);
CR_DECLARE(SHOOT_MIN_ANGLE_TOLERANCE);
CR_DECLARE(SHOOT_DRIBBLE_IF_NO_SHOT);
CR_DECLARE(PASS_TARGET_WIDTH);

inline static void cr_do_setup()
{
  static bool cr_setup = false;

  if (!cr_setup) {
    CR_SETUP(tactic, SHOOT_AIM_PREF_AMOUNT, CR_DOUBLE);
    CR_SETUP(tactic, SHOOT_MIN_ANGLE_TOLERANCE, CR_DOUBLE);
    CR_SETUP(tactic, SHOOT_DRIBBLE_IF_NO_SHOT, CR_INT);
    CR_SETUP(tactic, PASS_TARGET_WIDTH, CR_DOUBLE);

    cr_setup = true;
  } 
}

bool tshoot_registered = 
Tactic::registerParser("shoot", TShoot::parser);

TShoot::TShoot(Type _type, int _deflect_target) : RobotTactic(true, true) 
{
  cr_do_setup();

  type = _type; 
  deflect_target = _deflect_target;

  prev_target_set = false; 
  eval.set(TRegion(TCoordinate(0, 0, 
			       TCoordinate::SBall, 
			       TCoordinate::OBall, true), 
		   400), &eval_fn, 0.2442);
  deflect_target = _deflect_target;
}

Tactic *TShoot::parser(const char *param_string)
{
  Type type;
  int target;
  char t;

  param_string += Parse::pChar(param_string, t);

  switch(t) {
  case 'N': type = NoAim; break;
  case 'D': type = Deflect; break;
  case 'A':
  default: type = Aim; break;
  }

  if (type == Deflect) Parse::pInt(param_string, target);

  return new TShoot(type, target);
}

void TShoot::command(World &world, int me, Robot::RobotCommand &command,
		     bool debug)
{
  vector2d ball = world.ball_position();

  vector2d target;
  double angle_tolerance;

  if (!prev_target_set) {
    prev_target = (ball - (world.teammate_position(me) - ball));
    prev_target_set = true; 
  }

  Type the_type = type;

  if (world.twoDefendersInTheirDZone())
    the_type = NoAim;

  bool got_target = false;

  if (the_type == Aim) {
    got_target = evaluation.aim(world, world.now, world.ball_position(),
				world.their_goal_r, 
				world.their_goal_l,
				OBS_EVERYTHING_BUT_ME(me),
				prev_target, DVAR(SHOOT_AIM_PREF_AMOUNT),
				target, angle_tolerance);
  }

  if (the_type == Deflect) {
    vector2d t = world.teammate_position(getTeammateId(deflect_target));
    vector2d toward = t - ball;
    vector2d towardperp = toward.perp();

    if (towardperp.x < 0) towardperp *= -1;

    got_target = evaluation.aim(world, world.now, world.ball_position(),
				t + towardperp.norm(80.0),
				t - towardperp.norm(40.0),
				OBS_EVERYTHING_BUT_US,
				prev_target, DVAR(SHOOT_AIM_PREF_AMOUNT),
				target, angle_tolerance);
    got_target = true;

    if ((target - ball).length() < 400.0)
      target = (ball + target) / 2.0;
    else 
      target = target + (ball - target).norm(200.0);
  }

  if (the_type == NoAim || (!got_target && !IVAR(SHOOT_DRIBBLE_IF_NO_SHOT))) {
    // Guaranteed to return true and fill in the parameters when
    // obs_flags is empty.
    evaluation.aim(world, world.now, world.ball_position(),
		   world.their_goal_r, world.their_goal_l, 0, 
		   target, angle_tolerance);
    got_target = true;
  }

  if (got_target) {
    if (debug) {
      gui_debug_line(me, GDBG_TACTICS, ball, target);
      gui_debug_line(me, GDBG_TACTICS, ball,
		     (target - ball).rotate(angle_tolerance) + ball);
      gui_debug_line(me, GDBG_TACTICS, ball,
		     (target - ball).rotate(-angle_tolerance) + ball);
    }
    
    if (angle_tolerance < DVAR(SHOOT_MIN_ANGLE_TOLERANCE)) {
      angle_tolerance = DVAR(SHOOT_MIN_ANGLE_TOLERANCE);
    }
    
    prev_target = target;
    
    // Aim for point in front of obstacles.
    vector2d rtarget;
    vector2d targ_ball;

    targ_ball = target - ball;
    world.obsLineFirst(target-targ_ball.bound(500)*0.75, target,
		       OBS_OPPONENTS /*| OBS_THEIR_DZONE*/, rtarget);

    command.cmd = Robot::CmdMoveBall;
    command.ball_target = target;
    command.target = rtarget;
    command.angle_tolerance = angle_tolerance;
    command.ball_shot_type = Robot::BallShotOnGoal;
  } else {
    eval.update(world, getObsFlags());

    command.cmd = Robot::CmdDribble;
    command.target = eval.point();
    command.ball_target = eval.point();
    command.angle = eval.angle();

    if (debug) {
      eval.region.gui_draw(me, GDBG_TACTICS, world);
      gui_debug_x(me, GDBG_TACTICS, command.target);
    }
  }
}

double TShoot::successProb(World &world)
{
  Robot::RobotCommand c;

  command(world, 0, c, false);

  return bound(c.angle_tolerance / M_PI_16, 0, 1);
}

double TShoot::eval_fn(World &world, const vector2d p, 
		       int obs_flags, double &a)
{
  vector2d target;
  double tolerance;
  
  obs_flags |= OBS_OPPONENTS | OBS_OUR_DZONE | OBS_THEIR_DZONE | OBS_WALLS;
  if (world.obsPosition(p, obs_flags)) { return -1; } 

  obs_flags = OBS_OPPONENTS;
  for(int i=0; i<world.n_opponents; i++) {
    if (point_on_segment(p, world.ball_position(), world.opponent_position(i)) 
	  == world.ball_position())
      obs_flags &= ~OBS_OPPONENT(i);
  }

  if (world.obsLine(p, world.ball_position(), obs_flags)) { return -1; }

  if (evaluation.aim(world, world.now, p, 
		     world.their_goal_r, world.their_goal_l, 
		     OBS_OPPONENTS, target, tolerance)) {
    a = (target - p).angle();

    return tolerance;
  } else return 0.0;

}

bool tsteal_registered = 
Tactic::registerParser("steal", TSteal::parser);

Tactic *TSteal::parser(const char *param_string)
{
  int n = Parse::skipWS(param_string, " \t");

  if (param_string[n] == '\0' || param_string[n] == '\n')
    return new TSteal();
  
  TCoordinate t;

  Parse::pTCoordinate(param_string, t);

  return new TSteal(t);
}

void TSteal::command(World &world, int me, Robot::RobotCommand &command,
		     bool debug)
{
  command.cmd = Robot::CmdSteal;
  command.target = world.ball_position();
  command.ball_target = target_set ? target.asVector(world) : 
    (world.ball_position() + world.robot[me]->sensors.good_ball_dir * 300.0);

  if (debug) {
    gui_debug_line(me, GDBG_TACTICS, world.teammate_position(me),
		   world.ball_position(), G_ARROW_FORW);
    gui_debug_line(me, GDBG_TACTICS, world.ball_position(),
		   command.target, G_ARROW_FORW);
  }
}

bool tclear_registered = 
Tactic::registerParser("clear", TClear::parser);

void TClear::command(World &world, int me, Robot::RobotCommand &command,
		     bool debug)
{
  vector2d ball = world.ball_position();

  vector2d target;
  double angle_tolerance;

  bool aimed = false;

  vector2d downfield[2];

  Robot::BallShotType shot_type= Robot::BallShotClear;

  downfield[0].set(ball.x + 180.0, -FIELD_WIDTH_H);
  downfield[1].set(ball.x + 180.0, FIELD_WIDTH_H);

  if (debug)
    gui_debug_line(me, GDBG_TACTICS, downfield[0], downfield[1]);

  if (!prev_target_set) prev_target = world.their_goal;

  aimed = evaluation.aim(world, world.now, world.ball_position(),
			 downfield[0], downfield[1],
			 OBS_EVERYTHING_BUT_ME(me),
			 prev_target, DVAR(SHOOT_AIM_PREF_AMOUNT),
			 target, angle_tolerance);

  if (!aimed) {
    // Guaranteed to return true and fill in the parameters when 
    // obs_flags is empty.
    evaluation.aim(world, world.now, world.ball_position(),
		   downfield[0], downfield[1],
		   0, target, angle_tolerance);
  }

  target = (target - ball).norm(MIN(FIELD_LENGTH_H - ball.x, 1000.0)) + ball;

  // If the target tolerances include the goal then just aim there.
  double a = (target - ball).angle();
  double a_to_goal = (world.their_goal - ball).angle();
  //double a_to_goal_l = (world.their_goal_l - ball).angle();
  //double a_to_goal_r = (world.their_goal_r - ball).angle();

  if (fabs(anglemod(a - a_to_goal)) < 0.8 * angle_tolerance) {
    if (a > a_to_goal) {
      target = world.their_goal;
      angle_tolerance -= fabs(anglemod(a - a_to_goal));
      shot_type = Robot::BallShotOnGoal;
    } else if (a < a_to_goal) {
      target = world.their_goal;
      angle_tolerance -= fabs(anglemod(a - a_to_goal));
      shot_type = Robot::BallShotOnGoal;
    }
  }

  if (debug) {
    gui_debug_line(me, GDBG_TACTICS, ball, target);
    gui_debug_line(me, GDBG_TACTICS, ball,
		   (target - ball).rotate(angle_tolerance) + ball);
    gui_debug_line(me, GDBG_TACTICS, ball,
		   (target - ball).rotate(-angle_tolerance) + ball);
  }

  if (angle_tolerance < DVAR(SHOOT_MIN_ANGLE_TOLERANCE)) {
    angle_tolerance = DVAR(SHOOT_MIN_ANGLE_TOLERANCE);
  }

  prev_target = target;
  prev_target_set = true;

  command.cmd = Robot::CmdMoveBall;
  command.target = target;
  command.ball_target = target;
  command.angle_tolerance = angle_tolerance;
  command.ball_shot_type = shot_type;
}

bool tactive_def_registered = 
Tactic::registerParser("active_def", TActiveDef::parser);

Tactic *TActiveDef::parser(const char *param_string)
{
  int n = Parse::skipWS(param_string, " \t");

  if (param_string[n] == '\0' || param_string[n] == '\n')
    return new TActiveDef();
  
  TCoordinate t;

  Parse::pTCoordinate(param_string, t);

  return new TActiveDef(t);
}

void TActiveDef::command(World &world, int me, Robot::RobotCommand &command, 
			 bool debug)
{
  if (world.possession == World::TheirBall) 
    steal.command(world, me, command, debug);
  else
    clear.command(world, me, command, debug);
}

bool tpass_registered = 
Tactic::registerParser("pass", TPass::parser);

TPass::TPass(int _target)
{
  cr_do_setup();
  target = _target;
}

Tactic *TPass::parser(const char *param_string)
{
  int target;
  Parse::pInt(param_string, target);
  return new TPass(target);
}

void TPass::command(World &world, int me, Robot::RobotCommand &command,
		    bool debug)
{
  vector2d p[2], targetp, ball;
  double angle_tolerance;

  targetp = world.teammate_position(getTeammateId(target));
  ball = world.ball_position();

  targetp += (ball - targetp).norm(70.0);

  p[0] = targetp + (targetp - ball).perp().norm(-DVAR(PASS_TARGET_WIDTH));
  p[1] = targetp + (targetp - ball).perp().norm(DVAR(PASS_TARGET_WIDTH));

  evaluation.aim(world, world.now, world.ball_position(),
		 p[0], p[1], 
		 OBS_EVERYTHING_BUT_ME(me) & ~OBS_TEAMMATE(target),
		 targetp, angle_tolerance);

  if (debug) {
    gui_debug_line(me, GDBG_TACTICS, ball, targetp);
    gui_debug_line(me, GDBG_TACTICS, ball,
		   (targetp - ball).rotate(angle_tolerance) + ball);
    gui_debug_line(me, GDBG_TACTICS, ball,
		   (targetp - ball).rotate(-angle_tolerance) + ball);
  }

  // Set the drive target as 1m from the target, with some exceptions
  // when close.
  vector2d mytarget;

  if ((targetp - ball).length() > 1100)
    mytarget = ball + (targetp - ball).norm(1000);
  else if ((targetp - ball).length() < 100)
    mytarget = targetp;
  else
    mytarget = targetp + (ball - targetp).norm(100);

  command.cmd = Robot::CmdMoveBall;
  command.target = targetp; // mytarget;
  command.ball_target = targetp;
  command.angle_tolerance = angle_tolerance;
  command.ball_shot_type = Robot::BallShotPass;
}

bool tdribble_to_shoot = 
Tactic::registerParser("dribble_to_shoot", TDribbleToShoot::parser);

Tactic *TDribbleToShoot::parser(const char *param_string)
{
  TRegion r;

  param_string += Parse::pTRegion(param_string, r);

  return new TDribbleToShoot(r);
}

double TDribbleToShoot::eval_fn(World &world, const vector2d p, 
				int obs_flags, double &a) 
{
  vector2d target;
  double tolerance;
  
  obs_flags |= OBS_OPPONENTS | OBS_OUR_DZONE | OBS_THEIR_DZONE;
  
  if (world.obsPosition(p, obs_flags)) { a = 0.0; return -1; } 
  
  if (evaluation.aim(world, world.now, p, 
		     world.their_goal_r, world.their_goal_l, 
		     OBS_OPPONENTS, target, tolerance)) {
    a = (target - p).angle();
    
    return tolerance;
  } else return 0.0;
}

void TDribbleToShoot::command(World &world, int me, 
			      Robot::RobotCommand &command,
			      bool debug)
{
  eval.addPoint(world.teammate_position(me));
  eval.update(world, getObsFlags());

  command.cmd = Robot::CmdDribble;
  command.target = eval.point();
  command.ball_target = eval.point();
  command.angle = eval.angle();

  if (debug) {
      eval.region.gui_draw(me, GDBG_TACTICS, world);
      gui_debug_x(me, GDBG_TACTICS, command.target);
  }
}

bool tdribble_to_region = 
Tactic::registerParser("dribble_to_region", TDribbleToRegion::parser);

Tactic *TDribbleToRegion::parser(const char *param_string)
{
  TRegion r;

  param_string += Parse::pTRegion(param_string, r);

  return new TDribbleToRegion(r);
}

void TDribbleToRegion::command(World &world, int me, 
			       Robot::RobotCommand &command,
			       bool debug)
{
  command.cmd = Robot::CmdDribble;
  command.target = region.center(world);
  command.ball_target = region.center(world);
  command.angle = (region.center(world) - world.teammate_position(me)).angle();

  if (debug) {
      region.gui_draw(me, GDBG_TACTICS, world);
      gui_debug_x(me, GDBG_TACTICS, command.target);
  }
}

bool tspin_to_region = 
Tactic::registerParser("spin_to_region", TSpinToRegion::parser);

Tactic *TSpinToRegion::parser(const char *param_string)
{
  TRegion r;

  param_string += Parse::pTRegion(param_string, r);

  return new TSpinToRegion(r);
}

void TSpinToRegion::command(World &world, int me, 
			    Robot::RobotCommand &command,
			    bool debug)
{
  command.cmd = Robot::CmdSteal;
  command.target = world.ball_position();
  command.ball_target = region.center(world);

  if (debug) {
      region.gui_draw(me, GDBG_TACTICS, world);
      gui_debug_x(me, GDBG_TACTICS, command.target);
  }
}

bool treceive_pass_registered = 
Tactic::registerParser("receive_pass", TReceivePass::parser);

void TReceivePass::command(World &world, int me, Robot::RobotCommand &command, 
			 bool debug)
{
  command.cmd = Robot::CmdRecieveBall;
}

bool treceive_deflection_registered = 
Tactic::registerParser("receive_deflection", TReceiveDeflection::parser);

void TReceiveDeflection::command(World &world, int me, 
				 Robot::RobotCommand &command, 
				 bool debug)
{
  vector2d mypos = world.teammate_position(me);

#if 0
  vector2d ball = world.ball_position();
  vector2d ballv = world.ball_velocity();

  double t = closest_point_time(ball, ballv, mypos, vector2d(0, 0));
  vector2d target = (ball + ballv * t);

  target += (target - world.their_goal).norm(40.0);

  gui_debug_line(me, GDBG_TACTICS, world.teammate_position(me), target, 
		 G_ARROW_FORW);

  if (ballv.length() < 100 || ballv.dot(mypos - ball) < 0) {
    command.cmd = Robot::CmdPosition;
    command.target = world.teammate_position(me);
    command.velocity = vector2d(0, 0);
    command.angle = world.teammate_direction(me);
    command.obs = 0;
 } else if ((target - mypos).length() < 100.0 && t < 1.0) {
    command.cmd = Robot::CmdSpin;
    command.target = target;
    command.ball_target = world.their_goal;
    got_to_spin = true;
  } else {
    command.cmd = Robot::CmdPosition;
    command.target = target;
    command.velocity = vector2d(0, 0);
    command.angle = (target - mypos).angle();
    command.obs = OBS_EVERYTHING_BUT_ME(me) & ~OBS_BALL;
  }
#else
  vector2d target = mypos;

  command.cmd = Robot::CmdPosition;
  command.target = target;
  command.velocity = vector2d(0, 0);
  command.angle = (target - mypos).angle();
  command.obs = OBS_EVERYTHING_BUT_ME(me) & ~OBS_BALL;
#endif
}

Status TReceiveDeflection::isDone(World &world, int me)
{
  vector2d ball = world.ball_position();
  vector2d ballv = world.ball_velocity();
  vector2d mypos = world.teammate_position(me);

  if (ballv.length() < 100 || ballv.dot(mypos - ball) < 0)
    return got_to_spin ? Completed : Aborted;
  else
    return InProgress;
}







