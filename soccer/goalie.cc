// goalie.cc
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

#include <stdio.h>
#include <math.h>

#include <configreader.h>

#include "evaluation.h"
#include "soccer.h"
#include "goalie.h"

CR_DECLARE(GOALIE_MAX_ANGLE);
CR_DECLARE(GOALIE_DIST_OFF_BALL);
CR_DECLARE(GOALIE_DIST_MIN);
CR_DECLARE(GOALIE_DIST_MAX);

CR_DECLARE(GOALIE_PENALTY_MATCH_THEM);
CR_DECLARE(GOALIE_PENALTY_LINE);
CR_DECLARE(GOALIE_PENALTY_MAX);

static bool cr_setup = false;

bool tgoalie_registered = Tactic::registerParser("goalie", TGoalie::parser);

TGoalie::TGoalie()
{
  if (!cr_setup) {
    CR_SETUP(tactic, GOALIE_MAX_ANGLE, CR_DOUBLE);
    CR_SETUP(tactic, GOALIE_DIST_OFF_BALL, CR_DOUBLE);
    CR_SETUP(tactic, GOALIE_DIST_MIN, CR_DOUBLE);
    CR_SETUP(tactic, GOALIE_DIST_MAX, CR_DOUBLE);

    CR_SETUP(tactic, GOALIE_PENALTY_MATCH_THEM, CR_INT);
    CR_SETUP(tactic, GOALIE_PENALTY_LINE, CR_DOUBLE);
    CR_SETUP(tactic, GOALIE_PENALTY_MAX, CR_DOUBLE);

    cr_setup = true;
  }

  penalty_play = false; penalty_tend = 0.0;
}

void TGoalie::command(World &world, int me, Robot::RobotCommand &command,
		      bool debug)
{
  if (penalty_play && world.isGameRunning() && (penalty_tend > world.time) &&
      (world.teammate_type(me) != ROBOT_TYPE_DIFF)) {

    if (debug)
      gui_debug_printf(me, GDBG_TACTICS, "Charging %f : penalty_end!!!\n", penalty_tend - world.time);

    penaltyCharge(world, me, command,debug);
    return;

  } else if (world.restartPenalty() && (world.restartWhoseKick() == World::TheirBall)) {
    //      printf("Running penalty play\n");
    if (debug) {
      gui_debug_printf(me, GDBG_TACTICS, "Starting penalty play...\n");
    }
    
    penalty_tend = world.time + 0.75;
    penalty_play = true;
    penaltyPlay(world, me, command,debug);
    return;
  }

  penalty_play = false;
  penalty_tend = 0;

  
  bool intercept = true;

  vector2d ball = world.ball_position();
  vector2d target, target_vel;
  vector2d pos, dir, obs;
  double angle,max_x;

  const vector2d bbox_min(-FIELD_WIDTH_H,-DEFENSE_WIDTH_H);
  const vector2d bbox_max(             0, DEFENSE_WIDTH_H);

  pos = world.teammate_position(me);

  dir.set(-1,0);
  obs = evaluation.farthest(world, world.now,
                            (OBS_TEAMMATES|OBS_OPPONENTS)&~OBS_TEAMMATE(me),
                            bbox_min,bbox_max,
                            dir);
  if(debug){
    // printf("%f,%f\n",V2COMP(obs));
    gui_debug_line(me, GDBG_TACTICS, obs-dir.perp()*100, obs+dir.perp()*100);
  }

  max_x = bound(obs.x - OMNIBOT_RADIUS,
                -FIELD_LENGTH_H + DEFENSE_DEPTH-OMNIBOT_RADIUS,
                -FIELD_LENGTH_H + DVAR(GOALIE_DIST_MAX));
  /*
  printf("%f (%f [%f,%f])\n",FIELD_LENGTH_H+max_x,
         obs.x-OMNIBOT_RADIUS,
         -FIELD_LENGTH_H+DEFENSE_DEPTH-OMNIBOT_RADIUS,
         -FIELD_LENGTH_H+DVAR(GOALIE_DIST_MAX));
  */

  // Call defend to find the target position.
  if (!evaluation.defend_line(world, world.now, 
			      world.our_goal_r, world.our_goal_l,
			      DVAR(GOALIE_DIST_MIN), FIELD_LENGTH_H + max_x,
			      DVAR(GOALIE_DIST_OFF_BALL), intercept,
			      target, target_vel)) {
    target = ball;
    target.x -= 50.0;

    target_vel = vector2d(0, 0);
  }

  // Bound position
  if (target.x > ball.x) target.x = ball.x;
  if (fabs(target.y) > GOAL_WIDTH_H) {
    target = intersection(ball, target, 
			  vector2d(-1400, GOAL_WIDTH_H * sign_nz(target.y)),
			  vector2d(0, GOAL_WIDTH_H * sign_nz(target.y)));
    target.x = bound(target.x, -FIELD_LENGTH_H+DVAR(GOALIE_DIST_MIN), max_x);
  }
  // printf("x = %f\n",x+FIELD_LENGTH_H-DEFENSE_DEPTH);

  // Use angle to ball as the goalie's facing.
  angle = (ball - target).angle();
  angle = abs_bound(angle,DVAR(GOALIE_MAX_ANGLE));

  if (world.teammate_type(me) == ROBOT_TYPE_DIFF)
    angle = world.teammate_nearest_direction(me, angle + M_PI_2);

  // Do not avoid any obstacles, except the ball.
  int obs_flags = 0;

  // get behind ball (avoiding it) if in front of it
  if((ball.x < pos.x) && (fabs(ball.y) < GOAL_WIDTH_H)){
    angle = world.teammate_direction(me);
    target.x -= TEAMMATE_EFFECTIVE_RADIUS + BALL_RADIUS;

    obs_flags = OBS_BALL;
  }else if(world.inOurDefenseZone()){
    target = ball;
    gui_debug_printf(me, GDBG_TACTICS,"Goalie:Clearing\n");
  }

  if(debug){
    vector2d aim_target;
    double angle_tolerance;

    gui_debug_line(me, GDBG_TACTICS, 
		   world.teammate_position(me), target, G_ARROW_FORW);

    evaluation.aim(world, world.now, world.ball_position(),
		   world.our_goal_r, world.our_goal_l, OBS_TEAMMATE(me),
		   aim_target, angle_tolerance);

    gui_debug_line(me, GDBG_TACTICS, ball, aim_target);
    gui_debug_line(me, GDBG_TACTICS, ball,
		   (aim_target - ball).rotate(angle_tolerance) + ball);
    gui_debug_line(me, GDBG_TACTICS, ball,
		   (aim_target - ball).rotate(-angle_tolerance) + ball);
  }

  command.cmd = Robot::CmdPosition;
  command.target = target;
  command.velocity = vector2d(0, 0); // target_vel;
  command.angle = angle;
  command.obs = obs_flags;
}

Tactic *TGoalie::parser(const char *param_string)
{
  return new TGoalie();
}


////// Goalie Penalty kick code

void TGoalie::penaltyPlay(World &world, int me, Robot::RobotCommand &command,
			  bool debug)
{

  if (debug) {
    gui_debug_printf(me, GDBG_TACTICS, "Running penalty play");
  }
    
  vector2d ball = world.ball_position();
  vector2d target, target_vel;
  vector2d pos, dir;
  double angle;
  
  pos = world.teammate_position(me);

  dir.set(-1,0);

  // Call defend to find the target position.
  target.set(DVAR(GOALIE_PENALTY_LINE), 0);
  target_vel = vector2d(0, 0);
  angle = 0;

  // need to adjust for where opponent is kicking here

  // need to gate this
  int oid = world.choosePenaltyKicker();
  if ((oid >= 0) && IVAR(GOALIE_PENALTY_MATCH_THEM)) {
    vector2d kpos = world.opponent_position(oid);
    vector2d kvec = (ball - kpos).norm();
    vector2d knorm = kvec.perp();

    // we want kperp to point towards the goal to use the right side
    if (knorm.x > 0)
      knorm = -knorm;

    // make robot independent
    double rad = OMNIBOT_RADIUS;

    if (world.teammate_type(me) == ROBOT_TYPE_DIFF)
      rad = DIFFBOT_LENGTH_H;
    vector2d projpoint = ball + knorm * rad;
    
    vector2d glinea(DVAR(GOALIE_PENALTY_LINE), -GOAL_WIDTH_H);
    vector2d glineb(DVAR(GOALIE_PENALTY_LINE), GOAL_WIDTH_H);
    
    vector2d go2point = intersection(projpoint, projpoint + kvec, glinea, glineb);
    vector2d ipoint = intersection(ball, ball + kvec, glinea, glineb);

    // need bounding

    // fix this!!!
    go2point.y = bound(go2point.y, -DVAR(GOALIE_PENALTY_MAX), DVAR(GOALIE_PENALTY_MAX));

    // check we are not just in the middle
    //    if ((go2point.y - ball.y) * (kpos.y - ball.y) > 0)
    //      go2point.y = ball.y;
    if (fabs(ipoint.y) < OMNIBOT_RADIUS)
      go2point.y = ball.y;

    if (debug) {
      gui_debug_line(me, GDBG_TACTICS, kpos, kpos + kvec * 1000.0, 0);

      gui_debug_line(me, GDBG_TACTICS, 
		     vector2d(DVAR(GOALIE_PENALTY_LINE), -DVAR(GOALIE_PENALTY_MAX)),
		     vector2d(DVAR(GOALIE_PENALTY_LINE), DVAR(GOALIE_PENALTY_MAX)), 0);
      gui_debug_line(me, GDBG_TACTICS, 
		     projpoint, projpoint + kvec, 0); 
      gui_debug_line(me, GDBG_TACTICS, 
		     projpoint, projpoint + kvec * 1000.0, 0); 
      gui_debug_line(me, GDBG_TACTICS, 
		     go2point, go2point, G_ARROW_FORW); 
      
    }

    // lets try it
    target = go2point;
    angle = (ball - target).angle();

  }


  if (world.teammate_type(me) == ROBOT_TYPE_DIFF)
    angle = world.teammate_nearest_direction(me, angle + M_PI_2);

  // Do not avoid any obstacles, except the ball.
  int obs_flags = 0;

  // get behind ball (avoiding it) if in front of it

  if(debug){
    vector2d aim_target;
    double angle_tolerance;

    gui_debug_line(me, GDBG_TACTICS, 
		   world.teammate_position(me), target, G_ARROW_FORW);

    evaluation.aim(world, world.now, world.ball_position(),
		   world.our_goal_r, world.our_goal_l, OBS_TEAMMATE(me),
		   aim_target, angle_tolerance);

    gui_debug_line(me, GDBG_TACTICS, ball, aim_target);
    gui_debug_line(me, GDBG_TACTICS, ball,
		   (aim_target - ball).rotate(angle_tolerance) + ball);
    gui_debug_line(me, GDBG_TACTICS, ball,
		   (aim_target - ball).rotate(-angle_tolerance) + ball);
  }

  command.cmd = Robot::CmdPosition;
  command.target = target;
  command.velocity = vector2d(0, 0); // target_vel;
  command.angle = angle;
  command.obs = obs_flags;
}


bool TGoalie::penaltyCharge(World &world, int me, Robot::RobotCommand &command,
			    bool debug)
{
  if (debug) {
    gui_debug_printf(me, GDBG_TACTICS, "Penalty charge down");
  }
  
  vector2d ball = world.ball_position();
  vector2d mypos = world.teammate_position(me);
  vector2d r2ball = (ball - mypos);
  
  if ((mypos.x > -(PENALTY_SPOT - OMNIBOT_RADIUS)) || 
      (r2ball.x < OMNIBOT_RADIUS)) {
    printf("Maybe do something now!!!\n");
    return (false);
  }
  
  // change this to drive at ball
  // nav to point and fix obstacles
  command.cmd = Robot::CmdPosition;
  command.target = ball;
  command.velocity = r2ball.norm(500.0);
  command.angle = 0; //r2ball.angle();
  command.obs = 0; //OBS_EVERYTHING_BUT_ME(me);

  return (true);
}
