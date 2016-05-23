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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "net_gui.h"

#include "util.h"
#include "geometry.h"
#include "constants.h"

#include "timer.h"
#include "world.h"

#include "obstacle.h"
#include "robot.h"


const bool robot_print     = false;
const bool robot_sub_state = false;
const bool robot_debug_die = false;


const char *state_name[] = {
  "SMGotoBall",
  "SMFaceBall",
  "SMApproachBall",
  "SMPullBall",
  "SMFaceTarget",
  "SMDriveToGoal",
  "SMKick",
  "SMSpinAtBall",
  "SMPosition",
  "SMRecieveBall",
  "SMWait"
};

vector2d on_field(vector2d pos,double radius)
{
  pos.x = bound(pos.x,-FIELD_LENGTH_H+radius,FIELD_LENGTH_H-radius);
  pos.y = bound(pos.y,-FIELD_WIDTH_H +radius,FIELD_WIDTH_H -radius);
  return(pos);
}

void Robot::init(int _my_id)
{
  my_id = _my_id;
  state = last_state = SMGotoBall;
  state_start_time = 0.0;

  last_dist_from_target = 0.0;
  last_target_da = 0.0;
  spin_dir = 0;
}

const vector2d own_goal_pos(-FIELD_LENGTH_H-2*BALL_RADIUS,0);
const vector2d opp_goal_pos( FIELD_LENGTH_H+2*BALL_RADIUS,0);

void Robot::updateSensors(World &world)
{
  Sensors &s = sensors;
  vraw vpos;
  double f;

  //==== update high level sensors ====//

  // find out current position
  s.r_pos = world.teammate_position(my_id);
  s.r_ang = world.teammate_direction(my_id);
  s.r_vel = world.teammate_velocity(my_id);
  s.r_ang_vel = world.teammate_angular_velocity(my_id);
  s.r_fwd.set(cos(s.r_ang),sin(s.r_ang));

  s.opp_goal_rel = opp_goal_pos - s.r_pos;

  // ball information
  s.ball_pos  = world.ball_position();
  s.ball_vel  = world.ball_velocity();
  s.ball_rel  = s.ball_pos - s.r_pos;
  s.ball_dist = s.ball_rel.length();
  s.own_goal_to_ball = s.ball_pos - own_goal_pos;
  s.ball_to_opp_goal = opp_goal_pos - s.ball_pos;
  s.ball_dist_from_wall = min(FIELD_LENGTH_H-fabs(s.ball_pos.x),
                              FIELD_WIDTH_H -fabs(s.ball_pos.y));
  f = (s.ball_pos.x + FIELD_LENGTH_H) / FIELD_LENGTH;
  s.good_ball_dir = (s.own_goal_to_ball.norm() * (1-f) + 
                     s.ball_to_opp_goal.norm() * f).norm();
  s.spin_dir = (int)sign_nz(dot(s.ball_rel.perp(),s.good_ball_dir));
  /*
  printf("SPIN: gdb=<%f,%f> bp=<%f,%f> %f %d",
         V2COMP(s.good_ball_dir),V2COMP(s.ball_rel.perp()),
         dot(s.ball_rel.perp(),s.good_ball_dir),s.spin_dir);
  */
  s.ball_conf = world.ball_raw(vpos);
  s.ball_in_opp_goal = (s.ball_pos.x > FIELD_LENGTH_H+2*BALL_RADIUS)
    && fabs(s.ball_pos.y < GOAL_WIDTH_H);
  s.robot_ball = s.ball_rel.rotate(-s.r_ang);
  s.ball_in_front = (s.robot_ball.x > -20) &&
                    (fabs(s.robot_ball.y) < 60);
  s.ball_on_front = s.ball_in_front && (s.robot_ball.x < 120);

  s.ball_target.set(FIELD_LENGTH_H /*+BALL_RADIUS*/,
                    bound(s.ball_pos.y,
                          -GOAL_WIDTH_H+ROBOT_RADIUS,
                           GOAL_WIDTH_H-ROBOT_RADIUS));
  s.ball_target_rel = s.ball_target - s.r_pos;
  s.on_goal = (s.r_fwd.cross(world.their_goal_l - s.r_pos) > 0) &&
              (s.r_fwd.cross(world.their_goal_r - s.r_pos) < 0);
  // printf("on_goal=%d\n",s.on_goal);

  s.can_drive = target_ball_rel.x>60.0 &&
    target_ball_rel.x<500.0 &&
    fabs(target_ball_rel.y)<60.0;

  //==== set up obstacles ====//
  vector2d p,v;
  int i;

  // set up teammates as obstacles
  for(i=0; i<world.n_teammates; i++){
    if(i != my_id){
      p = world.teammate_position(i);
      v = world.teammate_velocity(i);
      s.obs.add_circle(p.x,p.y,110,v.x,v.y,1);
    }
  }

  // set up opponents as obstacles
  for(i=0; i<world.n_teammates; i++){
    p = world.opponent_position(i);
    v = world.opponent_velocity(i);
    s.obs.add_circle(p.x,p.y,95,v.x,v.y,1);
  }

  // walls
  // OBS_WALLS
  s.obs.add_half_plane(-FIELD_LENGTH_H,              0, 1, 0,1);
  s.obs.add_half_plane( FIELD_LENGTH_H,              0,-1, 0,1);
  s.obs.add_half_plane(               0,-FIELD_WIDTH_H, 0, 1,1);
  s.obs.add_half_plane(               0, FIELD_WIDTH_H, 0,-1,1);

  // defense zones
  // OBS_OUR_DZONE
  s.obs.add_rectangle(-FIELD_LENGTH_H-DEFENSE_DEPTH,0,
                      DEFENSE_DEPTH*4,DEFENSE_WIDTH,1);
  //OBS_THEIR_DZONE
  s.obs.add_rectangle( FIELD_LENGTH_H+DEFENSE_DEPTH,0,
                      DEFENSE_DEPTH*4,DEFENSE_WIDTH,1);

  s.obs.set_mask(OBS_EVERYTHING);
}

//==== State Machine Core Functions ==================================//

Robot::SMState Robot::gotoBall(World &world,Sensors &s,RobotCommand &cmd,
                               NavTarget &nav)
{
  vector2d targ,ball_pred;
  double targ_dist,max_behind,behind,t;
  double ball_rad;
  int obs_id;

  // obstacle radius that nav_to_point uses
  ball_rad = bound(s.ball_dist-180,30,60);

  if(cmd.cmd == CmdSteal){
    max_behind = 0;
  }else{
    max_behind = 160; // bound(100,1,100); // -100/10*time_in_state,1,100);
  }

  t = max(s.ball_dist-200.0,0.0) / 1000.0;
  ball_pred = world.ball_position(world.now + t);

  // walk forward looking for free space
  obs_id = -1;
  for(behind=max_behind; behind>=0.0; behind-=10.0){
    targ = ball_pred - target_rel.norm(behind);
    targ = on_field(targ,ROBOT_RADIUS);
    if(s.obs.check(targ,obs_id)){
      break;
    }
  }
  if(robot_print) printf("  behind = %f (obs %d)\n",behind,obs_id);

  if(fabs(behind) < ball_rad+90){
    targ = ball_pred - (ball_pred - s.r_pos).norm(ball_rad+90+10);
    nav.obs &= ~(OBS_BALL);
  }

  /*
  // if no free space go directly to ball
  if(!s.obs.check(targ)){
    behind = 0.0;
    targ = ball_pred;
  }
  */

  targ_dist = Vector::distance(targ,s.r_pos);
  if(state_changed){
    last_dist_from_target = targ_dist;
  }
  gui_debug_printf(my_id, GDBG_TACTICS,
		   "  targ_dist=%f\n",targ_dist);

  if(false){
    printf("  behind=%f targ_dist=%f last=%f\n",
           behind,targ_dist,last_dist_from_target);
  }

  // probably want to add obstacle check here
  if(!world.inOurDefenseZone()){
    switch(cmd.cmd){
      case CmdMoveBall:
      case CmdDribble:
        if(targ_dist<150 && last_dist_from_target<=targ_dist){
	  // printf("  %f <= %f\n",last_dist_from_target,targ_dist);
          if(s.can_drive){
            return(SMDriveToGoal);
          /*}else if(fabs(target_ball_rel.x) < 0.0){
	     return(SMSpinAtBall);*/
	  }else if(s.ball_vel.length()<200){
            return(SMFaceBall);
          }
        }/* else if(target_ball_rel.x>60.0 && fabs(target_ball_rel.y)<50.0){
          return(SMDriveToGoal);
	  }*/
        break;
      case CmdSteal:
        if(targ_dist<200 && last_dist_from_target<targ_dist){
          return(SMSpinAtBall);
        }
        break;
    default:
      printf("Unimeplemented state/cmd pair.\n");
    }
  }

  nav.pos    = targ;
  nav.angle  = s.ball_rel.angle();
  nav.obs  &= ~(OBS_THEIR_DZONE);

  last_dist_from_target = targ_dist;

  if(robot_print){
    printf("  targ=<%f,%f> targ_dist = %f\n",V2COMP(targ),targ_dist);
  }

  return(SMGotoBall);
}

Robot::SMState Robot::faceBall(World &world,Sensors &s,RobotCommand &cmd,
                               NavTarget &nav)
{
  double da,dta;

  da  = angle_mod(s.ball_rel.angle() - s.r_ang);
  dta = angle_mod(target_rel.angle() - s.r_ang);

  if(fabs(da) < 0.1){
    if(s.can_drive){
      return(SMDriveToGoal);
    }else{
      return(SMApproachBall);
    }
  }

  nav.direct = true;
  nav.vel_xya.set(0,0,6*sin(da));

  return(SMFaceBall);
}

Robot::SMState Robot::approachBall(World &world,Sensors &s,RobotCommand &cmd,
                                   NavTarget &nav)
{
  vector2d ball_targ,ball_dir;
  double da,speed;

  /*
  printf("  (%d) (%d %d) %d\n",
         s.ball_dist > 300,
         target_ball_rel.x>0.0,fabs(target_ball_rel.y)<60.0,
         s.ball_on_front);
  */

  if((s.ball_dist > 300) || world.inOurDefenseZone()) return(SMGotoBall);
  if(s.can_drive && (s.ball_dist_from_wall > 100) &&
     time_in_state>=1.0){ // && s.r_vel.length()<200)){
    return(SMDriveToGoal);
  }

  // printf("  bof=%d t=%f wd=%f\n",s.ball_on_front,time_in_state,s.ball_dist_from_wall);
  if(s.ball_on_front && time_in_state >= 1.0){
    // return((s.ball_dist_from_wall > 100)? SMFaceTarget : SMPullBall);
    return((s.ball_dist_from_wall > 50)? SMFaceTarget : SMSpinAtBall);
  }else if(time_in_state >= 1.0){
    return(SMFaceBall);
  }

  if(state_changed) initial_ball_dist = s.ball_dist;

  ball_targ = on_field(s.ball_pos,90);
  ball_dir = (ball_targ - s.r_pos).norm();
  da = angle_mod(s.ball_rel.angle() - s.r_ang);
  speed = 1.5*initial_ball_dist * (1.0 - time_in_state);

  nav.dribble = true;
  nav.direct  = true;
  if(omni){
    ball_dir = ball_dir.rotate(-s.r_ang) * speed;
    nav.vel_xya.set(ball_dir.x,ball_dir.y,0);
  }else{
    nav.vel_xya.set(speed,0,0);
  }
  // printf("  <%f,%f> <%f,%f,%f>\n",V2COMP(ball_dir),V3COMP(nav.vel_xya));

  return(SMApproachBall);
}

Robot::SMState Robot::pullBall(World &world,Sensors &s,RobotCommand &cmd,
                               NavTarget &nav)
{
  double vx;

  if(time_in_state > 1.0) return(SMFaceTarget);
  if(!s.ball_on_front && s.ball_conf>0.5) return(SMGotoBall);

  vx = -500.0 * time_in_state;

  nav.dribble = true;
  nav.direct = true;
  nav.vel_xya.set(vx,0,0);

  return(SMPullBall);
}

Robot::SMState Robot::faceTarget(World &world,Sensors &s,RobotCommand &cmd,
                                 NavTarget &nav)
{
  double da,sx,sa,vx,va;

  da = angle_mod(target_rel.angle() - s.r_ang);

  if(robot_print){
    printf("  (%d %d) (%d) (%d)\n",
           !s.ball_on_front,s.ball_conf>0.5,
           fabs(da) < 0.2,time_in_state > 4.0);
  }

  if(!s.ball_on_front && s.ball_conf>0.5) return(SMGotoBall);
  if(target_ball_rel.x>0.0 &&
     fabs(target_ball_rel.y)<60.0) return(SMDriveToGoal);

  if(world.teammate_stuck(my_id)>0.90 && time_in_state>1.0){
    return(SMSpinAtBall);
  }

  sa = min(  4.0*time_in_state,  2.5);
  sx = min(500.0*time_in_state,500.0);
  va = bound(2*da,-sa,sa);
  vx = bound((ROBOT_RADIUS+BALL_RADIUS)*va/tan(da),-sx,sx);

  nav.dribble = true;
  nav.direct = true;
  nav.vel_xya.set(vx,0,va);

  return(SMFaceTarget);
}

Robot::SMState Robot::driveToGoal(World &world,Sensors &s,RobotCommand &cmd,
                                  NavTarget &nav)
{
  vector2d ball_to_target = cmd.ball_target-s.ball_pos;
  // double carrot_dist = max(500.0-s.ball_dist,1.0);
  // vector2d carrot_pos = s.ball_pos + ball_to_target.norm(carrot_dist);
  double t,dba;
  double tdist,tbdist;

  t = cosine(s.ball_rel,target_rel);
  dba = angle_mod(ball_to_target.angle() - s.r_ang);
  tdist  = target_rel.length();
  tbdist = Vector::distance(cmd.ball_target,s.r_pos);

  if(robot_print){
    printf("  to kick: c%d f%d s%d a%d d%d\n",
	   cmd.cmd==CmdMoveBall,s.ball_on_front,
	   Vector::dot(s.r_vel,s.ball_rel.norm()) > 100.0,
	   (fabs(dba) < cmd.angle_tolerance),
	   (tbdist < 1000+500*omni));
  }

  if(cmd.cmd==CmdMoveBall && s.ball_on_front &&
     Vector::dot(s.r_vel,s.ball_rel.norm()) > 100.0 &&
     (fabs(dba) < cmd.angle_tolerance)){

    switch(cmd.ball_shot_type){
      case BallShotOnGoal:
      case BallShotPass:
        if((tbdist > 100) && (tbdist < 1000+500*omni) &&
           (fabs(dba) < cmd.angle_tolerance*0.75)) return(SMKick);
        break; 

      case BallShotClear:
        // if(!omni || s.on_goal) return(SMKick);
	if(!omni || fabs(s.r_ang)<0.5) return(SMKick);
        // don't let the omni kick out of the field
        // 0.5 radians is about 30 degrees
    }
  }

  if(target_ball_rel.x<20.0 ||
     target_ball_rel.x>250.0 ||
     fabs(target_ball_rel.y)>90.0) return(SMGotoBall);
  if(s.ball_dist_from_wall < 50) return(SMSpinAtBall);

  /*
  if(tdist > 200 &&
     (target_ball_rel.x<0.0 ||
      fabs(target_ball_rel.y)>90.0 ||
      fabs(target_ball_rel.x)>90.0)) return(SMGotoBall);
  // if(t < 0.0) return(SMGotoBall);
  */

  /*
  if(world.teammate_stuck(my_id) > 0.90){
    return(SMSpinAtBall);
  }
  */

  if(robot_print) printf("  <%f,%f> dba=%f\n",
                         V2COMP(target_ball_rel),dba);

  if(omni){
    nav.dribble = true;

    nav.pos   = cmd.target; // carrot_pos; // s.ball_pos; // cmd.target;
    nav.angle = (cmd.target - s.r_pos).angle();
    nav.vel   = ball_to_target.norm(1000);

    nav.vel_xya.set(0,bound(target_ball_rel.y,-100,100),0);

    // nav.pos   = carrot_pos; // cmd.target;
    // nav.vel   = ball_to_target.norm(500);
    // nav.angle = (cmd.target - s.r_pos).angle();

    /*
    // nav.pos   = cmd.target;
    if(Vector::distance(cmd.target,s.r_pos) < 40){
      nav.angle = (cmd.ball_target - s.r_pos).angle();
    }else{
       // nav.angle = (cmd.target - s.r_pos).angle();
    }
    */
  }else{
    if(s.r_vel.length() < 1000) nav.dribble = true;
    nav.pos   = cmd.target; // carrot_pos;
    // nav.vel   = ball_to_target.norm(500);
    // nav.pos   = cmd.target;
    // nav.angle = target_rel.angle();
    nav.angle = ball_to_target.angle();
  }
  nav.obs  &= ~(OBS_BALL|OBS_THEIR_DZONE);
  nav.type = GotoPointShoot;

  return(SMDriveToGoal);
}

Robot::SMState Robot::kick(World &world,Sensors &s,RobotCommand &cmd,
                           NavTarget &nav)
{
  double da,va;
  double k;

  if(state_changed){
    last_target_da = 2*M_PI;
  }

  if(time_in_state>0.25+1.75*omni) return(SMGotoBall);
  if(!s.ball_in_front) return(SMGotoBall);

  da = angle_mod((cmd.ball_target-s.ball_pos).angle() - s.r_ang);
  // - s.ball_vel.angle());
  // (s.r_ang_vel*da < 0.0)
  /*
    ((fabs(da) < 0.25) && (fabs(da) > fabs(last_target_da))) ||
    (time_in_state > 0.50)){
  */

  k = omni? 1 : 4;
  if(fabs(da) < min(0.50+k*time_in_state,1.0)*cmd.angle_tolerance
     && (!omni || world.time - last_kick_timestamp > 0.75)){
    printf("Kick! (t=%f)\n",time_in_state);
    nav.kick = true;
    last_kick_timestamp = world.time;
  }

  if(robot_print){
    printf("  targ=<%f,%f>:%f va=%f da=%f k=%d\n",
           V2COMP(target_rel),target_rel.angle(),
           s.r_ang_vel,da,
           nav.kick);
  }

  nav.direct = true;
  if(omni){
    va = 6*sin(da);
    nav.vel_xya.set(world.teammate_robot_velocity(my_id).x,
		    -va*70.0,
		    va);
  }else{
    nav.vel_xya.set(world.teammate_robot_velocity(my_id).x + 1000*FRAME_PERIOD,
		    0.0,
		    6*sin(da));
  }

  last_target_da = da;

  return(SMKick);
}

Robot::SMState Robot::spinAtBall(World &world,Sensors &s,RobotCommand &cmd,
                                 NavTarget &nav)
{
  double ofs;
  // vector2d dzone_closest;

  if(state_changed){
    start_dist_from_ball = s.ball_dist;
  }

  if(cmd.cmd == CmdSpin){
    ofs = offset_to_line(s.r_pos,cmd.ball_target,s.ball_pos);
    nav.pos = cmd.target;
  }else{
    if(world.inOurDefenseZone()) return(SMGotoBall);
    if(s.ball_dist > min(start_dist_from_ball+10,300.0)) return(SMGotoBall);
    if(time_in_state > 10.0) return(SMGotoBall);

    if(cmd.ball_shot_type == BallShotClear){
      /*
      dzone_closest.set(
        bound(s.ball_pos.x,-FIELD_WIDTH_H,-FIELD_WIDTH_H+DEFENSE_DEPTH),
        bound(s.ball_pos.y,-DEFENSE_WIDTH_H, DEFENSE_WIDTH_H));
      ofs = offset_to_line(dzone_closest,s.r_pos,s.ball_pos);
      */
      ofs = Vector::dot(s.good_ball_dir.perp(),s.ball_pos-s.r_pos);
    }else{
      ofs = offset_to_line(s.r_pos,cmd.ball_target,s.ball_pos);
    }
    nav.pos   = s.ball_pos;
  }

  if(false && world.obsLine(s.r_pos,s.ball_pos - s.ball_rel.norm(90),
		   (OBS_OPPONENTS|OBS_TEAMMATES)&~(OBS_TEAMMATE(my_id)))){
    // printf("spinAtBall::goto\n");
    nav.pos   = s.ball_pos - s.ball_pos.norm(45);
    nav.angle = s.ball_rel.angle();
  }else{
    // printf("spinAtBall::spin\n");
    spin_dir = (int)sign_nz(ofs);
    nav.angle = -10*spin_dir;
    nav.spin  = true;
  }

  return(SMSpinAtBall);
}

Robot::SMState Robot::position(World &world,Sensors &s,RobotCommand &cmd,
                               NavTarget &nav)
{
  nav.pos   = cmd.target;
  nav.vel   = cmd.velocity;
  nav.angle = cmd.angle;
  nav.obs   = cmd.obs;
  nav.type  = cmd.goto_point_type;

  return(SMPosition);
}

Robot::SMState Robot::recieveBall(World &world,Sensors &s,RobotCommand &cmd,
                                  NavTarget &nav)
{
  double a,da;

  if(!s.ball_on_front){
    a  = s.ball_rel.angle();
    da = angle_mod(a - s.r_ang);
  }else{
    da = 0;
  }

  nav.dribble = true;
  nav.direct = true;
  nav.vel_xya.set(0,0,3*sin(da));

  return(SMRecieveBall);
}

Robot::SMState Robot::wait(World &world,Sensors &s,RobotCommand &cmd,
                           NavTarget &nav)
{
  if(time_in_state > 1.0) return(SMGotoBall);

  nav.direct = true;
  nav.vel_xya.set(0,0,0);

  return(SMWait);
}


// precondition: update_sensors has been run this timestep
Status Robot::run(World &world,RobotCommand &cmd,Trajectory &tcmd)
{
  Sensors &s = sensors;
  Status status;
  NavTarget nav;
  int old_state;
  int n;

  omni = (world.teammate_type(my_id) == ROBOT_TYPE_OMNI);

  // If command changed, set initial state
  if(cmd.cmd != last_cmd){
    switch(cmd.cmd){
      case CmdMoveBall:    state = SMGotoBall; break;
      case CmdSteal:       state = SMGotoBall; break;
      case CmdDribble:     state = SMGotoBall; break;
      case CmdRecieveBall: state = SMRecieveBall; break;
      case CmdPosition:    state = SMPosition; break;
      case CmdSpin:        state = SMSpinAtBall; break;

      default:
        printf("Invalid Command.\n");
        return(Failed);
        break;
    }
  };

  // common calculations
  target_rel = cmd.target - s.r_pos;
  target_ball_rel.set(s.ball_rel.dot(target_rel.norm()),
		      s.ball_rel.dot(target_rel.norm().perp()));
  // target_ball_rel = s.ball_rel.rotate(-(cmd.target-s.r_pos).angle());

  // execute states until no longer switching
  n = 10;
  state_changed = false;
  do{
    old_state = state;
    time_in_state = world.time - state_start_time;

    mzero(nav);
    nav.obs = OBS_EVERYTHING_BUT_ME(my_id);
    if(robot_sub_state) printf("SubState [%s]\n",state_name[state]);

    switch(state){
      case SMGotoBall:     state = gotoBall    (world,s,cmd,nav); break;
      case SMFaceBall:     state = faceBall    (world,s,cmd,nav); break;
      case SMApproachBall: state = approachBall(world,s,cmd,nav); break;
      case SMPullBall:     state = pullBall    (world,s,cmd,nav); break;
      case SMFaceTarget:   state = faceTarget  (world,s,cmd,nav); break;
      case SMDriveToGoal:  state = driveToGoal (world,s,cmd,nav); break;
      case SMKick:         state = kick        (world,s,cmd,nav); break;
      case SMSpinAtBall:   state = spinAtBall  (world,s,cmd,nav); break;
      case SMPosition:     state = position    (world,s,cmd,nav); break;
      case SMRecieveBall:  state = recieveBall (world,s,cmd,nav); break;
      case SMWait:         state = wait        (world,s,cmd,nav); break;
    }
    if(state!=old_state){
      state_start_time = world.time;
      state_changed = true;
    }
  }while(state!=old_state && --n);

  if(n <= 0){
    printf("Robot::Oscillation!\n");
    nav.direct = true;
    nav.vel_xya.set(100,0,0);

    if(robot_debug_die) exit(1);
  }

  // common calculations
  if(ttl > 0) ttl--;

  if(robot_print){
    printf("State: %s %0.2fs R(%8.2f,%8.2f)\n",
           state_name[state],time_in_state,
           V2COMP(s.r_pos));
  }

  gui_debug_printf(my_id, GDBG_TACTICS, 
		   "State: %s %0.2fs\n",state_name[state],time_in_state);
  gui_debug_printf(my_id, GDBG_TACTICS, 
		   "  robot_ball=<%f,%f>\n",V2COMP(s.robot_ball));
  if(robot_print){
    printf("  robot_ball=<%f,%f>:%d target_ball_rel=<%f,%f> stuck=%f\n",
           V2COMP(s.robot_ball),s.ball_on_front,
	   V2COMP(target_ball_rel),
	   world.teammate_stuck(my_id));
  }

  // carry out target command
  if(nav.direct){
    tcmd.vx = nav.vel_xya.x;
    tcmd.vy = nav.vel_xya.y;
    tcmd.va = nav.vel_xya.z;
  }else{
    if(nav.spin){
      tcmd = spin_to_point(world,my_id,nav.pos,nav.angle);
    }else{
      tcmd = nav_to_point(world,my_id,
                          nav.pos,nav.vel,nav.angle,
                          nav.obs,nav.type);
      tcmd.vx += nav.vel_xya.x;
      tcmd.vy += nav.vel_xya.y;
      tcmd.va += nav.vel_xya.z;
    }
  }
  tcmd.kicker_on   = nav.kick;
  tcmd.dribbler_on = nav.dribble;
  // if(robot_print && nav.kick) printf("Kick!\n");

  switch(cmd.cmd){
    case CmdMoveBall:
      status = (nav.kick)? Completed : InProgress;
      break;

    case CmdSteal:
      status = InProgress;
      break;

    case CmdRecieveBall:
      status = (s.ball_on_front)? Completed : InProgress;
      break;
 
    case CmdSpin:
    case CmdDribble:
    case CmdPosition:
      status = (Vector::distance(cmd.target,s.r_pos) < 20.0)? Completed : InProgress;
      break;

    default:
      status = InProgress;
  }

  // some final calculations
  last_state = state;
  last_cmd = cmd.cmd;

  return(status);
}

Status Robot::run(World &world,RobotCommand &cmd)
{
  Trajectory tcmd;
  Status s;

  s = run(world,cmd,tcmd);
  world.go(my_id,tcmd.vx,tcmd.vy,tcmd.va,
           tcmd.kicker_on,tcmd.dribbler_on);

  return(s);
}

double Robot::time(World &world,RobotCommand &cmd)
{
  Sensors &s = sensors;
  double t;

  int nobs = 0;

  switch(cmd.cmd){
  case CmdMoveBall:
  case CmdDribble:
    t = (Vector::distance(s.r_pos,s.ball_pos) +
	 Vector::distance(s.ball_pos,cmd.target)) / 1000.0;
    
    t += fabs(anglemod((s.ball_pos - s.r_pos).angle() - 
		       (cmd.target - s.ball_pos).angle())) / 1.5;
    
    nobs += world.obsLineNum(s.r_pos, s.ball_pos,
			     OBS_EVERYTHING_BUT_ME(my_id) & ~OBS_BALL);
    nobs += world.obsLineNum(s.ball_pos, cmd.target,
			     OBS_EVERYTHING_BUT_ME(my_id) & ~OBS_BALL);
    
    t += (nobs * M_PI * ROBOT_DEF_WIDTH_H) / 500.0;
    
    break;
    
  case CmdSteal:
    t = Vector::distance(s.r_pos,s.ball_pos) / 1000.0;
    
    nobs += world.obsLineNum(s.r_pos, s.ball_pos,
			     OBS_EVERYTHING_BUT_ME(my_id) & ~OBS_BALL);
    
    t += (nobs * M_PI * ROBOT_DEF_WIDTH_H) / 500.0;
    
    break;
    
  case CmdRecieveBall:
  case CmdSpin:
    t = Vector::distance(s.r_pos,s.ball_pos) / 1000.0;
    break;
    
  case CmdPosition:
    t = Vector::distance(s.r_pos,cmd.target) / 1000.0;
    
    nobs += world.obsLineNum(s.r_pos, cmd.target,
			     OBS_EVERYTHING_BUT_ME(my_id) & ~OBS_BALL);
    t += (nobs * M_PI * ROBOT_DEF_WIDTH_H) / 500.0;
    break;
    
  default:
    t = 0;
  }
  
  return(t);
}
