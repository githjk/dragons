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

#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "util.h"
#include "geometry.h"
#include "constants.h"
#include "soccer.h"
#include "world.h"

#include "obstacle.h"
#include "path_planner.h"

/*
#define MAXFLOAT 3.40282347e+38F
#define LATENCY_FRAMES 1
#define TIMESTEP (1/30.0)
*/

struct box{
  vector2d min,max;

  vector2d bound(vector2d p){
    return(vector2d(::bound(p.x,min.x,max.x),::bound(p.y,min.y,max.y)));
  }
  bool inside(vector2d p){
    return((p.x >= min.x) && (p.x <= max.x) &&
           (p.y >= min.y) && (p.y <= max.y));
  }
};

//==== Behaviors =====================================================//

class Robot{
public:
// interface
  enum GotoPointType{
    GotoPointMove    = 0,
    GotoPointShoot   = 1, 
    GotoPointDribble = 2,
    GotoPointPass    = 3,
    GotoPointMoveForw= 4
  };

  struct Sensors{
    obstacles obs;

    vector2d r_pos,r_vel,r_fwd;
    double r_ang,r_ang_vel,ball_dist,ball_dist_from_wall;
    vector2d ball_pos,ball_vel,ball_rel;

    vector2d own_goal_to_ball;
    vector2d ball_to_opp_goal;
    vector2d good_ball_dir;
    bool on_goal;

    vector2d opp_goal_rel;
    vector2d ball_target,ball_target_rel;
    vector2d robot_ball;
    bool ball_on_front,ball_in_front;

    float ball_conf;
    int spin_dir; // 1=CCW, -1=CW
    bool can_drive;
    bool ball_in_opp_goal;
  };

  struct NavTarget{
    // navigation target
    vector2d pos;
    vector2d vel;
    double angle;
    int obs;

    // raw robot velocities
    vector3d vel_xya;

    // flags
    bool direct;   // raw velocities (not a navigation target)
    bool dribble;  // turn on dribbler
    bool kick;     // turn on kicker
    bool spin;     // spin to target point
    GotoPointType type;

    // how much left to do
    Status status;
  };

  struct Trajectory{
    double vx, vy, va;
    bool kicker_on, dribbler_on;
    
    double eta;

    Trajectory(){;}

    Trajectory(double _vx, double _vy, double _va, 
               double _eta = 0.0,
               bool _kicker_on = false, bool _dribbler_on = false) {
      vx = _vx; vy = _vy; va = _va;
      kicker_on = _kicker_on;  dribbler_on = _dribbler_on;
      eta = _eta;
    }
  };

  enum CommandType{
    CmdMoveBall,    // target, ball_target, angle_tolerance, ball_speed
    CmdSteal,       // target
    CmdDribble,     // target, angle
    CmdRecieveBall, // (no parameters)
    CmdPosition,    // target, velocity, angle, obs, goto_point_type
    CmdSpin         // target, ball_speed, obs
  };

  enum BallShotType{
    BallShotOnGoal,
    BallShotClear,
    BallShotPass
  };

  struct RobotCommand{
    CommandType cmd;
    int obs; // obstacle flags
    vector2d target,velocity;
    vector2d ball_target; // where we'd like the ball to go if moving ball
    double angle,angle_tolerance;
    BallShotType ball_shot_type;
    GotoPointType goto_point_type;
    // double ball_speed;
  };

// internal state structs
  enum SMState{
    SMGotoBall,
    SMFaceBall,
    SMApproachBall,
    SMPullBall,
    SMFaceTarget,
    SMDriveToGoal,
    SMKick,
    SMSpinAtBall,
    SMPosition,
    SMRecieveBall,
    SMWait
  };

// member variables
public:
  Sensors sensors;
  int my_id;

private:
  SMState state,last_state;
  CommandType last_cmd;

  vector2d target_rel;
  vector2d target_ball_rel;
  double state_start_time; // timestamp of when we first got in state
  double start_dist_from_ball;
  double time_in_state;
  double last_dist_from_target;
  double last_target_da;
  double initial_ball_dist; // used by approach ball
  int spin_dir;
  int ttl;
  double last_kick_timestamp;
  bool omni;
  bool state_changed;

public:
  void init(int _my_id);

  void updateSensors(World &world);

  SMState gotoBall    (World &world,Sensors &s,RobotCommand &cmd,NavTarget &nav);
  SMState faceBall    (World &world,Sensors &s,RobotCommand &cmd,NavTarget &nav);
  SMState approachBall(World &world,Sensors &s,RobotCommand &cmd,NavTarget &nav);
  SMState pullBall    (World &world,Sensors &s,RobotCommand &cmd,NavTarget &nav);
  SMState faceTarget  (World &world,Sensors &s,RobotCommand &cmd,NavTarget &nav);
  SMState driveToGoal (World &world,Sensors &s,RobotCommand &cmd,NavTarget &nav);
  SMState kick        (World &world,Sensors &s,RobotCommand &cmd,NavTarget &nav);
  SMState spinAtBall  (World &world,Sensors &s,RobotCommand &cmd,NavTarget &nav);
  SMState position    (World &world,Sensors &s,RobotCommand &cmd,NavTarget &nav);
  SMState recieveBall (World &world,Sensors &s,RobotCommand &cmd,NavTarget &nav);
  SMState wait        (World &world,Sensors &s,RobotCommand &cmd,NavTarget &nav);

  Status run(World &world,RobotCommand &cmd,Trajectory &tcmd);
  Status run(World &world,RobotCommand &cmd);
  double time(World &world,RobotCommand &cmd);

  // in goto-point.cc
  float motion_time_1d(float dx,float vel0,float vel1,
                       float max_vel,float max_accel,
                       float &t_accel,float &t_cruise,float &t_decel);
  double max_speed(double dx,double max_a);
  void compute_motion_1d(double x0, double v0, double v1,
                         double a_max, double v_max, double a_factor,
                         double &traj_accel, double &traj_time);
  void compute_motion_2d(vector2d x0, vector2d v0, vector2d v1,
                         double a_max, double v_max, double a_factor,
                         vector2d &traj_accel, double &time);
  double compute_stop(double v, double max_a);
  Trajectory goto_point(World &world, int me, 
                        vector2d target_pos, vector2d target_vel,
                        double target_ang,
			GotoPointType type = GotoPointMove);
  Trajectory goto_point_omni(World &world, int me, 
                             vector2d target_pos, vector2d target_vel,
                             double target_ang,
			     GotoPointType type = GotoPointMove);
  Trajectory goto_point_diff(World &world, int me, 
                             vector2d target_pos, vector2d target_vel,
                             double target_ang,
			     GotoPointType type = GotoPointMove);
  Trajectory nav_to_point(World &world, int me,
                          vector2d target_pos, vector2d target_vel,
                          double target_angle,int obs_flags,
			  GotoPointType type = GotoPointMove);
  Trajectory goto_point_speed(World &world, int me,
                              vector2d target_pos,vector2d target_vel,
                              double target_angle,
                              GotoPointType type = GotoPointMove);
  Trajectory spin_to_point(World &world, int me,
                           vector2d target_pos,
                           double target_ang_vel);
};

#endif /*__ROBOT_H__*/
