// world.h
//
// This class holds the entire state of the soccer system.  This includes
// the known state of the world as well as any required memory for the
// system.
//
// Any general state retrieval methods go in this class.
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

#include "constants.h"
#include "soccer.h"
#include "world.h"

#include "robot.h"

CR_DECLARE(DZONE_HYSTERESIS_DURATION);
CR_DECLARE(POSSESSION_US_HYSTERESIS_DURATION);
CR_DECLARE(POSSESSION_THEM_HYSTERESIS_DURATION);

static void cr_setup_do()
{
  static bool cr_setup = false;

  if (!cr_setup) {
    CR_SETUP(strategy, DZONE_HYSTERESIS_DURATION, CR_DOUBLE);
    CR_SETUP(strategy, POSSESSION_US_HYSTERESIS_DURATION, CR_DOUBLE);
    CR_SETUP(strategy, POSSESSION_THEM_HYSTERESIS_DURATION, CR_DOUBLE);

    cr_setup = true;
  } 
}

World::World()
{
  cr_setup_do();
}

void World::init(int _side, int _color)
{
  int i;

  side = _side;
  color = _color;

  // Basic Field Vectors
  our_goal = vector2d(-FIELD_LENGTH_H, 0.0);
  our_goal_l = vector2d(-FIELD_LENGTH_H, GOAL_WIDTH_H);
  our_goal_r = vector2d(-FIELD_LENGTH_H, -GOAL_WIDTH_H);
  their_goal = -our_goal;
  their_goal_l = -our_goal_r;
  their_goal_r = -our_goal_l;

  down_field = vector2d(1.0, 0);

  now = LATENCY_DELAY;

  game_state = 'S';

  // Robot Internal State
  for(i=0; i<MAX_TEAM_ROBOTS; i++){
    path[i].init(400,100,0.15,0.75,100);
    path[i].robot_id = i;

    robot[i] = new Robot;
    robot[i]->init(i);
  }

  // init the modeller data sructures
  modeller.initialize();

  // init roles
  orole_goalie = -1;
  trole_goalie = -1;
  trole_active = -1;
}

void World::update(const net_vframe &f)
{
  frame = f;
  time = f.timestamp;

  // Config
  tracker.SetConfig(f.config);

  // Update Ball Kalman Filter
  tracker.ball.reset(time, (float*) &f.ball.state, (float*) f.ball.variances,
		     (BallTracker::OccludeFlag) f.ball.occluded, 
		     f.ball.occluding_team, 
		     f.ball.occluding_robot, f.ball.occluding_offset);

  // Update Robot Filters
  n_teammates = 0; 
  n_opponents = 0; 

  for(int i=0; i<NUM_TEAMS; i++) {
    for(int j=0; j<MAX_TEAM_ROBOTS; j++) {
      if (!tracker.Exists(i, j)) continue;

      if (i == color) teammate_id_to_index[n_teammates++] = j;
      else opponent_id_to_index[n_opponents++] = j;

      tracker.robots[i][j].reset(time, (float *) &f.robots[i][j].state);
    }
  }

  // Game State
  goal_scored = 0;

  if (strchr("tTgGz", f.refstate) == NULL)
    game_state = f.refstate;
  else if (strchr("gG", f.refstate) != NULL && last_ref_state != f.refstate) {
    if ((f.refstate == 'g' && color == TEAM_YELLOW) ||
	(f.refstate == 'G' && color == TEAM_BLUE))
      goal_scored = 1;
    else 
      goal_scored = -1;
  }

  last_ref_state = f.refstate;

  // Update Robot Internal State
  for(int i=0; i<MAX_TEAM_ROBOTS; i++)
    robot[i]->updateSensors(*this);

  // Update High Level Information
  updateHighLevel();

  // update the modeller
  modeller.update(f);
}

void World::updateHighLevel() 
{
  static double last_time = -1;

  // Elapsed is set to the time passed since last call to updateHighLevel().
  double elapsed = (last_time < 0) ? 0.0 : time - last_time;
  last_time = time;

  vector2d ball = ball_position();

  // ball position
  double epsilon = 100.0;
  if (ball.x - epsilon > ballXThreshold) 
    ballXThreshold = ball.x - epsilon;
  else if (ball.x + epsilon < ballXThreshold) 
    ballXThreshold = ball.x + epsilon;

  if (ball.y - epsilon > ballYThreshold) 
    ballYThreshold = ball.y - epsilon;
  else if (ball.y + epsilon < ballYThreshold) 
    ballYThreshold = ball.y + epsilon;

  // fieldPosition
  if (ballXThreshold > 400) fieldPosition = TheirSide;
  else if (ballXThreshold < -400) fieldPosition = OurSide;
  else fieldPosition = Midfield;

  // possession
  double our_dist = 
    (teammate_position(nearest_teammate(ball)) - ball).length();
  double their_dist = 
    (opponent_position(nearest_opponent(ball)) - ball).length();

  if (our_dist > 180) possession_our_duration = 0;
  else if (their_dist > 180) possession_our_duration += elapsed;
  if (their_dist > 180) possession_their_duration = 0;
  else if (our_dist > 180) possession_their_duration += elapsed;

  if (possession_our_duration > DVAR(POSSESSION_US_HYSTERESIS_DURATION) &&
      possession_their_duration < DVAR(POSSESSION_THEM_HYSTERESIS_DURATION))
    possession = OurBall;
  else if (possession_our_duration < DVAR(POSSESSION_US_HYSTERESIS_DURATION) &&
	   possession_their_duration > DVAR(POSSESSION_THEM_HYSTERESIS_DURATION))
    possession = TheirBall;
  else if (possession_our_duration == 0 &&
	   possession_their_duration == 0)
    possession = LooseBall;

  // situation
  if (game_state == 's') {

    if (possession == TheirBall || fieldPosition == OurSide)
      situation = Defense;
    else if (possession == OurBall || fieldPosition == TheirSide ||
	     situation == Special)
      situation = Offense;
    else situation = situation;

  } else if (game_state != 'S') {
    situation = Special;
    if (World::restart())
      possession = restartWhoseKick();
  }

  // opponent roles
  //
  // goalie
  if (orole_goalie < 0 ||
      !obsPosition(opponent_position(orole_goalie), OBS_THEIR_DZONE)) {
    for(int i=0; i<n_opponents; i++) {
      if (i == orole_goalie) continue;
      if (obsPosition(opponent_position(i), OBS_THEIR_DZONE)) { 
	orole_goalie = i; break; 
      }
    }
  }

  if (orole_goalie > 0 && opponent_position(orole_goalie).x < 0)
    orole_goalie = -1;

  // defense zones
  if (obsPosition(ball_position(), OBS_OUR_DZONE, BALL_RADIUS))
    our_dzone_duration += elapsed;
  else our_dzone_duration = 0.0;

  if (obsPosition(ball_position(), OBS_THEIR_DZONE, BALL_RADIUS)) 
    their_dzone_duration += elapsed;
  else their_dzone_duration = 0.0;
}

vector2d World::ball_position(double t)
{
  if (t < 0) t = now;
  return tracker.ball.position(t) * side;
}

vector2d World::ball_velocity(double t)
{
  if (t < 0) t = now;
  return tracker.ball.velocity(t) * side;
}

Matrix World::ball_covariances(double t)
{
  if (t < 0) t = now;
  return tracker.ball.covariances(t);
}

int World::teammate_type(int id)
{
  return frame.config.teams[color].robots[teammate_id_to_index[id]].type;
}

double World::teammate_radius(int id)
{
  if (teammate_type(id) == ROBOT_TYPE_DIFF) return DIFFBOT_RADIUS;
  else return OMNIBOT_RADIUS;
}

int World::teammate_frame_index(int id)
{
  return teammate_id_to_index[id];
}

vector2d World::teammate_position(int id, double t)
{
  if (t < 0) t = now;
  return tracker.robots[color][teammate_id_to_index[id]].position(t) * side;
}

vector2d World::teammate_velocity(int id, double t)
{
  if (t < 0) t = now;
  return tracker.robots[color][teammate_id_to_index[id]].velocity(t) * side;
}

vector2d World::teammate_robot_velocity(int id, double t)
{
  return teammate_velocity(id, t).rotate(-teammate_direction(id, t));
}

double World::teammate_direction(int id, double t)
{
  if (t < 0) t = now;
  return anglemod( (side < 0 ? M_PI : 0.0) + 
		   tracker.robots[color][teammate_id_to_index[id]].direction(t));
}

double World::teammate_angular_velocity(int id, double t)
{
  if (t < 0) t = now;
  return tracker.robots[color][teammate_id_to_index[id]].angular_velocity(t);
}

double World::teammate_stuck(int id)
{
  return tracker.robots[color][teammate_id_to_index[id]].stuck(0.0);
}

void World::teammate_command(int id, double vx, double vy, double va)
{
  tracker.robots[color][teammate_id_to_index[id]].command(time, vector3d(vx, vy, va));
}

vector2d World::opponent_position(int id, double t)
{
  if (t < 0) t = now;
  return tracker.robots[!color][opponent_id_to_index[id]].position(t) * side;
}

vector2d World::opponent_velocity(int id, double t)
{
  if (t < 0) t = now;
  return tracker.robots[!color][opponent_id_to_index[id]].velocity(t) * side;
}

void World::teammate_raw(int id, vraw &vpos)
{
  vpos = frame.robots[color][teammate_id_to_index[id]].vision;

  vpos.pos *= -1;
  vpos.angle = anglemod(vpos.angle + M_PI);
}

void World::opponent_raw(int id, vraw &vpos)
{
  vpos = frame.robots[!color][opponent_id_to_index[id]].vision;

  vpos.pos *= -1;
  vpos.angle = anglemod(vpos.angle + M_PI);
}

double World::ball_raw(vraw &vpos)
{
  vpos = frame.ball.vision;

  vpos.pos *= -1;
  vpos.angle = anglemod(vpos.angle + M_PI);

  return((time > frame.ball.vision.timestamp)?
         0.0 : frame.ball.vision.conf);
}

int World::ball_collision(double t)
{
  if (t < 0) t = now;

  int team, robot;

  if (tracker.ball.collision(t, team, robot)) {
    if (team == color) return OBS_TEAMMATE(robot);
    else if (team == !color) return OBS_OPPONENT(robot);
  }
  
  return 0;
}

void World::go(int id, double vx, double vy, double va, 
	       bool kicker_on, bool dribbler_on)
{
  if (isnan(vx) || isnan(vy) || isnan(va)) {
    fprintf(stderr, "WARNING NAN: World::go(id = %d)\n", id);
    return;
  }

#if 0
  if (fabs(vx) < 10.0 && fabs(vy) < 10.0 && fabs(va) < 0.0001 && 
      !kicker_on && !dribbler_on) {
    fprintf(stderr, "HALT %d: %f %f %f\n", id, vx, vy, va);
    return halt(id);
  } else 
    fprintf(stderr, "NOHALT %d: %f %f %f\n", id, vx, vy, va);
#endif


  teammate_command(id, vx, vy, va);
  radio_send(id, vx, vy, va, kicker_on, dribbler_on);
}

void World::halt(int id)
{
  teammate_command(id, 0, 0, 0);
  radio_halt(id);
}

bool World::ballOutOfPlay(double time)
{
  vector2d ball = ball_position(time);
  return (fabs(ball.x) - BALL_DIAMETER > FIELD_LENGTH_H ||
	  fabs(ball.y) - BALL_DIAMETER > FIELD_WIDTH_H);
}

vector2d World::freeKickPosition(vector2d ball)
{
  if (fabs(ball.x) > FIELD_LENGTH_H - FREEKICK_FROM_GOAL) {
    ball.x = copysign(FIELD_LENGTH_H - FREEKICK_FROM_GOAL, ball.x);
    ball.y = copysign(FIELD_WIDTH_H - FREEKICK_FROM_WALL, ball.y);
  } else if (fabs(ball.y) > FIELD_WIDTH_H - FREEKICK_FROM_WALL)
    ball.y = copysign(FIELD_WIDTH_H - FREEKICK_FROM_WALL, ball.y);

  return ball;
}

int World::sideBall()
{
  return (ballYThreshold > 0.0 ? 1 : -1);
}

int World::sideStrong()
{
  double center = 0.0;
  for(int i=0; i < n_opponents; i++)
    center += opponent_position(i).y;
  return (center > 0.0 ? 1 : -1);
}

int World::sideBallOrStrong()
{
  if (fabs(ball_position().y) > GOAL_WIDTH_H) return sideBall();
  else return sideStrong();
}

bool World::inOurDefenseZone()
{
  return (our_dzone_duration > DVAR(DZONE_HYSTERESIS_DURATION));
}


bool World::inTheirDefenseZone()
{
  return (their_dzone_duration > DVAR(DZONE_HYSTERESIS_DURATION));
}

bool World::twoDefendersInTheirDZone()
{
  int n = 0;

  for(int i=0; i<n_opponents; i++)
    if (obsPosition(opponent_position(i), OBS_THEIR_DZONE, 0))
      n++;

  return (n >= 2);
}

bool World::restart()
{
  return (strchr("kKpPfF ", game_state) != NULL);
}

bool World::restartPenalty()
{
  return (strchr("pP ", game_state) != NULL);
}

bool World::restartNeutral()
{
  return (game_state == COMM_RESTART);
}

World::Possession World::restartWhoseKick()
{
  bool blue = (strchr("KPF", game_state) != NULL);
  bool yellow = (strchr("kpf", game_state) != NULL);

  if ((blue && color == TEAM_BLUE) ||
      (yellow && color == TEAM_YELLOW)) return OurBall;
  else if (!blue && !yellow) return LooseBall;
  else return TheirBall;
}


int World::nearest_teammate(vector2d p, double time)
{
  int dist_i = -1;
  double dist = 0;

  for(int i=0; i<n_teammates; i++) {
    double d = (p - teammate_position(i, time)).length();
    if (dist_i < 0 || d < dist) {
      dist_i = i; dist = d;
    }
  }

  return dist_i;
}

int World::nearest_opponent(vector2d p, double time)
{
  int dist_i = -1;
  double dist = 0;

  for(int i=0; i<n_opponents; i++) {
    double d = (p - opponent_position(i, time)).length();
    if (dist_i < 0 || d < dist) {
      dist_i = i; dist = d;
    }
  }

  return dist_i;
}

int World::obsPosition(vector2d p, int obs_flags, 
		       double pradius, double time = -1)
{
  int rv = 0;

  // Teammates
  for(int i=0; i<n_teammates; i++) { 
    if (!(obs_flags & OBS_TEAMMATE(i))) continue;

    double radius = TEAMMATE_EFFECTIVE_RADIUS + pradius; 

    if ((p - teammate_position(i, time)).length() <= radius) 
      rv |= OBS_TEAMMATE(i);
  }

  // Opponents
  for(int i=0; i<n_opponents; i++) {
    if (!(obs_flags & OBS_OPPONENT(i))) continue;

    double radius = OPPONENT_EFFECTIVE_RADIUS + pradius;

    if ((p - opponent_position(i, time)).length() <= radius) 
      rv |= OBS_OPPONENT(i);
  }

  // Ball
  if (obs_flags & OBS_BALL) {
    double radius = BALL_RADIUS + pradius;
    if ((p - ball_position(time)).length() <= radius) 
      rv |= OBS_BALL;
  }

  // Walls
  if (obs_flags & OBS_WALLS) {
    double radius = pradius;
    if (fabs(p.x) + radius > FIELD_LENGTH_H || 
	fabs(p.y) + radius > FIELD_WIDTH_H)
      rv |= OBS_BALL;
  }
  
  // Defense Zones
  if (obs_flags & OBS_OUR_DZONE) {
    double radius = pradius;
    if (p.x <= -FIELD_LENGTH_H + DEFENSE_DEPTH + radius &&
	fabs(p.y) <= DEFENSE_WIDTH_H + radius) 
      rv |= OBS_OUR_DZONE; 
  }

  if (obs_flags & OBS_THEIR_DZONE) {
    double radius = pradius;
    if (p.x >= FIELD_LENGTH_H - DEFENSE_DEPTH - radius &&
	fabs(p.y) <= DEFENSE_WIDTH_H + radius) 
      rv |= OBS_THEIR_DZONE; 
  }

  // Nothing Left
  return rv;
}

int World::obsLine(vector2d p1, vector2d p2, int obs_flags,
		   double pradius, double time)
{
  // Teammates
  for(int i=0; i<n_teammates; i++) { 
    if (!(obs_flags & OBS_TEAMMATE(i))) continue;

    double radius = TEAMMATE_EFFECTIVE_RADIUS + pradius; 

    vector2d p = teammate_position(i, time);
    if ((point_on_segment(p1, p2, p) - p).length() > radius)
      obs_flags &= ~OBS_TEAMMATE(i);
  }

  // Opponents
  for(int i=0; i<n_opponents; i++) {
    if (!(obs_flags & OBS_OPPONENT(i))) continue;

    double radius = OPPONENT_EFFECTIVE_RADIUS + pradius;

    vector2d p = opponent_position(i, time);
    if ((point_on_segment(p1, p2, p) - p).length() > radius)
      obs_flags &= ~OBS_OPPONENT(i);
  }

  // Ball
  if (obs_flags & OBS_BALL) {
    double radius = BALL_RADIUS + pradius;

    vector2d p = ball_position(time);
    if ((point_on_segment(p1, p2, p) - p).length() > radius)
      obs_flags &= ~OBS_BALL;
  }

  // Walls
  
  // Defense Zones

  // Nothing Left
  return obs_flags;
}

int World::obsLineFirst(vector2d p1, vector2d p2, int obs_flags,
			vector2d &first, double pradius, double time = -1)
{
  int rv = 0;

  first = p2;

  // Teammates
  for(int i=0; i<n_teammates; i++) { 
    if (!(obs_flags & OBS_TEAMMATE(i))) continue;

    double radius = TEAMMATE_EFFECTIVE_RADIUS + pradius; 

    vector2d p = teammate_position(i, time);
    vector2d pp = point_on_segment(p1, first, p);
    double d = (pp - p).length();

    if (d < radius) {
      double dx = sqrt(radius * radius - d * d);
      
      if ((p1 - pp).length() < dx) { first = p1; return OBS_TEAMMATE(i); }
      else {
	first = pp + (p1 - pp).norm(dx);
	rv = OBS_TEAMMATE(i);
      }
    }
  }

  // Opponents
  for(int i=0; i<n_opponents; i++) {
    if (!(obs_flags & OBS_OPPONENT(i))) continue;

    double radius = OPPONENT_EFFECTIVE_RADIUS + pradius; 

    vector2d p = opponent_position(i, time);
    vector2d pp = point_on_segment(p1, first, p);
    double d = (pp - p).length();

    if (d < radius) {
      double dx = sqrt(radius * radius - d * d);
      
      if ((p1 - pp).length() < dx) { first = p1; return OBS_OPPONENT(i); }
      else {
	first = pp + (p1 - pp).norm(dx);
	rv = OBS_OPPONENT(i);
      }
    }
  }

  // Ball
  if (obs_flags & OBS_BALL) {
    double radius = BALL_RADIUS + pradius;

    vector2d p = ball_position(time);
    vector2d pp = point_on_segment(p1, first, p);
    double d = (pp - p).length();

    if (d < radius) {
      double dx = sqrt(radius * radius - d * d);
      
      if ((p1 - pp).length() < dx) { first = p1; return OBS_BALL; }
      else {
	first = pp + (p1 - pp).norm(dx);
	rv = OBS_BALL;
      }
    }
  }

  // Walls
  
  // Defense Zones
  if (obs_flags & OBS_THEIR_DZONE) {
    if (obsPosition(p1, OBS_THEIR_DZONE, pradius, time)) {
      first = p1; return OBS_THEIR_DZONE;
    }

    vector2d i;

    i = intersection(p1, p2, 
		     vector2d(FIELD_LENGTH_H-DEFENSE_DEPTH-pradius, 
			      DEFENSE_WIDTH_H+pradius),
		     vector2d(FIELD_LENGTH_H-DEFENSE_DEPTH-pradius, 
			      -DEFENSE_WIDTH_H-pradius));
    if ((i - p1).dot(first - p1) > 0 && (i - first).dot(p1 - first) > 0) {
      first = i; rv = OBS_THEIR_DZONE;
    }

    i = intersection(p1, p2, 
		     vector2d(FIELD_LENGTH_H-DEFENSE_DEPTH-pradius, 
			      DEFENSE_WIDTH_H+pradius),
		     vector2d(FIELD_LENGTH_H, DEFENSE_WIDTH_H+pradius));
    if ((i - p1).dot(first - p1) > 0 && (i - first).dot(p1 - first) > 0) {
      first = i; rv = OBS_THEIR_DZONE;
    }

    i = intersection(p1, p2, 
		     vector2d(FIELD_LENGTH_H-DEFENSE_DEPTH-pradius, 
			      -DEFENSE_WIDTH_H-pradius),
		     vector2d(FIELD_LENGTH_H, -DEFENSE_WIDTH_H-pradius));
    if ((i - p1).dot(first - p1) > 0 && (i - first).dot(p1 - first) > 0) {
      first = i; rv = OBS_THEIR_DZONE;
    }
  }


  // Nothing Left
  return obs_flags;
}

int World::obsLineNum(vector2d p1, vector2d p2, int obs_flags, 
		      double pradius, double time = -1)
{
  int count = 0;

  // Teammates
  for(int i=0; i<n_teammates; i++) { 
    if (!(obs_flags & OBS_TEAMMATE(i))) continue;

    double radius = TEAMMATE_EFFECTIVE_RADIUS + pradius; 

    vector2d p = teammate_position(i, time);
    if ((point_on_segment(p1, p2, p) - p).length() <= radius)
      count++;
  }

  // Opponents
  for(int i=0; i<n_opponents; i++) {
    if (!(obs_flags & OBS_OPPONENT(i))) continue;

    double radius = OPPONENT_EFFECTIVE_RADIUS + pradius;

    vector2d p = opponent_position(i, time);
    if ((point_on_segment(p1, p2, p) - p).length() <= radius)
      count++;
  }

  // Ball
  if (obs_flags & OBS_BALL) {
    double radius = BALL_RADIUS + pradius;

    vector2d p = ball_position(time);
    if ((point_on_segment(p1, p2, p) - p).length() <= radius)
      count++;
  }

  // Walls
  
  // Defense Zones

  // Nothing Left
  return count;
}

bool World::obsBlocksShot(vector2d p, double pradius, double time)
{
  vector2d ball = ball_position(time);

  double a = (their_goal_r - ball).perp().dot(p - ball);
  double b = (their_goal_l - ball).perp().dot(p - ball);

  return (a * b) < 0;
}


int World::choosePenaltyKicker(void)
{
  int opponentid = -1;
  double minx = 0.0;
  vector2d rpos;

  // for a penalty kick we know that they have to be behind
  // the kicker
  for (int i = 0; i < n_opponents; i++) {
    rpos = opponent_position(i);
    if (rpos.x < minx) {
      opponentid = i;
      minx = rpos.x;
    }
  }
  return (opponentid);
}


