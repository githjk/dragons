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

#ifndef __world_h__
#define __world_h__

#include <reality/net_vision.h>
#include <ball_tracker.h>
#include <robot_tracker.h>

#include "path_planner.h"

#include "modeller.h"

#include "commands.h"

class Robot;

class World {
private:
  net_vframe frame;

  VTracker tracker;
  int teammate_id_to_index[MAX_TEAM_ROBOTS];
  int opponent_id_to_index[MAX_TEAM_ROBOTS];

  void updateHighLevel();

  Modeller modeller;

  double our_dzone_duration;
  double their_dzone_duration;

  double possession_our_duration;
  double possession_their_duration;

  char last_ref_state;

public:
  // Constructors & Destructors
  World();

  // Initialize
  void init(int _side, int _color);

  // Update
  void update(const net_vframe &f);

  // Team and Side
  char color;
  char side;

  // Time information
  double time; // The timestamp corresponding to the world's state.

  double now; // This is the time it will be before any robot would
              //  receive a command from the system.  This predicts
              //  ahead our latency.
              // This is substituded as the time argument if the time
              //  argument is not specified or is negative.

  // Basic Field Information
  vector2d our_goal, their_goal;
  vector2d our_goal_r, our_goal_l;
  vector2d their_goal_r, their_goal_l;
  
  vector2d down_field;

  inline vector2d to_world(const vector2d x) { 
    return x * side; }
  inline double to_world_dir(const double x) { 
    return anglemod((side < 0 ? M_PI : 0.0) + x); }
  inline vector2d from_world(const vector2d x) {
    return x * side; }
  inline double from_world_dir(const double x) {
    return (side < 0 ? M_PI : 0.0) + x; }

  // Basic Ball Information
  vector2d ball_position(double time = -1);
  vector2d ball_velocity(double time = -1);
  Matrix ball_covariances(double time = -1);
  double ball_raw(vraw &vpos);

  int ball_collision(double time = -1);

  // Basic Teammate Information
  int n_teammates;
  int teammate_type(int id);
  double teammate_radius(int id);
  int teammate_frame_index(int id);
  vector2d teammate_position(int id, double time = -1);
  vector2d teammate_velocity(int id, double time = -1); // Not relative
  vector2d teammate_robot_velocity(int id, double time = -1);

  double teammate_direction(int id, double time = -1);
  double teammate_angular_velocity(int id, double time = -1);

  double teammate_stuck(int id);

  void teammate_raw(int id, vraw &vpos);

  void teammate_command(int id, double vx, double vy, double va);

  // Basic Opponent Information
  int n_opponents;
  vector2d opponent_position(int id, double time = -1);
  vector2d opponent_velocity(int id, double time = -1);
  void opponent_raw(int id, vraw &vpos);

  // Game State (from Referee)
  char game_state;
  char goal_scored; // 1: our goal, -1: their goal, 0: no goal

  // Send Command to Robot
  void go(int id, double vx, double vy, double va, 
	  bool kicker_on = false, bool dribbler_on = false);
  void halt(int id);

  // Robot internal state
  Robot *robot[MAX_TEAM_ROBOTS];
  path_planner path[MAX_TEAM_ROBOTS];

  /////////////////////////////////////////////////////////////////
  //
  // High-Level Information
  //
  // Useful general computations about the state of the world.
  //

  /////////////////////////
  // Predicate Information

  enum Possession { TheirBall, LooseBall, OurBall };
  enum FieldPosition { TheirSide, Midfield, OurSide };
  enum Situation { Defense, Offense, Special };
  
  Possession possession;
  FieldPosition fieldPosition;
  Situation situation;

  // This is related to the ball's current position but with
  // positional hysteresis.
  double ballXThreshold, ballYThreshold;

  // Returns -1 or +1 corresponding to which side has the ball, or is
  // the opponent's strong side.  Uses above hysteresis thresholds.
  int sideBall();
  int sideStrong();
  int sideBallOrStrong();

  // Hysteresis defense zone booleans.  Uses a timed hysteresis.
  bool inOurDefenseZone();
  bool inTheirDefenseZone();

  /////////////////////////
  // Roles

  // Opponent Roles
  int orole_goalie;

  // Teammate Roles
  int trole_goalie;
  int trole_active;

  bool twoDefendersInTheirDZone();

  /////////////////////////
  // Restarts

  // Is the current state a restart?  Is it our kick?
  bool restart();
  bool isGameRunning() {
    return (game_state == COMM_START);
  }
  bool restartPenalty();
  bool restartNeutral();
  Possession restartWhoseKick();

  // Is the ball out of play?
  // Where would the free kick likely be taken given a last ball position.
  bool ballOutOfPlay(double time = -1);
  vector2d freeKickPosition(vector2d last_ball_position);

  /////////////////////////
  // Obstacle Computations

  // Finds the nearest teammate/opponent to a point on the field.
  int nearest_teammate(vector2d p, double time = -1);
  int nearest_opponent(vector2d p, double time = -1);

  // Obs methods return an obs_flag set to why a position or other
  // shape is not open.  Or zero if the position or shape is open
  //
  // obsLineNum() returns the number of obstacles on the line.
  //
  // obsBlocksShot() returns wether a point would block a shot right
  // now.

  int obsPosition(vector2d p, int obs_flags,
		  double pradius = TEAMMATE_EFFECTIVE_RADIUS, 
		  double time = -1);
  int obsLine(vector2d p1, vector2d p2, int obs_flags,
	      double pradius = TEAMMATE_EFFECTIVE_RADIUS, double time = -1);
  int obsLineFirst(vector2d p1, vector2d p2, int obs_flags,
		   vector2d &first, 
		   double pradius = TEAMMATE_EFFECTIVE_RADIUS, 
		   double time = -1);
  int obsLineNum(vector2d p1, vector2d p2, int obs_flags, 
		 double pradius = TEAMMATE_EFFECTIVE_RADIUS, double time = -1);

  bool obsBlocksShot(vector2d p, 
		     double pradius = TEAMMATE_EFFECTIVE_RADIUS,
		     double time = -1);

  int choosePenaltyKicker(void);

  /////////////////////////
  // Miscellaneous

  // Given a direction returns either it or the opposite way, whichever
  //  is closest to the robot's current direction.
  double teammate_nearest_direction(int id, double d) {
    if (fabs(anglemod(teammate_direction(id) - d)) > M_PI_2) 
      return anglemod(d + M_PI);
    else
      return d;
  }

private:
};

#endif
