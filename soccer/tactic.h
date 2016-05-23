// tactic.h
// 
// Parent class for tactics.
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

#ifndef __tactic_h__
#define __tactic_h__

#include <deque>
#include <vector>

#include "soccer.h"
#include "world.h"
#include "robot.h"

class TCoordinate {
public:
  enum otype { OAbsolute, OBall };
  enum stype { SAbsolute, SBall, SStrong, SBallOrStrong, SGui };

private:
  otype origin;
  stype side;
  bool dynamic;
  bool absolute;

  vector2d c;

  vector2d asVectorNotAbsolute(World &w);
  
public:
  TCoordinate(double x, double y,
	      stype _side = SAbsolute, 
	      otype _origin = OAbsolute,
	      bool _dynamic = false) {
    c = vector2d(x, y);
    side = _side; origin = _origin; dynamic = _dynamic; 
    absolute = (origin == OAbsolute && side == SAbsolute); }

  TCoordinate(vector2d _p = vector2d(0, 0), 
	      stype _side = SAbsolute, 
	      otype _origin = OAbsolute,
	      bool _dynamic = false) {
    c = _p;
    side = _side; origin = _origin; dynamic = _dynamic; 
    absolute = (origin == OAbsolute && side == SAbsolute); }

  vector2d asVector(World &w) {
    if (absolute) return c;
    else return asVectorNotAbsolute(w);
  }

  double asDirection(World &w) {
    return asVector(w).angle(); }

  vector2d getVelocity(World &w) {
    if (dynamic && origin == OBall) return w.ball_velocity();
    else return vector2d(0, 0); }
};

class TRegion {
private:
  enum { Circle,  Rectangle } type;
  
  TCoordinate p[2];
  
  double radius;
  
public:
  TRegion() { type = Circle; radius = 0; }
  TRegion(TCoordinate p1, TCoordinate p2, double _radius) {
    type = Rectangle; p[0] = p1; p[1] = p2; radius = _radius; }
  TRegion(TCoordinate p1, double _radius) {
    type = Circle; p[0] = p1; radius = _radius; }

  vector2d center(World &w);
  vector2d sample(World &w); 

  vector2d centerVelocity(World &w);

  void diagonal(World &w, vector2d p, vector2d &d1, vector2d &d2);

  bool inRegion(World &w, vector2d p);

  void gui_draw(int robot, int level, World &world);
};

typedef vector<int> TRoleMap;

class Tactic {
public:
  virtual const char *name() const { return "Unknown Tactic"; }

private:
  // These things may be set to give a broader context for the tactic.
  // See, e.g., getTeammateId() and getObsFlags().
  TRoleMap *teammate_map, *opponent_map;
  int priority;

public:
  // Constructor
  Tactic(bool _active = false, bool _manipulates_ball = false) {
    active = _active;
    manipulates_ball = _manipulates_ball;
    teammate_map = opponent_map = NULL; 
    priority = 0;
  }

  // Register
  //
  // This allows a tactic to be associated with a string and parsing
  // routine so that ascii commands can be converted into tactic classes
  // to be executed.  Useful for gui specification of tactics.
  //
  typedef Tactic *(*parse_fn)(const char *param_string);
  static bool registerParser(const char *name, parse_fn parser);
  static Tactic *parse(const char *string);

  // Clone
  // 
  // Each subclass must define a clone method.  For most cases the following
  // just needs to be added in the class definition:  
  //
  // virtual Tactic *clone() const { return new NewTactic(*this); }
  //
  virtual Tactic *clone() const = 0;

  // Role Mapping
  //
  // This provides a mechanism for referring to robots indirectly.  Id's
  // are passed through the tactic's role map to get a robot id.
  //
  // You can also grab an obs_flags int with all teammates higher in
  // the rolemap than the tactic's priority.  This can then be used to
  // yield to higher priority tactics when, e.g., finding good
  // positions on the field.
  //
  void setTeammateMap(TRoleMap *m) { teammate_map = m; }
  void setOpponentMap(TRoleMap *m) { opponent_map = m; }

  int getTeammateId(int id) { return teammate_map ? (*teammate_map)[id] : id; }
  int getOpponentId(int id) { return opponent_map ? (*opponent_map)[id] : id; }

  int getObsFlags() { 
    if (!teammate_map) return 0;

    int obs_flags = 0;
    for(int i=0; i<priority ; i++)
      obs_flags |= OBS_TEAMMATE(getTeammateId(i));

    return obs_flags; }

  void setPriority(int _priority) {
    priority = _priority; }
  
  //
  // Tactic specified methods and fields.
  //
  
  bool active;  // Is true if this is an active tactic.
  bool manipulates_ball; // Is true if tactic manipulates the ball.

  // Returns the robot most apt to accomplish the tactic.
  // By default this is the robot with the lowest estimated time modulo
  //  the provided time bias.
  virtual int selectRobot(World &world, bool candidates[], double bias[]);

  int selectRobot(World &world, bool candidates[]) {
    static double bias[MAX_TEAM_ROBOTS] = {0};
    return selectRobot(world, candidates, bias); }

  // Returns the time to accomplish the tactic.
  virtual double estimatedTime(World &world, int me) { return 0.0; }

  // Returns the status of the tactic.  
  virtual Status isDone(World &world, int me) { return Succeeded; }

  // Performs the tactic and returns the motor traj.
  virtual void run(World &world, int me) = 0;

private:

  struct registration {
    const char *name;
    parse_fn parser;
  };
  
  static deque<registration> registrar;
};

class RobotTactic : public Tactic {
protected:
  Status the_status;

  void makeCommand(World &world, int me, bool debug, 
		   Robot::RobotCommand &command,
		   bool &ignore_status);

public:
  RobotTactic(bool _active = false, bool _manipulates_ball = false) : 
    Tactic(_active, _manipulates_ball) { 
    the_status = InProgress; }

  virtual void command(World &world, int me, Robot::RobotCommand &command,
		       bool debug) { 
    command.cmd = Robot::CmdPosition;
    command.target = world.teammate_position(me);
    command.velocity = vector2d(0, 0);
    command.angle = world.teammate_direction(me);
    command.obs = 0;
  }

  // This is slightly inefficient.  Both estimated_time() and doit()
  // generate the command afresh.  But since the command may be robot
  // dependent it's not something one can easily cache.
  //
  // Also debugging output is not printed on call from estimatedTime()
  // but are on the call from run().

public:
  virtual double estimatedTime(World &world, int me) {
    Robot::RobotCommand the_command;
    bool ignore_status;

    makeCommand(world, me, false, the_command, ignore_status);
    return world.robot[me]->time(world, the_command); }

  virtual Status isDone(World &world, int me) {
    return the_status; }

  virtual void run(World &world, int me) {
    Robot::RobotCommand the_command;
    bool ignore_status;

    makeCommand(world, me, true, the_command, ignore_status);
    the_status = world.robot[me]->run(world, the_command); 
    if (ignore_status) the_status = InProgress;
  }
};

#endif
