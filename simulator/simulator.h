/*
 * TITLE:  Simulator.h
 *
 * PURPOSE:  This file implements the small size robot simulator classes.
 *
 * WRITTEN BY:	Brett Browning, Michael Bowling
 */
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

#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__


#include <pthread.h>
#include <bits/pthreadtypes.h>
#include <semaphore.h>

#include "constants.h"
#include "geometry.h"
#include "../utils/polygon.h"
#include "vqueue.h"
#include "../include/rtypes.h"

/**************************** TYPES *********************************/


/* maximum number of edges on a robot */
#define MAX_RPOINTS	 10


/* running states for the simulator */
#define RUNSTATE_PAUSE         0
#define RUNSTATE_STEPFORWARD   1
#define RUNSTATE_PLAY          2

/* struct to encapsulate the simulated robot information */
typedef struct {
  int   id;
  bool  yellow;
  bool  opponent;
  int   rtype;  
  bool  collision;
  Polygon  polygon;	
  int  nr_points;
  RPosition pos, oldpos;
  RVelocity  vcmd, vel;
  VelocityQueue	vqueue;
  double conf;
  bool kick, drib;
} SimRobot;


/* struct to capture the simulated ball data */
typedef struct {
  vector2d vel, pos, oldpos;
  double conf;
} SimBall;



/**************************** CLASSES *******************************/

/*
 * RobotSimulator -
 *
 * this class contains the simulator.
 */
class Simulator {
private:
  void Tick();

  pthread_t run_thread;

  unsigned long int rthread;

  sem_t commands_mutex;
  sem_t runstate_mutex;
  sem_t update_flag;

  int num_yellow, num_blue;
  
  SimBall  ball;
  SimRobot blue_robots[MAX_TEAM_ROBOTS];
  SimRobot yellow_robots[MAX_TEAM_ROBOTS];
	
  int frameCount;
  bool usegoals;
  bool usenoise;
  vector2d field_max;

  Polygon field_boundary;
  int goalscored;

public:

  Simulator();
  ~Simulator();

  // Initializer
  //  static bool Initialize(void);
  bool Initialize(bool _usegoals = false, bool _usenoise = false);
  static void Run(Simulator *sim);

  // Add Robot/Opponents
  bool AddRobot(bool yellow, int rtype, double x = 0, double y = 0, double a = 0);
  bool AddRobot(bool yellow, int rtype, vector2d pos, double a = 0)
  	{ return (AddRobot(yellow, rtype, pos.x, pos.y, a)); }
  	
  bool SetNumRobots(bool yellow, int n);

  // Retrieve state.
  double Time(void);
  int Frame(void);
  int GetNumRobots(bool yellow) { 
    return (IS_YELLOW(yellow, num_yellow, num_blue));
  }
  vector2d GetRobotPosition(bool yellow, int robot);
  double GetRobotDirection(bool yellow, int robot);
  vector2d GetRobotVelocity(bool yellow, int robot);
  double GetRobotAngularVelocity(bool yellow, int robot);
  int GetRobotType(bool yellow, int robot);
  double GetRobotConfidence(bool yellow, int robot);
  vector2d GetBallPosition(void);
  vector2d GetBallVelocity(void);
  double GetBallConfidence(void);

  int IsGoal(void) {
    return (goalscored);
  }
  
  // Set state.
  void SetBall(vector2d p, vector2d v);
  void SetBallPosition(vector2d p);
  void SetBallVelocity(vector2d v);
  void StopBall(void);

  void SetRobot(bool yellow, int robot, vector2d p, double a, vector2d v, double va);
  void SetRobot(bool yellow, int robot, vector2d p, double a) {
    SetRobot(yellow, robot, p, a, vector2d(0,0), 0.0);
  }
  void SetRobotRandom(bool yellow, int robot);

  // Set actions by robots.
  void SetRobotCommand(bool yellow, int robot, double vx, double vy, double va,
		       bool kick = false, bool drib = false);
  void SetRobotCommand(bool yellow, int robot, vector2d v, double va,
		       bool kick = false, bool drib = false) { 
    SetRobotCommand(yellow, robot, v.x, v.y, va); 
  }
  
  // stop everything dead
  void StopAll(void);

  // wait for the next update
  void wait_for_update(void);

public:

  // Running State
  char running_state;

  void SetRunState(char s) { 
    sem_wait(&runstate_mutex); running_state = s; sem_post(&runstate_mutex); }

  void Pause(void) {
  	SetRunState(RUNSTATE_PAUSE); 
  }
  void Step(void) { 
  	SetRunState(RUNSTATE_STEPFORWARD); 
  }
  void Play(void) { 
  	SetRunState(RUNSTATE_PLAY); 
  }
private:
  vector2d rand_posvector(void);
  double rand_heading(void);
  void SetPolygon(SimRobot &r);
  void AccelerationLimit(SimRobot &sr, double maxdv);
  void SpeedLimit(SimRobot &sr, double maxv);
  bool CheckWallCollision(SimRobot &r, double minx, double maxx, double miny, double maxy);
  void UpdateRobot(SimRobot &r, double dt);
  bool RobotBallCollisionPolygon(SimRobot &r, SimBall &b, double dt);
  bool RobotBallCollisionRadius(SimRobot &r, SimBall &b, double dt);
};



#endif /* __SIMULATOR_H__ */




