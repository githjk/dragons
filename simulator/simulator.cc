/*
 * TITLE:  Simulator.cc
 *
 * PURPOSE:  This file implements the small size robot simulator classes.
 *
 * WRITTEN BY:  Scott Lenser, Michael Bowling, Brett Browning
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

#include <stdio.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <fstream.h>

#include <pthread.h>
#include <semaphore.h>

#include <constants.h>

#include <lineseg.h>
#include <polygon.h>
#include <util.h>
#include "simulator.h"
#include "vqueue.h"
#include "../utils/util.h"

#include "utils/configreader.h"

//#define DEBUG
//#define SIM_THREADED

#define USE_VQUEUE

// temp
#define FSGN(a)    (((a) >= 0) ? 1 : -1)

/**************************** TYPES ******************************************/


/* number of verticies for the different robot classes -- needs fix */
#define NR_DIFF_POINTS		6
#define NR_OMNI_POINTS		6

/* polygons for different robot types 
 * -  must be in clockwise order
 * - first side has kicker
 * - TYPE none is a circle and is not a polygon
 */

// diff drive
///const double RobotSimulator::robotPointsX[numRobotPoints] =
///  {60,  60, -35, -60, -60, -35};
///const double RobotSimulator::robotPointsY[numRobotPoints] =
///  {75, -75, -75, -50,  50,  75};
// inflated by size of ball as approximation (ball radius 21mm)
const double diffpoints_x[NR_DIFF_POINTS] =
  {81,  81, -56, -81, -81, -56};
const double diffpoints_y[NR_DIFF_POINTS] =
  {96, -96, -96, -71,  71,  96};

// omnidirectional
///const double RobotSimulator::robotPointsX[numRobotPoints] =
///  { 60.6,  60.6,  25.98, -86.6, -86.6,  25.98};
///const double RobotSimulator::robotPointsY[numRobotPoints] =
///  { 65.0, -65.0, -85.0 ,  -2.0,   2.0,  85.0};
// inflated by size of ball as approximation (ball radius 21mm)
const double omnipoints_x[NR_OMNI_POINTS] =
  { 83.80,  83.80,   35.95, -119.75, -119.75,   35.95};
const double omnipoints_y[NR_OMNI_POINTS] =
  { 89.89, -89.89, -117.5 ,  -27.63,   27.63,  117.5 };

#define NR_BOUNDARY_POINTS 12
const double field_boundary_x[NR_BOUNDARY_POINTS] =
  { -FIELD_LENGTH_H, -FIELD_LENGTH_H, -FIELD_LENGTH_H - GOAL_DEPTH, 
    -FIELD_LENGTH_H - GOAL_DEPTH, -FIELD_LENGTH_H, -FIELD_LENGTH_H, 
    FIELD_LENGTH_H, FIELD_LENGTH_H, FIELD_LENGTH_H + GOAL_DEPTH, 
    FIELD_LENGTH_H  + GOAL_DEPTH, FIELD_LENGTH_H, FIELD_LENGTH_H 
  };

const double field_boundary_y[NR_BOUNDARY_POINTS] =
  { -FIELD_WIDTH_H, -GOAL_WIDTH_H, -GOAL_WIDTH_H, 
    GOAL_WIDTH_H, GOAL_WIDTH_H, FIELD_WIDTH_H,
    FIELD_WIDTH_H, GOAL_WIDTH_H, GOAL_WIDTH_H, 
    -GOAL_WIDTH_H, -GOAL_WIDTH_H, -FIELD_WIDTH_H
  };

static const double timePerFrame = FRAME_PERIOD; // 30 fps

static const double wheelBase = 110.0; // mm
static const double robotHalfSize = (170.0 / 2.0); // mm
static const double maxRobotSpeed = 1800.0; // mm/s
static const double maxBallSpeed = 1800.0;  // mm/s
static const double maxRobotAccel = 3000.0; // mm/s^2

static const double ballCarpetFrictionCoef = 0.03;
static const double wallPerpBounceDecay = .4;
static const double robotPerpBounceDecay = .37;

static const double kickerSpeed = 50.0; // not tuned


static const double minX = -FIELD_LENGTH_H;
static const double maxX = FIELD_LENGTH_H;
static const double minY = -FIELD_WIDTH_H;
static const double maxY = FIELD_WIDTH_H;


// config parameters
// ball parameters
CR_DECLARE(BALL_CARPET_FRICTION_COEF);
CR_DECLARE(WALL_PERP_BOUNCE_DECAY);

// diffbot parameters
CR_DECLARE(DIFFBOT_POLY_X);
CR_DECLARE(DIFFBOT_POLY_Y);
CR_DECLARE(DIFFBOT_NRPOINTS);
CR_DECLARE(DIFFBOT_WHEELBASE);
CR_DECLARE(DIFFBOT_MAX_SPEED);
CR_DECLARE(DIFFBOT_MAX_ANG_VEL);
CR_DECLARE(DIFFBOT_MAX_ACC);
CR_DECLARE(DIFFBOT_MAX_ANG_ACC);
CR_DECLARE(DIFFBOT_PERP_BOUNCE_DECAY);
CR_DECLARE(DIFFBOT_KICKER_SPEED);
CR_DECLARE(DIFFBOT_DRIB_CATCH_SPEED);
CR_DECLARE(DIFFBOT_DRIB_BOUNCE_DECAY);

// omnibot parameters
CR_DECLARE(OMNIBOT_POLY_X);
CR_DECLARE(OMNIBOT_POLY_Y);
CR_DECLARE(OMNIBOT_NRPOINTS);
CR_DECLARE(OMNIBOT_WHEELBASE);
CR_DECLARE(OMNIBOT_MAX_SPEED);
CR_DECLARE(OMNIBOT_MAX_ANG_SPEED);
CR_DECLARE(OMNIBOT_MAX_ACC);
CR_DECLARE(OMNIBOT_MAX_ANG_ACC);
CR_DECLARE(OMNIBOT_PERP_BOUNCE_DECAY);
CR_DECLARE(OMNIBOT_KICKER_SPEED);
CR_DECLARE(OMNIBOT_DRIB_CATCH_SPEED);
CR_DECLARE(OMNIBOT_DRIB_BOUNCE_DECAY);

/**************************** GLOBALS ****************************************/


/***************************** ROBOTSIMULATOR CLASS ****************************/

/*
 * rand_posvector -
 *
 * function to calculate the random position vector on the field with a normal
 * distribution
 */
vector2d Simulator::rand_posvector(void)
{
  return (vector2d((rand() * (maxX - minX)) / RAND_MAX + minX,
		   (rand() * (maxY - minY)) / RAND_MAX + minY));
}

double Simulator::rand_heading(void)
{
  return (((double) rand() * M_PI * 2.0) / ((double) RAND_MAX));
}

/*
 * SetPolygon -
 *
 * Creates a new polygon class for the specified robot type
 */
void Simulator::SetPolygon(SimRobot &r)
{
  printf("setting polygons\n");

  switch (r.rtype) {
  case ROBOT_TYPE_DIFF:
    //    r.polygon.SetPolygon(min(VARSIZE(DIFFBOT_POLY_X), VARSIZE(DIFFBOT_POLY_Y)),
    //			 VDVAR(DIFFBOT_POLY_X), VDVAR(DIFFBOT_POLY_Y));

    r.polygon.SetPolygon(NR_DIFF_POINTS, diffpoints_x, diffpoints_y);
    break;
  case ROBOT_TYPE_OMNI:
    //    r.polygon.SetPolygon(min(VARSIZE(OMNIBOT_POLY_X), VARSIZE(OMNIBOT_POLY_Y)), 
    //   VDVAR(OMNIBOT_POLY_X), VDVAR(OMNIBOT_POLY_Y));
    r.polygon.SetPolygon(NR_OMNI_POINTS, omnipoints_x, omnipoints_x);
    break;
  }
}


void Simulator::AccelerationLimit(SimRobot &sr, double maxdv)
{
#if 0
  double a = sr.vel.v.angle() - sr.pos.dir;
  double vmag = sr.vel.v.length();
  
  vector2d oldv(vmag * cos(a), vmag * sin(a));
  vector2d dv = sr.vel.v - oldv;
  double dvmag = dv.length();
  
  if (dvmag > maxdv)
    sr.vel.v = oldv + dv * (maxdv / dvmag);
#else
  vector2d curv = sr.vel.v.rotate(-sr.pos.dir);
  vector2d dv = sr.vcmd.v - curv;

  if (dv.length() > maxdv)
    sr.vcmd.v = curv + dv.norm() * maxdv;
#endif
}


void Simulator::SpeedLimit(SimRobot &sr, double maxv)
{
  // Check maximum speed
  double vmag = sr.vel.v.length();
  if (vmag > maxv) 
    sr.vel.v *= maxv / vmag;
}


bool Simulator::CheckWallCollision(SimRobot &r, double minx, double maxx, 
				   double miny, double maxy)
{
  bool out_x_min,out_x_max,out_y_min,out_y_max;
  bool collision;
  
  out_x_min = (r.pos.p.x < minx);
  out_x_max = (r.pos.p.x > maxx);
  out_y_min = (r.pos.p.y < miny);
  out_y_max = (r.pos.p.y > maxy);
  
  
  if (out_x_min) {
    r.pos.p.x = minx;
    if(fabs(r.vel.v.x) > fabs(r.vel.v.y)) 
      r.pos.p.y = r.oldpos.p.y;
  } else if (out_x_max) {
    r.pos.p.x = maxx;
    if(fabs(r.vel.v.x) > fabs(r.vel.v.y)) 
      r.pos.p.y = r.oldpos.p.y;
  }
  
  if (out_y_min) {
    r.pos.p.y = miny;
    if (!(out_x_min || out_x_max)) {
      if (fabs(r.vel.v.y) > fabs(r.vel.v.x)) 
	r.pos.p.x = r.oldpos.p.x;
    }
  } else if (out_y_max) {
    r.pos.p.y = maxy;
    if (!(out_x_min || out_x_max)) {
      if(fabs(r.vel.v.y) > fabs(r.vel.v.x)) 
	r.pos.p.x = r.oldpos.p.x;
    }
  }
  
  collision = (out_x_min || out_x_max || out_y_min || out_y_max);
  if (collision) {
    r.vel.v.set(0.0, 0.0);
    r.pos.dir = r.oldpos.dir;
  }
  
  return (collision);
}


void Simulator::UpdateRobot(SimRobot &r, double dt)
{    
  /* do an acceleration check */
  AccelerationLimit(r, maxRobotAccel * dt);

  /* Speed Limit */
  SpeedLimit(r, maxRobotSpeed);

  /* store off old velocities and positions */
  r.oldpos = r.pos;

  /* transform to world coords */
  r.vel.v = r.vcmd.v;
  r.vel.v = r.vel.v.rotate(r.pos.dir);
  
  r.pos.p += r.vel.v * dt;
  r.pos.dir = anglemod(r.pos.dir + r.vcmd.va * dt);
  
  /* look for wall collisions */	
  r.collision = CheckWallCollision(r, minX, maxX, minY, maxY);
}


bool Simulator::RobotBallCollisionPolygon(SimRobot &r, SimBall &b, double dt)
{
  double dist = (r.pos.p - b.pos).length();
  LineSeg result;
  int line_id = -1;
  LineSeg test_seg(b.oldpos, b.pos);  
  
  /* check if we need to do anything at all */
  if (dist > robotHalfSize + BALL_RADIUS * 2.0)
    return (false);
  
  /* translate/rotate the edge polygon */
  r.polygon.RotateTranslate(r.pos.dir, r.pos.p);
  
  /* test for collisisions between the old position and the new */
  if (!r.polygon.GetIntersectingLine(test_seg, result, &line_id)) {
    
    /* if the b is inside the robot then we need to check if we hit it */
    if (!r.polygon.IsInside(b.pos))
      return (false);
    
    double dist = (r.pos.p - r.oldpos.p).length();
    double angle = (r.pos.p - r.oldpos.p).angle();
    
    /* did we instead drive through the b ? */        
    if (!r.polygon.GetIntersectingLineSweep(b.pos, dist, angle, result, &line_id)) {
      double angle = r.pos.dir;
      angle += M_PI_2;
      
      vector2d p1 = vector2d(cos(angle), sin(angle)) * 200.0;
      result = LineSeg(r.pos.p + p1, r.pos.p - p1);
    }
  } 
  
  // work out if we are kicking the ball
  // is it on our kicker?
  vector2d kvel(0.0, 0.0);
  if (line_id == 0) {
    if (r.vcmd.kick) {
      if (r.rtype == ROBOT_TYPE_DIFF)
	kvel.x = DVAR(DIFFBOT_KICKER_SPEED);
      else
	kvel.x = DVAR(OMNIBOT_KICKER_SPEED);
      kvel.rotate(r.pos.dir);
      printf("Kick!!!!...\n");
    } else if (r.vcmd.drib) {
      vector2d brel = ball.vel - r.vel.v;
      vector2d n(1.0,0);
      n = n.rotate(r.pos.dir);
      vector2d p = n.perp();
      
      vector2d bdrib(dot(brel, n), dot(brel, p));
      double dlimit;
      dlimit = (r.rtype ? DVAR(DIFFBOT_DRIB_CATCH_SPEED) : 
		DVAR(OMNIBOT_DRIB_CATCH_SPEED));
      printf("drib %f %f\n", bdrib.x, bdrib.y);

      if (bdrib.x < 0) {
	ball.vel = bdrib;
	if (fabs(bdrib.x) < dlimit) {
	  ball.vel.x = 0;
	} else {
	  ball.vel.x *= (r.rtype ? DVAR(DIFFBOT_DRIB_BOUNCE_DECAY) :
			 DVAR(OMNIBOT_DRIB_BOUNCE_DECAY));
	}
	ball.vel = ball.vel.rotate(r.pos.dir + M_PI_2);
	ball.vel += r.vel.v;
      }
    }
  }	

  /* calculate a line segment representing relative vector from origin */
  LineSeg seg(vector2d(0.0,0.0), b.vel - r.vel.v - kvel);
  seg.Reflect(result);
  b.vel = seg.LineVector() * robotPerpBounceDecay + r.vel.v;
  
  // need this to avoid oscilation around the edge of the robot
  b.pos = b.oldpos + b.vel * dt;

  // this bit of code was broken ???
  /* last final catch all check to prevent screw ups */
  //  if (r.polygon.IsInside(b.pos)) {
  //    fprintf(stderr, "Moving ball outside...\n");
  //    r.polygon.MoveOutside(b.pos);
  //  }
  
  return (true);
}


bool Simulator::RobotBallCollisionRadius(SimRobot &r, SimBall &b, double dt)
{
  double dist = (r.pos.p - b.pos).length();
  //  LineSeg ball_seg(b.oldpos, b.pos);  
  
  
  /* check if we need to do anything at all */
  if (dist > robotHalfSize + BALL_RADIUS)
    return (false);
	
  //  printf("Collision Radius: \n");
  
  /* find the closest point on teh line segment to our robot 
   * center and check if it is close enough. If so we hit the ball
   */
  //  vector2d closest;
  //	dist = ball_seg.Distance(r.pos, closest);
  //	if (dist > robotHalfSize + BALL_RADIUS)
  //		return (false);
  
  /* TO DO: need to work this part out still */
  
  /* last final catch all check to prevent screw ups */
  vector2d r2bvec = r.pos.p - b.pos;
  if (r2bvec.length() < robotHalfSize + BALL_RADIUS) {
    b.pos -= r2bvec.norm() * (robotHalfSize + BALL_RADIUS);
    b.vel.set(0.0, 0.0);
  }
  
  return (true);
}


/*
 * Initialize -
 *
 * This function initialises the simulator variables and links for
 * the main program
 *
 * RETURN VALUE: TRUe on success, FALSE on failure
 */
bool Simulator::Initialize(bool _usegoals = false, bool _usenoise = false)
{
  usegoals = _usegoals;
  usenoise = _usenoise;

  /* run the main simulator thread */
#ifdef DEBUG
  printf("Creating thread\n");
#endif

#ifdef SIM_THREADED
  return (!pthread_create(&run_thread, NULL, 
			  (pthread_start) Simulator::Run, (void *) this));
#else
  return (true);
#endif
}

/*
 * Run -
 *
 * This is the main thread for running the simulator. It enters a for ever
 * loop and updates the simulator by calling the Tick() function whe  the
 * time is up. Otherwise it does nothing.
 */
void Simulator::Run(Simulator *sim)
{

#ifdef SIM_THREADED

  /*
   * Multi-threaded version of the code. The thread eneters a for ever loop 
   * and calls the Tick() funciton to do the work as appropriate
   */
  
  struct timeval current, last;
  double secs, total;
  long frames = 0;
  
#ifdef DEBUG
  printf("Running the simulator thread\n");
#endif

  gettimeofday(&current, NULL);
  last = current;

  /* main loop - if time is up call the Tick() to simulate otherwise do nothing */
  while (1) {
    gettimeofday(&current, NULL);
    secs = ((current.tv_sec + current.tv_usec / 1000000.0) -
	    (last.tv_sec + last.tv_usec / 1000000.0));

    if (secs >= timePerFrame) {
      sim->Tick();
      last = current;
      
      frames++;
      total += secs;
      if (total > 5.0) {
	fprintf(stderr, "** SIM: %f FPS **\n", frames / 5.0);
	frames = 0; total = 0.0;
      }
    } else {
      // try to sleep...
      // but I can't sleep...
      // the light, it hurts my eyes...
      // must keep programming...
      // I am a zombie...
      // heeeeeeeeelllllppppp...

      // Don't sleep if lees than 20ms since the scheduler can't schedule us
      //   again in time.
      //if((timePerFrame-secs)*1e6-20e3 > 0.0)
      //  usleep((unsigned long)((timePerFrame-secs)*1e6-20e3));
    }
  }

#else

  /*
   * This code is non-threaded and must be called repeatedly. It will check the
   * clock and wait if appropriate before executing the tick function and returning
   */
  struct timeval current;
  static double secs_last = 0.0, total = 0.0;
  double secs;
  static long frames = 0;
  

  // wait for the time to expire
  do {
    gettimeofday(&current, NULL);
    secs = (current.tv_sec + current.tv_usec / 1000000.0);
    double dt = (timePerFrame - (secs - secs_last));
    if (dt * 1e6 - 20e3 > 0.0)
      usleep((unsigned long)((timePerFrame - (secs - secs_last)) * 1e6 - 20e3));
  } while (secs < secs_last + timePerFrame);

  // go do the grunt work
  sim->Tick();
  total += secs - secs_last;
  secs_last = secs;
  frames++;

  if (total > 5.0) {
    fprintf(stderr, "** SIM: %f FPS **\n", frames / 5.0);
    total = 0.0;
    frames = 0;
  }
#endif

}

void Simulator::wait_for_update(void)
{
#ifdef SIM_THREADED
  sem_wait(&update_flag);
#else
  Run(this);
#endif
}


/*
 * Simulator constructor -
 *
 * Constructor initialises the appropriate variables. It starts with the default
 * configuration.
 */
Simulator::Simulator() 
{
  /* initialise the random seed */
  struct timeval tv;
  gettimeofday(&tv, NULL);
  srand(tv.tv_usec);

  running_state = RUNSTATE_PLAY;

  // read in the config parameters
  CR_SETUP(simulator, BALL_CARPET_FRICTION_COEF, CR_DOUBLE);
  CR_SETUP(simulator, WALL_PERP_BOUNCE_DECAY, CR_DOUBLE);

  // diffbot parameters
  CR_SETUP(simulator, DIFFBOT_POLY_X, CR_DOUBLE);
  CR_SETUP(simulator, DIFFBOT_POLY_Y, CR_DOUBLE);
  CR_SETUP(simulator, DIFFBOT_NRPOINTS, CR_INT);
  CR_SETUP(simulator, DIFFBOT_WHEELBASE, CR_DOUBLE);
  CR_SETUP(simulator, DIFFBOT_MAX_SPEED, CR_DOUBLE);
  CR_SETUP(simulator, DIFFBOT_MAX_ANG_VEL, CR_DOUBLE);
  CR_SETUP(simulator, DIFFBOT_MAX_ACC, CR_DOUBLE);
  CR_SETUP(simulator, DIFFBOT_MAX_ANG_ACC, CR_DOUBLE);
  CR_SETUP(simulator, DIFFBOT_PERP_BOUNCE_DECAY, CR_DOUBLE);
  CR_SETUP(simulator, DIFFBOT_KICKER_SPEED, CR_DOUBLE);
  CR_SETUP(simulator, DIFFBOT_DRIB_CATCH_SPEED, CR_DOUBLE);
  CR_SETUP(simulator, DIFFBOT_DRIB_BOUNCE_DECAY, CR_DOUBLE);

  // omnibot parameters
  CR_SETUP(simulator, OMNIBOT_POLY_X, CR_DOUBLE);
  CR_SETUP(simulator, OMNIBOT_POLY_Y, CR_DOUBLE);
  CR_SETUP(simulator, OMNIBOT_NRPOINTS, CR_INT);
  CR_SETUP(simulator, OMNIBOT_WHEELBASE, CR_DOUBLE);
  CR_SETUP(simulator, OMNIBOT_MAX_SPEED, CR_DOUBLE);
  CR_SETUP(simulator, OMNIBOT_MAX_ANG_SPEED, CR_DOUBLE);
  CR_SETUP(simulator, OMNIBOT_MAX_ACC, CR_DOUBLE);
  CR_SETUP(simulator, OMNIBOT_MAX_ANG_ACC, CR_DOUBLE);
  CR_SETUP(simulator, OMNIBOT_PERP_BOUNCE_DECAY, CR_DOUBLE);
  CR_SETUP(simulator, OMNIBOT_KICKER_SPEED, CR_DOUBLE);
  CR_SETUP(simulator, OMNIBOT_DRIB_CATCH_SPEED, CR_DOUBLE);
  CR_SETUP(simulator, OMNIBOT_DRIB_BOUNCE_DECAY, CR_DOUBLE);

  // goal depth on screen appears to be short..so make a little short to make sure
  // we can get the ball back
  field_max.set(FIELD_LENGTH_H + GOAL_DEPTH / 2.0, FIELD_WIDTH_H + WALL_WIDTH + 50.0);
  field_boundary.SetPolygon(NR_BOUNDARY_POINTS, field_boundary_x, field_boundary_y);

  // default number of robots
  num_yellow = num_blue = 0;
  
  /* intialise the robot coordinates etc */
  for (int i = 0; i < MAX_TEAM_ROBOTS; i++) {
    blue_robots[i].rtype = ROBOT_TYPE_NONE;
    SetRobotRandom(false, i);
    SetPolygon(blue_robots[i]);
    
    yellow_robots[i].rtype = ROBOT_TYPE_NONE;
    SetRobotRandom(true, i);
    SetPolygon(yellow_robots[i]);
  }

  // Ball
  ball.pos.set(rand_posvector());
  ball.oldpos = ball.pos;
  ball.vel.set(0.0, 0.0);
  
  //    double mbs = maxBallSpeed / sqrt(2);
  //    ball.vel.x = (rand() * 2.0 * mbs) / RAND_MAX - mbs;
  //    ball.vel.y = (rand() * 2.0 * mbs) / RAND_MAX - mbs;

  // Init the semaphores 
  sem_init(&commands_mutex, 0, 1);
  sem_init(&runstate_mutex, 0, 1);
  sem_init(&update_flag, 0, 1);

  frameCount = 0;
  goalscored = false;
}

/*
 * ~Simulator -
 *
 * This deconstructor deallocates all the allocated main variables 
 */
Simulator::~Simulator()
{}

/*
 * AddRobot -
 *
 *( This function adds a robot to the specified position and orientation
 * it also sets the robot type
 *
 * RETURN VALUE: TRUE on success, false on failure
 */
bool Simulator::AddRobot(bool yellow, int rtype, double x = 0, double y = 0, double a = 0)
{
  if (yellow) {
    if (num_yellow >= MAX_TEAM_ROBOTS) 
      return false;
  
    yellow_robots[num_yellow].rtype = rtype;
    SetRobot(yellow, num_yellow, vector2d(x, y), a, vector2d(0, 0), 0);
    SetPolygon(yellow_robots[num_yellow]);
    
    /* all done so add numbe rof robots */
    num_yellow++;
  } else {
    if (num_blue >= MAX_TEAM_ROBOTS) 
      return false;
  
    blue_robots[num_blue].rtype = rtype;
    SetRobot(yellow, num_blue, vector2d(x, y), a, vector2d(0, 0), 0);
    SetPolygon(blue_robots[num_blue]);
    
    /* all done so add numbe rof robots */
    num_blue++;
  }

#ifdef DEBUG
  printf("Adding robot %i, ny %i, nb %i\n", yellow, num_yellow, num_blue);
#endif

  return (true);
}
 

/*
 * SetNumRobots -
 *
 * This function adds robots to the list (or deletes ones currently there) to
 * reach the desired number of bots
 */                  
bool Simulator::SetNumRobots(bool yellow, int n)
{
  if ((n < 0) || (n > MAX_TEAM_ROBOTS))
    return false;

  if (yellow) {
    while (num_yellow < n)  {
      AddRobot(yellow, ROBOT_TYPE_DIFF);
      SetRobotRandom(yellow, num_yellow - 1);
    }
    if (n < num_yellow) 
      num_yellow = n;
  } else {
    while (num_blue < n)  {
      AddRobot(yellow, ROBOT_TYPE_DIFF);
      SetRobotRandom(yellow, num_blue - 1);
    }
    if (n < num_blue) 
      num_blue = n;
  }
  return (true);
}


/*
 * SetRobotCommand -
 *
 * This function sets the robot commands for the next time step
 */
void Simulator::SetRobotCommand(bool yellow, int robot, double vx, double vy, double va,
				bool kick = false, bool drib = false)
{
  sem_wait(&commands_mutex);


#ifdef USE_VQUEUE    	
  if (yellow) {
    yellow_robots[robot].vqueue.SetCommand(vx, vy, va, kick, drib, frameCount);
  } else {
    blue_robots[robot].vqueue.SetCommand(vx, vy, va, kick, drib, frameCount);
  }
#else
  if (yellow) {
    yellow_robots[robot].vcmd.v.set(vx, vy);
    yellow_robots[robot].vcmd.va = va;
    yellow_robots[robot].vcmd.kick = kick;
    yellow_robots[robot].vcmd.drib = drib;

  } else {
    blue_robots[robot].vcmd.v.set(vx, vy);
    blue_robots[robot].vcmd.va = va;
    blue_robots[robot].vcmd.kick = kick;
    blue_robots[robot].vcmd.drib = drib;
  }
#endif
    
  sem_post(&commands_mutex);
}



/*
 * StopAll -
 *
 * This command stops all the robots dead
 */
void Simulator::StopAll(void)
{
  sem_wait(&commands_mutex);

  for(int i = 0; i < num_yellow; i++)
    yellow_robots[i].vqueue.SetCommand(0, 0, 0, 0, 0, frameCount);
  for(int i = 0; i < num_blue; i++)
    blue_robots[i].vqueue.SetCommand(0, 0, 0, 0, 0, frameCount);

  sem_post(&commands_mutex);
}


/*
 * Tick -
 *
 * This funciton is the guts of the simulator. It should be called on each simulation tick. 
 * (Hence the name :-). The function calculates the new position for the robots and ball
 * based on their kinematics and dynamics. It then accounts for the collisions between
 * all the objects and the objects and the field. 
 */
void Simulator::Tick(void)
{
  double secs = timePerFrame;

  /* wait on the runstate mutex to prevent conflicts */
  sem_wait(&runstate_mutex);

  // If paused then still signal a new frame.
  if (running_state == RUNSTATE_PAUSE) { 
	  
    // This forces the count 0.
    while (!sem_trywait(&update_flag))
      ;  
    sem_post(&update_flag);
    sem_post(&runstate_mutex);
    return;
  } else if (running_state == RUNSTATE_STEPFORWARD) 
    running_state = RUNSTATE_PAUSE;
		
  sem_post(&runstate_mutex);
  sem_wait(&commands_mutex);

  // turn of the goal score flag
  goalscored = 0;

  /* Update each team of Robots with their new velocities */
  for(int i = 0; i < num_yellow; i++) {
#ifdef USE_VQUEUE
    yellow_robots[i].vqueue.GetCommand(yellow_robots[i].vcmd, frameCount);
#endif
    UpdateRobot(yellow_robots[i], secs);
  }

  /* Update each team of Robots with their new velocities */
  for(int i = 0; i < num_blue; i++) {
#ifdef USE_VQUEUE
    blue_robots[i].vqueue.GetCommand(blue_robots[i].vcmd, frameCount);
#endif
    UpdateRobot(blue_robots[i], secs);
  }

  // Update Ball
  /* bound the ball off the wall */  
  ball.oldpos = ball.pos;
  vector2d v = ball.vel;  
  if ((ball.pos.x < minX) || (ball.pos.x > maxX))
    v.x *= M_SQRT1_2;
  if ((ball.pos.y < minY) || (ball.pos.y > maxY))
    v.y *= M_SQRT1_2;
  ball.pos += ball.vel * secs;
  
  /* decay the ball velocity with friction */
  double dv_mag = ballCarpetFrictionCoef * GRAVITY * secs;
  double vmag = ball.vel.length();
  vector2d dv = -ball.vel;
  if (vmag > dv_mag)
    dv *= dv_mag / vmag;
  ball.vel += dv;
  
  // Check for Walls
  //
  // (mhb) Updated for 45' walls.  
  //  See: http://dept.physics.upenn.edu/courses/gladney/
  //  mathphys/java/sect4/subsubsection4_1_4_3.html
  
  if (fabs(ball.pos.y) > FIELD_WIDTH_H) {
    ball.vel.y -= FSGN(ball.pos.y) * secs * M_SQRT1_2 * GRAVITY * 5.0/7.0;
  }
  if ((fabs(ball.pos.x) > FIELD_LENGTH_H) && (fabs(ball.pos.y) > GOAL_WIDTH_H)) {
    ball.vel.x -= FSGN(ball.pos.x) * secs * M_SQRT1_2 * GRAVITY * 5.0/7.0;
  }
  if ((fabs(ball.pos.y) > FIELD_WIDTH_H + WALL_WIDTH) || 
      ((fabs(ball.pos.x) > FIELD_LENGTH_H + WALL_WIDTH) &&
       (fabs(ball.pos.y) > GOAL_WIDTH_H))) {
    ball.vel.set(0,0);
  }

  //  if ((fabs(ball.pos.x) > field_max.x) && (fabs(ball.pos.y) < GOAL_WIDTH_H))
  //    ball.vel.x -= FSGN(ball.pos.x) * secs * M_SQRT1_2 * GRAVITY * 5.0/7.0;

  //  if ((fabs(ball.pos.y) > field_max.y + WALL_WIDTH) ||
  //      (fabs(ball.pos.x) > field_max.x + WALL_WIDTH)) {
  //    ball.vel.set(0.0, 0.0);
  //  }

  /*

  if (ball.pos.x < minX)
    ball.vel.x += secs * M_SQRT1_2 * GRAVITY * 5.0/7.0;
  if (ball.pos.x > maxX)
    ball.vel.x -= secs * M_SQRT1_2 * GRAVITY * 5.0/7.0;
  if (ball.pos.y < minY)
    ball.vel.y += secs * M_SQRT1_2 * GRAVITY * 5.0/7.0;
  if (ball.pos.y > maxY)
    ball.vel.y -= secs * M_SQRT1_2 * GRAVITY * 5.0/7.0;

  if ((ball.pos.x < minX - WALL_WIDTH) || (ball.pos.x > maxX + WALL_WIDTH) || 
      (ball.pos.y < minY - WALL_WIDTH) || (ball.pos.y > maxY + WALL_WIDTH)) {
    ball.vel.set(0.0, 0.0);
  }
  */
  
  /* now check for robot collisions with the ball 
   * original code by S Lenser
   */
  for (int i = 0; i < num_blue; i++) {
    if (blue_robots[i].rtype == ROBOT_TYPE_NONE)
      RobotBallCollisionRadius(blue_robots[i], ball, secs);
    else
      RobotBallCollisionPolygon(blue_robots[i], ball, secs);

    // bound the ball position
    blue_robots[i].pos.p.x = bound(blue_robots[i].pos.p.x, -field_max.x, field_max.x);
    blue_robots[i].pos.p.y = bound(blue_robots[i].pos.p.y, -field_max.y, field_max.y);
  }
  for (int i = 0; i < num_yellow; i++) {
    if (yellow_robots[i].rtype == ROBOT_TYPE_NONE)
      RobotBallCollisionRadius(yellow_robots[i], ball, secs);
    else
      RobotBallCollisionPolygon(yellow_robots[i], ball, secs);

    // bound the ball position
    yellow_robots[i].pos.p.x = bound(yellow_robots[i].pos.p.x, -field_max.x, field_max.x);
    yellow_robots[i].pos.p.y = bound(yellow_robots[i].pos.p.y, -field_max.y, field_max.y);
  }

  // bound the ball position
  ball.pos.x = bound(ball.pos.x, -field_max.x, field_max.x);
  ball.pos.y = bound(ball.pos.y, -field_max.y, field_max.y);

  // check if the ball is in the goal
  if (usegoals) {
    if (fabs(ball.pos.x > FIELD_LENGTH_H + BALL_RADIUS * 2.0) &&
	fabs(ball.pos.y) < GOAL_WIDTH_H) {

      goalscored = ((ball.pos.x > 0) ? 1 : -1);

      printf("Goal scored on goal %i!!!!\n", ball.pos.x > 0);
      ball.pos.set(0, 0);
      ball.vel.set(0, 0);
    }
  }

  // generate noise input on confidences
  if (usenoise) {

    // crappy for now
    ball.conf = (double) rand() / (double) RAND_MAX;

    for (int i = 0; i < num_yellow; i++) 
      yellow_robots[i].conf = (double) rand() / (double) RAND_MAX;
    for (int i = 0; i < num_blue; i++) 
      blue_robots[i].conf = (double) rand() / (double) RAND_MAX;
  }

  /* increment the ol frame counter */
  frameCount++;
  
  // take care of semaphores 
  sem_post(&commands_mutex);

  // This forces the count 0.
  while (!sem_trywait(&update_flag))
    ;
  sem_post(&update_flag);
}



/**************************** State manipulation variables *******************/

double Simulator::Time(void)
{ 
  return frameCount * timePerFrame; 
}

int Simulator::Frame(void)
{
  return frameCount; 
}

vector2d Simulator::GetRobotPosition(bool yellow, int robot)
{
  return (IS_YELLOW(yellow, yellow_robots[robot].pos.p,
		    blue_robots[robot].pos.p));
}

double Simulator::GetRobotDirection(bool yellow, int robot)
{
  return (IS_YELLOW(yellow, yellow_robots[robot].pos.dir,
		    blue_robots[robot].pos.dir));
}

vector2d Simulator::GetRobotVelocity(bool yellow, int robot)
{
  return (IS_YELLOW(yellow, yellow_robots[robot].vel.v,
		    blue_robots[robot].vel.v));
}

double Simulator::GetRobotAngularVelocity(bool yellow, int robot)
{
  return (IS_YELLOW(yellow, yellow_robots[robot].vel.va,
		    blue_robots[robot].vel.va));
}


int Simulator::GetRobotType(bool yellow, int robot)
{
  return (IS_YELLOW(yellow, yellow_robots[robot].rtype,
		    blue_robots[robot].rtype));
}

double Simulator::GetRobotConfidence(bool yellow, int robot)
{
  if (usenoise)
    return (IS_YELLOW(yellow, yellow_robots[robot].conf, 
		      blue_robots[robot].conf));
  else
    return (1.0);
}

vector2d Simulator::GetBallPosition(void)
{
  return (ball.pos);
}

vector2d Simulator::GetBallVelocity(void)
{
  return (ball.vel);
}

double Simulator::GetBallConfidence(void)
{
  if (usenoise) 
    return (ball.conf);
  else
    return (1.0);
}
  
void Simulator::SetBall(vector2d p, vector2d v)
{
  ball.pos = p;
  ball.oldpos = p;
  ball.vel = v;
}

void Simulator::SetBallPosition(vector2d p)
{
  ball.pos = p;
  ball.oldpos = p;
}

void Simulator::SetBallVelocity(vector2d v)
{
  ball.vel = v;
}

void Simulator::StopBall(void)
{
  ball.vel.set(0, 0);
}


void Simulator::SetRobot(bool yellow, int robot, vector2d p, double a, vector2d v, double va)
{
  if (yellow) {
    yellow_robots[robot].pos.p = p;
    yellow_robots[robot].oldpos.p = p;
    yellow_robots[robot].vel.v = v;
    yellow_robots[robot].vel.va = va;
    yellow_robots[robot].pos.dir = a;
    yellow_robots[robot].oldpos.dir = a;
  } else {
    blue_robots[robot].pos.p = p;
    blue_robots[robot].oldpos.p = p;
    blue_robots[robot].vel.v = v;
    blue_robots[robot].vel.va = va;
    blue_robots[robot].pos.dir = a;
    blue_robots[robot].oldpos.dir = a;
  }
}
	

void Simulator::SetRobotRandom(bool yellow, int robot)
{
  SetRobot(yellow, robot, rand_posvector(), rand_heading());
}

