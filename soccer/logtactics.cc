// log_tactics.h
// 
// testing tactics for making logs of motion
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
#include <unistd.h>
#include "logtactics.h"

// TStep
//
// Makes the robot perform a step response function
//

bool tstep_registered = Tactic::registerParser("test_step", TStep::parser);

void TStep::run(World &world, int me)
{
  vector2d p = world.teammate_position(me);
  double angle = world.teammate_direction(me);
  vector2d v = world.teammate_velocity(me);
  double vangle = world.teammate_angular_velocity(me);
  vraw vpos;
  world.teammate_raw(me, vpos);

  if (f != NULL) {
    fprintf(f, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", 
	    vpos.pos.x, vpos.pos.y, vpos.angle,
	    p.x, p.y, angle, v.x, v.y, vangle);
  }

  world.go(me, vx, vy, va);
}

Tactic *TStep::parser(const char *param_string)
{
  double vx, vy, va;
  char fname[256];
  FILE *f = NULL;

  fprintf(stderr, "param_string: %s\n", param_string);

  fname[0] = 0;
  sscanf(param_string, "%lf %lf %lf %s", &vx, &vy, &va, fname);
  if ((*fname != 0) && ((f = fopen(fname, "wt")) == NULL)) {
    fprintf(stderr, "ERROR: Cannot open file %s\n", fname);
    return NULL;
  } else {
    fprintf(stderr, "Running step test\n");
    fprintf(stderr, "\tparams: vel = (%f %f %f)\n", vx, vy, va);
    fprintf(stderr, "\tOutput: %s\n", fname);
    return new TStep(vx, vy, va, fname, f);
  }
}

// TRamp
//
// Makes the robot perform a ramp response function
//

bool tramp_registered = Tactic::registerParser("test_ramp", TRamp::parser);

void TRamp::run(World &world, int me)
{
  // log everything important 
  vector2d rp = world.teammate_position(me);
  double angle = world.teammate_direction(me);
  vector2d rv = world.teammate_velocity(me);
  double vangle = world.teammate_angular_velocity(me);
  vraw vpos;
  world.teammate_raw(me, vpos);

  if (f != NULL) {
    fprintf(f, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", 
	    vpos.pos.x, vpos.pos.y, vpos.angle,
	    rp.x, rp.y, angle, rv.x, rv.y, vangle);
  }

  // .z component substitutes for rotation
  world.go(me, v.x, v.y, v.z);
  v += a * FRAME_PERIOD;

  // tricky thing to ramp up to (or down to) final velocity and hold steady
  if ((v.x - vf.x) * sign(a.x) > 0)
    v.x = vf.x;
  if ((v.y - vf.y) * sign(a.y) > 0)
    v.y = vf.y;
  if ((v.z - vf.z) * sign(a.z) > 0) {
    fprintf(stderr, "wrap\n");
    v.z = vf.z;
  }
}

Tactic *TRamp::parser(const char *param_string)
{
  vector3d v, a;
  char fname[256];
  FILE *f = NULL;

  fname[0] = 0;
  sscanf(param_string, "%lf %lf %lf %lf %lf %lf %s", &v.x, &v.y, &v.z, &a.x, &a.y, &a.z, fname);
  if ((*fname != 0) && ((f = fopen(fname, "wt")) == NULL)) {
    fprintf(stderr, "ERROR: Cannot open file %s\n", fname);
    return NULL;
  } else {
    fprintf(stderr, "Running ramp test\n");
    fprintf(stderr, "\tparams: vel = (%f %f %f), acc = (%f, %f, %f)\n", 
	    v.x, v.y, v.z, a.x, a.y, a.z);
    fprintf(stderr, "\tOutput: %s\n", fname);
    return new TRamp(v, a, fname, f);
  }
}


// TKick
//
// Makes the robot perform a ramp response function
//

bool tkick_registered = Tactic::registerParser("test_kick", TKick::parser);

void TKick::run(World &world, int me)
{
  fprintf(stderr, "testing...\n");

  if (world.time > tnext) {
    on = !on;
    tnext = world.time + (on ? ontime : offtime);
    fprintf(stderr, "Running kick test: kicker on %i\n", on);
  }

  fprintf(stderr, "Running kick test: kicker on %i, dribbler %i\n", on, dribbler);

  world.go(me, 0, 0, 0, on, dribbler);
}

Tactic *TKick::parser(const char *param_string)
{
  printf("setting up test kick\n");

  if (strcmp(param_string, "dribbler") == 0)
    return new TKick(true);
  else
    return new TKick(false);
}



// TKick
//
// test he dribbler
//

bool tdribble_registered = Tactic::registerParser("test_dribbler", TDribble::parser);

void TDribble::run(World &world, int me)
{
  fprintf(stderr, "testing...\n");

  if (world.time > tnext) {
    on = !on;
    tnext = world.time + (on ? ontime : offtime);
    fprintf(stderr, "Running dribble test: dribbleer on %i\n", on);
  }

  fprintf(stderr, "Running dribble test: dribbleer on %i\n", on);

  world.go(me, 0, 0, 0, false, on);
}

Tactic *TDribble::parser(const char *param_string)
{
  return new TDribble();
}

