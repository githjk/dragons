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

#ifndef __LOGTACTICS_H__
#define __LOGTACTICS_H__

#include "world.h"
#include "tactic.h"
#include <stdio.h>

class TStep : public Tactic {
public:
  double vx, vy, va;
  char fname[256];
  FILE *f;

  TStep(double _vx, double _vy, double _va, char *_fname, FILE *_f) {
    vx = _vx; vy = _vy; va = _va; f = _f;
    strcpy(fname, _fname);
    
  }

  virtual ~TStep(void) {
    fclose(f);
  }

  static Tactic *parser(const char *param_string);
  virtual Tactic *clone() const { return new TStep(*this); }

  virtual void run(World &world, int me);
};

class TRamp : public Tactic {
public:
  vector3d vf, v, a;
  char fname[256];
  FILE *f;

  TRamp(double _vx, double _vy, double _va, 
	    double _ax, double _ay, double _aa, char *_fname, FILE *_f) {
    vf.set(_vx, _vy, _va);
    v.set(0,0,0);
    a.set(_ax, _ay, _aa);
    strcpy(fname, _fname);
    f = _f;
  }
  TRamp(vector3d _v, vector3d _a, char *_fname, FILE *_f) {
    vf = _v;
    v.set(0,0,0);
    a = _a;
    strcpy(fname, _fname);
    f = _f;
  }

  virtual ~TRamp(void) {
    fclose(f);
  }

  static Tactic *parser(const char *param_string);
  virtual Tactic *clone() const { return new TRamp(*this); }

  virtual void run(World &world, int me);
};

class TKick : public Tactic {
public:
  double tnext;
  double ontime;
  double offtime;
  bool on, dribbler;

  TKick(bool _dribbler) {
    ontime = 0.1;
    offtime = 3.0;
    tnext = 0.0;
    on = false;
    dribbler = _dribbler;
  }

  static Tactic *parser(const char *param_string);
  virtual Tactic *clone() const { return new TKick(*this); }

  virtual void run(World &world, int me);
};

class TDribble : public Tactic {
public:
  double tnext;
  double ontime;
  double offtime;
  bool on;

  TDribble(void) {
    ontime = 0.5;
    offtime = 0.5;
    tnext = 0.0;
    on = false;
  }

  static Tactic *parser(const char *param_string);
  virtual Tactic *clone() const { return new TDribble(*this); }

  virtual void run(World &world, int me);
};

#endif
