// modeller.cc
//
// This class holds the modeller information for histogramming the 
// opponent team.
//
// Created by:  Brett Browning (brettb@cs.cmu.edu)
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

#include "utils/histogram.h"
#include "utils/geometry.h"
#include "reality/net_vision.h"
#include "modeller.h"


// Initialize al lthe data  
bool Modeller::initialize(void)
{
  vector2d pmax(FIELD_LENGTH_H, FIELD_WIDTH_H);

  // this should come from aconfig file
  uint xbins = 2 * ((uint) (pmax.x / 100.0));
  uint ybins = 2 * ((uint) (pmax.y / 100.0));

  // initialize everything
  free_ball = 0;
  ball_speed.set(5000.0, 0.0, 50);
  ball_position.set(pmax, -pmax, xbins, ybins);

  // initialize the team-based stuff
  for (int t = 0; t < NUM_TEAMS; t++) {
    inattack[t] = 0;
    possession[t] = 0;

    speed[t].set(5000.0, 0.0, 50);
    acceleration[t].set(5000.0, 0.0, 50);
    goalie_speed[t].set(5000.0, 0.0, 50);
    goalie_acceleration[t].set(5000.0, 0.0, 50);

    robots_near_ball[t].set(10.0, 0.0, 10);

    position[t].set(pmax, -pmax, xbins, ybins);
    goalie_position[t].set(pmax, -pmax, xbins, ybins);
    shots_on_goal[t].set(pmax, -pmax, xbins, ybins);
  }
  return (true);
}


void Modeller::update(const net_vframe &frame)
{
  // update the non-team based histograms
  vector2d v((double) frame.ball.state.vx, (double) frame.ball.state.vy);
  ball_speed.add(v.length());
  vector2d ball(vector2d((double) frame.ball.state.x, (double) frame.ball.state.y));
  ball_position.add(ball);

  // this one is wrong
  if (frame.ball.state.x > 0)
    inattack[TEAM_BLUE]++;
  else
    inattack[TEAM_YELLOW]++;

  // work out who is closest to the ball
  int nr_near_ball[NUM_TEAMS];
  int closest = TEAM_BLUE;
  double minsqdist = HUGE_VAL;

  // range to the ball for considered near ball
  double rangesq = 300.0 * 300.0;

  for (int t = 0; t < NUM_TEAMS; t++) {
    int goalie;
    double goaliesqdist = HUGE_VAL;
    vector2d goalie_pos(0,0), goalie_vel(0,0);

    nr_near_ball[t] = 0;
    for (int i = 0; i < MAX_TEAM_ROBOTS; i++) {
      if (frame.config.teams[t].robots[i].id >= 0) {
	vector2d p(frame.robots[t][i].state.x, frame.robots[t][i].state.y);
	vector2d v(frame.robots[t][i].state.vx, frame.robots[t][i].state.vy);

	position[t].add(p);
	speed[t].add(v.length());

	// workout goalie info
	vector2d goal(-FIELD_LENGTH_H, 0);
	if (t == TEAM_YELLOW)
	  goal = -goal;
	double d = (goal - p).sqlength();

	if (d < goaliesqdist) {
	  goaliesqdist = d;
	  goalie = i;
	  goalie_pos = p;
	  goalie_vel = v;
	}

	// work out possession info
	d = (ball - p).sqlength();
	if (d < minsqdist) {
	  minsqdist = d;
	  closest = t;
	}
	if (d < rangesq)
	  nr_near_ball[t]++;
      }
    }
    robots_near_ball[t].add(nr_near_ball[t]);
    goalie_position[t].add(goalie_pos);
    goalie_speed[t].add(goalie_vel.length());
  }

  // now add to possession
  // need to make this smarter
  if (minsqdist < 100.0 * 100.0) {
    possession[closest]++;
  } else {
    free_ball++;
  }

  // need to work out shots on goal here
}
    


 
