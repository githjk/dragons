// modeller.h
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

#ifndef __MODELLER_H__
#define __MODELLER_H__

#include <stdio.h>

#include "utils/histogram.h"
#include "utils/geometry.h"
#include "reality/net_vision.h"
#include "constants.h"

class Modeller {
public:
  Histogram1D speed[NUM_TEAMS], acceleration[NUM_TEAMS];
  Histogram1D goalie_speed[NUM_TEAMS], goalie_acceleration[NUM_TEAMS];
  Histogram2D position[NUM_TEAMS], goalie_position[NUM_TEAMS];

  // Histogram3D position3D;

  Histogram2D ball_position;
  Histogram1D ball_speed;
//  Histogram2D ball_position;
  
  // misc stats
  // number of times in attack or defense
  // defined
  uint inattack[NUM_TEAMS];
  uint possession[NUM_TEAMS], free_ball;
  
  Histogram1D robots_near_ball[NUM_TEAMS];

  // from where were shots on goal fired
  Histogram2D shots_on_goal[NUM_TEAMS];

public:
  
  Modeller(void) {}
  ~Modeller(void) {}

  bool initialize(void);
  void update(const net_vframe &frame);
  
  // operations to retrieve useful information
  
  
  
};


#endif
