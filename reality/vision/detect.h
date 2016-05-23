/*
 * TITLE:	Detect.h
 *
 * PURPOSE:	This is the main high-level vision class. 
 *
 * WRITTEN BY:	      James R Bruce
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

#ifndef __DETECT_H__
#define __DETECT_H__


#include "geometry.h"
#include "camera.h"
#include "vision.h"
#include "vtypes.h"
#include "markmap.h"

#include "reality/net_vision.h"

//#include "client/visionclient.h"

#define MAX_VISION_MARKERS 128
#define MAX_VISION_ROBOTS   16

class detect{
  struct vision_marker{
    double conf;
    vector2d loc;
    yuv color;
    char id; // 0,1
    region *reg;
    double dist,angle;
    vision_marker *next;
  };

  typedef MarkMap<vision_marker> markmap;

  class less_angle{
  public:
    bool operator()(const vision_marker &a,const vision_marker &b){
      return(a.angle < b.angle);
    }
  };

  struct vision_robot{
    double conf;
    vector2d loc;
    double angle;
    vision_marker marker[4];
    int id;
  }; // 228 bytes

  struct vision_calib_marker{
    double conf,d;
    vector2d pos;
    vector2d spos;
  };

  struct vision_ball{
    double conf;
    vector2d loc;
    yuv color;
  };

private:
  vision_robot *vrobot[NUM_TEAMS];
  int num_vrobots[NUM_TEAMS];
  // int cover[NUM_TEAMS][NUM_ID_MARKERS];
  int num_robots[NUM_TEAMS];
  rgb *reg_color;

  markmap team_markers[NUM_TEAMS];
  markmap orientation_markers;

  // crappy way for storing config info - BB
  net_vconfig vconfig;

  double latest_time;
  int frame;
public:
  detect() {init();}
  void init();
  void reset();

  void detect::print_markers(vision_marker *vmarker,int num);

  void find_ball(vlocations &nloc,vlocations &loc,LowVision &vision,camera &cam,double timestamp);
  void find_our_robots(int team, vlocations &nloc,vlocations &loc,
		       LowVision &vision,camera &cam,double timestamp);
  void find_opp_robots(int team, vlocations &nloc,vlocations &loc,
		       LowVision &vision,camera &cam,double timestamp);
  void find_calib_pattern(LowVision &vision,camera &cam,double timestamp);
  void find_common(LowVision &vision,camera &cam,double timestamp);

  void update(vlocations &loc,LowVision &vision,camera &cam,double timestamp);

  // a little crappy but it will do for now - BB
  void updateParams(net_vconfig  &vc) {
    vconfig = vc;
  }
};

#endif /* __DETECT_H__ */
