/*
 * TITLE:  Detect.cc
 *
 * PURPOSE:  This is the main high-level vision class. 
 *
 * WRITTEN BY:  James R Bruce
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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>

#include "geometry.h"
#include "camera.h"
#include "vision.h"

#include "constants.h"
#include "reality/net_vision.h"

#include "detect.h"
#include <algorithm>


//==== Constants =====================================================//

const double epsilon = 1E-10;
#define NONE (-1)

#define rdtsc(low) \
   __asm__ __volatile__("rdtsc" : "=a" (low) : : "edx")

//==== Configuration Constants =======================================//

// ROGI 115 x 150 x 105 (lwh)

// Screen bounding box (should not normally be neccesary)
const bool use_screen_bbox = true;
const int sx_min =   0;
const int sx_max = 640;
const int sy_min =   0/2;
const int sy_max = 480/2;

// maximum accepted speed
#define MAX_BALL_VELOCITY  10000
#define MAX_ROBOT_VELOCITY  4000

// height at which we estimate the ball to be, since we assume it is a
// disk rather than a sphere.
#define BALL_HEIGHT 30

#define MAX_REGIONS (640*480/MIN_EXP_REGION_SIZE)


//==== Debugging Flags ===============================================//

const bool ball_dump         = false;
const bool dump              = false;
const bool print_coordinates = false;

const bool save_debug_images        = false;
const int  save_debug_image_rate    = 30*1;
const bool debug_image_colorize_all = false;
const bool dump_our_robot_conf = false;

bool dump_vision_stats        = false;
bool find_calib_patterns      = false;
const int  find_calib_patterns_rate = 30*1;

//==== Patterns ======================================================//
/*
  Bright Green = 1
  White        = 0
  Read as binary number, counter-clockwise from forward
  (MSB front left from behind)

  MSB-> 1    4 <-LSB
         2  3

 -Diff Robots------
   # : cover
   0 : 03 = 0011 = WWGG
   1 : 05 = 0101 = WGWG
   2 : 06 = 0110 = WGGW
   3 : 09 = 1001 = GWWG
   4 : 10 = 1010 = GWGW

 -Omni Robots------
   5 : 12 = 1100 = GGWW
   6 : 01 = 0001 = WWWG
   7 : 02 = 0010 = WWGW
   8 : 04 = 0100 = WGWW
   9 : 08 = 1000 = GWWW
*/

const char rid_to_cover[MAX_ROBOT_ID] = {
   3,  5,  6,  9, 10, // diffs
  12,  1,  2,  4,  8, // omnis
   0,  0,  0,         // unused
   0,  0,  0
};
const int first_omni_id = 5;

// mappings from cover number to robot id this one is generated as a
// reverse mapping of rid_to_cover from above.
char cover_to_rid[MAX_ROBOT_ID];


//====================================================================//
//    Utility Functions
//====================================================================//

double gaussian(double x)
{
  return(exp(-(x*x)/2));
}

inline double density(region *reg)
{
  int box_area = (reg->x2-reg->x1+1) * (reg->y2-reg->y1+1);

  return(((double)reg->area) / box_area);
}

template <class track_t>
inline double speed(track_t &t0,track_t &t1)
{
  return(Vector::distance(t0.p,t1.p) /
	 (t1.timestamp - t0.timestamp + epsilon));
}

inline double speed(vector2d &p0,double t0,vector2d &p1,double t1)
{
  return(Vector::distance(p0, p1) / (t1 - t0 + epsilon));
}

bool check_bbox(region *reg)
{
  if(use_screen_bbox){
    return((reg->cen_x >= sx_min) &&
	   (reg->cen_x <= sx_max) &&
	   (reg->cen_y >= sy_min) &&
	   (reg->cen_y <= sy_max));
  }else{
    return(true);
  }
}

inline double box_conf(double x,double y,double r,double out,
		       double w,double h)
{
  double dx,dy,f;

  dx = fabs(x)+r - w;
  dy = fabs(y)+r - h;

  if(dx<0 && dy<0) return(1.0);

  f = max(dx,dy) / out;
  return(max(1.0 - f,0.0));
}

double field_conf(double x,double y,double r,double out)
{
  return(max(box_conf(x,y,r,out,FIELD_LENGTH_H           ,FIELD_WIDTH_H),
	     box_conf(x,y,r,out,FIELD_LENGTH_H+GOAL_DEPTH,GOAL_WIDTH_H)));
}

template <class item>
void add_bucket(item *bucket,int max,item &it)
{
  int i;

  if(it.conf < bucket[max-1].conf) return;

  i = max-2;
  while(i>=0 && it.conf>bucket[i].conf){
    bucket[i+1] = bucket[i];
    i--;
  }
  bucket[i+1] = it;
}

template <class data>
void roll(data *arr,data *tmp,int num,int displacement)
{
  int i;

  // make sure it's positive
  displacement = ((displacement % num) + num) % num;

  // check if we have to do anything
  if(displacement % num == 0) return;

  // rotate using temp storage
  for(i=0; i<num; i++) tmp[i] = arr[i];
  for(i=0; i<num; i++) arr[(i + displacement) % num] = tmp[i];
}

struct match_weight{
  short track_id,vision_id;
  float weight;
};

bool operator <(const match_weight &a,
                const match_weight &b)
{
  return(a.weight < b.weight);
}

void match(int *track_to_vision,int num_track,
	   int *vision_to_track,int num_vision,
	   match_weight *wts,int num_wts)
{
  match_weight w;
  int i,undone;

  for(i=0; i<num_track ; i++) track_to_vision[i] = NONE;
  for(i=0; i<num_vision; i++) vision_to_track[i] = NONE;

  sort(wts,wts+num_wts);
  undone = min(num_track,num_vision);

  for(i=0; i<num_wts; i++){
    w = wts[i];
    if(track_to_vision[w.track_id ]==NONE &&
       vision_to_track[w.vision_id]==NONE){
      track_to_vision[w.track_id ] = w.vision_id;
      vision_to_track[w.vision_id] = w.track_id;

      //printf("match: t[%d]<-v[%d] s=%f\n",
      //       w.track_id,w.vision_id,w.weight);

      undone--;
      if(undone == 0) return; // no more matches possible
    }
  }

  if(false){
    printf("Match T->V:[ ");
    for(i=0; i<num_track; i++) printf("%d ",track_to_vision[i]);
    printf("]\n");

    printf("Match V->T:[ ");
    for(i=0; i<num_vision; i++) printf("%d ",vision_to_track[i]);
    printf("]\n");
  }
}


//====================================================================//
//    Detect Class Implementation
//====================================================================//

void detect::init()
{
  int team;
  int i;

  latest_time = 0;

  for(team=0; team<NUM_TEAMS; team++){
    vrobot[team] = new vision_robot[MAX_VISION_ROBOTS];
  }

  reg_color = new rgb[MAX_REGIONS];
  if(!reg_color) exit(1);

  // init the Vconfig data - BB
  for (team = 0; team < NUM_TEAMS; team++) {
    vconfig.teams[team].cover_type = VCOVER_NONE;
    for (i = 0; i < MAX_TEAM_ROBOTS; i++) {
      vconfig.teams[team].robots[i].id = -1;
      vconfig.teams[team].robots[i].type = ROBOT_TYPE_NONE;
    }
  }

  // generate the cover mappings from rid_to_cover array
  for (i = 0; i < MAX_ROBOT_ID; i++) 
    cover_to_rid[i] = -1;

  for (i = 0; i < MAX_ROBOT_ID; i++) {
    if (rid_to_cover[i] != 0) 
      cover_to_rid[rid_to_cover[i]] = i;
  }

  for(team=0; team<NUM_TEAMS; team++){
    team_markers[team].init(16,12,FIELD_LENGTH_H+GOAL_DEPTH,FIELD_WIDTH_H,
			    MAX_VISION_ROBOTS);
  }
  orientation_markers.init(16,12,FIELD_LENGTH_H+GOAL_DEPTH,FIELD_WIDTH_H,
			   MAX_VISION_MARKERS);

  frame = 0;

#ifdef FILEDUMP
  if ((dumpfile = fopen("track.txt", "wt")) == NULL) {
    fprintf(stderr, "ERROR: Cannot open dump file\n");
  }
#endif
}

void detect::reset()
{
  printf("Reset.\n");
}

void detect::find_ball(vlocations &nloc,vlocations &loc,
                       LowVision &vision,camera &cam,double timestamp)
{
  // vision_ball ball;
  region *reg,*mreg;
  vector2d p;
  double conf;
  double d,md;
  int team,i;
  rgb c,bc;
  yuv ac;

  if(false){
    reg = vision.getRegions(COLOR_ORANGE);

    while((reg != NULL) && (reg->area >= 8)){
      ac = vision.getAverageColor(reg);
      printf("b %3d %2d %2d  %3d %3d %3d\n",
	     reg->area,(reg->x2-reg->x1+1),(reg->y2-reg->y1+1),
	     ac.y,ac.u,ac.v);
      reg = reg->next;
    }
  }

  mreg = reg = vision.getRegions(COLOR_ORANGE);
  if(!reg){
    nloc.ball.conf = 0;
    nloc.ball.timestamp = timestamp;
    nloc.ball.cur.loc.set(0,0);
    nloc.ball.cur.angle = 0.0;
    return;
  }

  bc.red   = 255;
  bc.green = 128;
  bc.blue  =   0;

  while(reg!=NULL && reg->area>=8){
    if(check_bbox(reg) &&
       reg->x2-reg->x1+1 >=  4 &&
       reg->x2-reg->x1+1 <  22 &&
       reg->y2-reg->y1+1 >=  2 &&
       reg->y2-reg->y1+1 <  10){

      conf = gaussian((40 - reg->area) / 30.0);
      ac = vision.getAverageColor(reg);
      p = cam.screenToWorld(reg->cen_x,reg->cen_y,BALL_HEIGHT);

      conf *= field_conf(p.x,p.y,0,4*WALL_WIDTH);
      conf *= (ac.v-ac.u > 75);
      conf = conf*0.99 + 0.01;

      if(false) printf("conf=%f %d\n",conf,ac.v-ac.u);

      if(save_debug_images){
	c.red   = (int)(bc.red   * conf);
	c.green = (int)(bc.green * conf);
	c.blue  = (int)(bc.blue  * conf);
	i = vision.getRegionID(reg);
	if(i < MAX_REGIONS) reg_color[i] = c;
      }

      // is this better than our current detection?
      if(conf > nloc.ball.conf){
	// find distance to nearest robot
	md = 100;
	for(team=0; team<NUM_TEAMS; team++){
	  for(i=0; i<MAX_TEAM_ROBOTS; i++){
	    if(nloc.robot[team][i].conf > 0.0){
	      d = Vector::distance(nloc.robot[team][i].cur.loc,p);
	      if(d < md) md = d;
	    }
	  }
	}

	if(false) printf("md=%f\n",md);

	// if too close to a robot, ignore
	if(md > 70){
	  // otherwise accept it
	  nloc.ball.conf = conf;
	  nloc.ball.timestamp = timestamp;
	  nloc.ball.cur.loc = p;
	  nloc.ball.cur.angle = 0.0;
	  mreg = reg;
	}else{
	  // printf("too close!\n");
	}
      }
    }
    reg = reg->next;
  }

  if(false){
    d = Vector::distance(loc.ball.cur.loc,nloc.ball.cur.loc);
    if(mreg!=NULL && d>20){
      printf("(%f %f) [%d %d]\n",
	     nloc.ball.cur.loc.x,
	     nloc.ball.cur.loc.y,
	     mreg->x2-mreg->x1+1,
	     mreg->y2-mreg->y1+1);
    }
  }
}

void detect::print_markers(vision_marker *vmarker,int num)
{
  vision_marker m;
  int i;

  for(i=0; i<num; i++){
    m = vmarker[i];
    printf("  m%d conf=%f loc(%7.2f,%7.2f) color(%3d,%3d,%3d) id(%d) dist=%7.2f angle=%7.2f\n",
	   i,m.conf,V2COMP(m.loc),
	   m.color.y,m.color.u,m.color.v,
	   m.id,m.dist,m.angle);
    if(m.loc.x > 10000) exit(2);
  }
}

void detect::find_our_robots(int team, vlocations &nloc,vlocations &loc,
                         LowVision &vision,camera &cam,double timestamp)
{
  region *reg;
  vector2d p;
  double conf;
  int i,j,k,num,vid,num_markers;
  rgb c,bc;
  vision_marker vmarker,*marker;
  double dist;
  double max_dist;
  int m;
  int uv,uv_max,uv_min,uv_thresh;
  int id,vision_id;
  vision_marker tmp[4];
  char cover_to_vid[NUM_TEAMS][16];
  vision_robot vr;
  double height = DIFFBOT_HEIGHT;

  // position & orientation vars
  vector2d l,r,f,b,cen,x,y;
  less_angle comp_angle;

  // set up a mapping from cover IDs to robots
  // BB fix: data already stored in cover_to_rid if covers enabled
  // map from enabled robot radio_id to the cover
  memset(cover_to_vid,-1,sizeof(cover_to_vid));

  char rid = -1;

  // assign the cover to array id mapping
  // must do this per frame since config can change
  if(vconfig.teams[team].cover_type == VCOVER_NORMAL){
    for(i=0; i<MAX_TEAM_ROBOTS; i++){
      if((rid = vconfig.teams[team].robots[i].id) >= 0){
	cover_to_vid[team][rid_to_cover[rid]] = i;
      }
    }
  }

  if(false){
    static int frames = 0;
    reg = vision.getRegions(COLOR_WHITE);
    while(reg != NULL){
      if(reg->area < 64){
	printf("n %d %d %d\n",reg->area,reg->x2-reg->x1,reg->y2-reg->y1);
      }
      reg = reg->next;
    }
    frames++;
    if(frames >= 600) exit(0);
  }

  // clear out confidences
  for(i=0; i<MAX_TEAM_ROBOTS; i++){
    nloc.robot[team][i].conf = 0.0;
  }

  //==== Get new robot team marker detections ====//

  mzero(vrobot[team],MAX_VISION_ROBOTS);
  mzero(vr);

  reg = vision.getRegions((team==TEAM_BLUE)? COLOR_BLUE:COLOR_YELLOW);

  if(team == TEAM_BLUE){
    bc.red =   0; bc.green =   0; bc.blue = 255;
  }else{
    bc.red = 255; bc.green = 255; bc.blue =   0;
  }

  num = 0;
  team_markers[team].clear();
  mzero(vmarker);
  while((reg != NULL) && (reg->area >= 8)){
    if ((reg->x2-reg->x1+1 >=  4) &&
	(reg->x2-reg->x1+1 <  16) &&
	(reg->y2-reg->y1+1 >=  2) &&
	(reg->y2-reg->y1+1 <   8)){

      p = cam.screenToWorld(reg->cen_x,reg->cen_y,height);
      if(orientation_markers.count(p,70) >= 4){
	conf = gaussian((24 - reg->area) / 30.0);
	conf *= field_conf(p.x, p.y, 0, 2 * WALL_WIDTH);
	conf = conf * 0.99 + 0.01;

	if(save_debug_images){
	  c.red   = (int)(bc.red   * conf);
	  c.green = (int)(bc.green * conf);
	  c.blue  = (int)(bc.blue  * conf);
	  k = vision.getRegionID(reg);
	  if(k < MAX_REGIONS) reg_color[k] = c;
	}

	vr.conf = conf;
	vr.loc  = p;
	add_bucket(vrobot[team],MAX_VISION_ROBOTS,vr);
	if(num < MAX_VISION_ROBOTS) num++;

	vmarker.conf = conf;
	vmarker.loc = p;
	vmarker.reg = reg;
	team_markers[team].add(vmarker);
      }
    }

    reg = reg->next;
  }
  num_vrobots[team] = num;

  //==== Find robots given team patch and orientation/ID markers ====//

  for(i=0; i<num; i++){
    marker = vrobot[team][i].marker;
    num_markers = orientation_markers.find_conf(marker,4,vrobot[team][i].loc,70);

    if(num_markers == 4){
      for(j=0; j<num_markers; j++){
	marker[j].color = vision.getAverageColor(marker[j].reg);
	marker[j].angle = (marker[j].loc - vrobot[team][i].loc).angle();

	if(save_debug_images){
	  c = YuvToRgb(marker[j].color);
	  k = vision.getRegionID(marker[j].reg);
	  if(k < MAX_REGIONS) reg_color[k] = c;
	}
      }
      sort(&marker[0],&marker[4],comp_angle);
      // print_markers(marker,4);

      // find pair that are furthest apart
      // also find color extents in uv
      m = 0;
      max_dist = 0;

      uv_max = 0;
      uv_min = 256*2;

      for(j=0; j<4; j++){
	dist = Vector::distance(marker[j].loc,marker[(j+1)%4].loc);
	if(dist > max_dist){
	  m = j;
	  max_dist = dist;
	}

	// printf("%d %d\n",marker[j].color.u,marker[j].color.v);
	// printf("%3d ",marker[j].color.y);

	// uv = marker[j].color.y;
	uv = marker[j].color.u + marker[j].color.v;
	if(uv > uv_max) uv_max = uv;
	if(uv < uv_min) uv_min = uv;
      }
      // printf("\n");

      // roll furthest pair to end
      roll(marker,tmp,4,4-1-m);

      // NOTE: Now we have the markers sorted by angle relative to
      // forward on the robot, and can actually do the stuff we want
      // to do

      uv_thresh = (uv_max + uv_min) / 2;
      vision_id = 0;
      for(j=0; j<4; j++){
	// uv = marker[j].color.y;
	uv = marker[j].color.u + marker[j].color.v;
	id = uv < uv_thresh;
	marker[j].id = id;
	vision_id = (vision_id << 1) | id;
	// printf("%d < %d\n",uv,uv_thresh);
      }

      // NOTE: Semi-Hack (probably not the prettiest solution)
      // fix height of markers if an omni
      if(cover_to_rid[vision_id] >= first_omni_id){
	for(j=0; j<4; j++){
	  if(marker[j].conf>0.0 && marker[j].reg){
	    p = cam.screenToWorld(marker[j].reg->cen_x,marker[j].reg->cen_y,
				  OMNIBOT_HEIGHT);
	    // printf("(%f,%f) : <%f,%f>\n",V2COMP(marker[j].loc),
            //   p.x-marker[j].loc.x,p.y-marker[j].loc.y);
	    marker[j].loc = p;
	  }
	}
      }

      // get best estimate of center of robot
      cen = (marker[0].loc + marker[1].loc +
	     marker[2].loc + marker[3].loc) / 4;

      // calculate orientation
      f = marker[3].loc - marker[0].loc;
      b = marker[2].loc - marker[1].loc;
      l = marker[0].loc - marker[1].loc;
      r = marker[3].loc - marker[2].loc;

      y = f + b;
      x.set(-y.y,y.x);
      x += l + r;

      x.normalize();
      y.set(-x.y,x.x);

      conf = (0.75*vrobot[team][i].conf + 0.25) *
	(0.75*marker[0].conf + 0.25) * (0.75*marker[1].conf + 0.25) *
	(0.75*marker[2].conf + 0.25) * (0.75*marker[3].conf + 0.25);
      if(dump_our_robot_conf) printf("%f\n",conf);
      vrobot[team][i].conf  = conf;
      vrobot[team][i].loc   = cen;
      vrobot[team][i].angle = x.angle();

      if(false){
	printf("%d %f %f %f\n",
	       cover_to_rid[vision_id],conf,V2COMP(cen));
      }

      vid = cover_to_vid[team][vision_id];
      // printf("vision[%d] -> robot[%d]\n",vision_id,vid);
      // printf("cen(%f,%f)\n",V2COMP(cen));
      if(vid>=0 && (vrobot[team][i].conf > nloc.robot[team][vid].conf)){
	nloc.robot[team][vid].conf = vrobot[team][i].conf;
	nloc.robot[team][vid].timestamp = timestamp;
	nloc.robot[team][vid].cur.loc   = vrobot[team][i].loc;
	nloc.robot[team][vid].cur.angle = vrobot[team][i].angle;
      }

      if(false){
	if(vid == 0){
	  printf("%f %f\n",V2COMP(nloc.robot[team][vid].cur.loc));
	}
      }

      if(false){
	printf("%s[%d] conf=%6.4f id=%2d (%8.2f,%8.2f):%6.3f\n",
	       (team == TEAM_BLUE)? "dkblue" : "yellow",i,
	       vrobot[team][i].conf, vision_id,
	       vrobot[team][i].loc.x, vrobot[team][i].loc.y,
	       vrobot[team][i].angle);
      }

      if(false){
	if(vision_id == 10){
	  printf("%8.2f %8.2f %6.3f %d\n",
		 V2COMP(vrobot[team][i].loc),vrobot[team][i].angle,
		 vision.getField());
	}
      }
    }
  }
}

void detect::find_opp_robots(int team, vlocations &nloc,vlocations &loc,
			     LowVision &vision,camera &cam,double timestamp)
{
  region *reg;
  vector2d p;
  double conf,d;
  int i,j;

  int rnum; // number of robots (how many we're looking for)
  int vnum; // number of robots detected by vision
  rgb c,bc;

  // matching variables
  int match_track_to_vision[MAX_TEAM_ROBOTS];
  int match_vision_to_track[MAX_VISION_ROBOTS];
  match_weight wts[MAX_VISION_ROBOTS*MAX_TEAM_ROBOTS];
  int num_wts;


  rnum = num_robots[team];
  if(rnum <= 0) return;

  // clear out confidences
  for(i=0; i<MAX_TEAM_ROBOTS; i++){
    nloc.robot[team][i].conf = 0.0;
  }

  if(false){
    reg = vision.getRegions((team==TEAM_BLUE)? COLOR_BLUE:COLOR_YELLOW);

    while((reg != NULL) && (reg->area >= 8)){
      printf("m%d %d %d %d\n",team,
	     reg->area,(reg->x2-reg->x1+1),(reg->y2-reg->y1+1));

      reg = reg->next;
    }
  }

  //==== Get new robot team marker detections ====//

  mzero(vrobot[team],MAX_VISION_ROBOTS);

  reg = vision.getRegions((team==TEAM_BLUE)? COLOR_BLUE:COLOR_YELLOW);

  if(team == TEAM_BLUE){
    bc.red =   0; bc.green =   0; bc.blue = 255;
  } else {
    bc.red = 255; bc.green = 255; bc.blue =   0;
  }

  vnum = 0;
  // Magic number city -- what do these numbers mean?
  while((reg != NULL) && (reg->area >= 8)){
    if((reg->x2-reg->x1+1 >=  3) &&
       (reg->x2-reg->x1+1 <  18) &&
       (reg->y2-reg->y1+1 >=  2) &&
       (reg->y2-reg->y1+1 <   8)){

      // printf("%d %d %d\n",reg->area,reg->x2-reg->x1,reg->y2-reg->y1);

      // printf("%d\n",reg->area);
      conf = gaussian((20 - reg->area) / 20.0);
      p = cam.screenToWorld(reg->cen_x,reg->cen_y,OPPONENT_HEIGHT);
      conf *= field_conf(p.x, p.y, 60, WALL_WIDTH);
      if(conf < 0.05) conf = 0.0;
      // conf = conf * 0.99 + 0.01;

      if(save_debug_images){
	c.red   = (int)(bc.red   * conf);
	c.green = (int)(bc.green * conf);
	c.blue  = (int)(bc.blue  * conf);
	i = vision.getRegionID(reg);
	if(i < MAX_REGIONS) reg_color[i] = c;
      }

      if(vnum < MAX_VISION_ROBOTS){
	vrobot[team][vnum].conf  = conf;
	vrobot[team][vnum].loc   = p;
	vnum++;
      }
    }

    reg = reg->next;
  }
  num_vrobots[team] = vnum;

  // Remove duplicate detections
  for(i=0; i<vnum; i++){
    for(j=i+1; j<vnum; j++){
      d = Vector::distance(vrobot[team][i].loc,vrobot[team][j].loc);
      if(d < 50){
	vrobot[team][j].conf = 0.0;
	// printf("vdup\n");
      }
    }
  }

  // Remove duplicate tracking entries
  for(i=0; i<rnum; i++){
    for(j=i+1; j<rnum; j++){
      d = Vector::distance(loc.robot[team][i].cur.loc,
			   loc.robot[team][j].cur.loc);
      if(d < 50){
	loc.robot[team][j].conf = 0.0;
	// loc.robot[team][j].cur.loc.x += drand48()*100-50;
	// printf("tdup\n");
      }
    }
  }

  // Set up matching weights
  num_wts = 0;
  for(i=0; i<rnum; i++){
    for(j=0; j<vnum; j++){
      if(vrobot[team][j].conf > 0.0){
	wts[num_wts].weight =
	  speed(loc.robot[team][i].cur.loc,loc.robot[team][i].timestamp,
		vrobot[team][j].loc,timestamp);
	wts[num_wts].track_id = i;
	wts[num_wts].vision_id = j;
	num_wts++;
      }
    }
  }

  // Perform the greedy matching
  match(match_track_to_vision,rnum,
	match_vision_to_track,vnum,
	wts,num_wts);

  // Grab out the results
  for(i=0; i<rnum; i++){
    j = match_track_to_vision[i];
    if(j != NONE){
      nloc.robot[team][i].conf      = vrobot[team][j].conf;
      nloc.robot[team][i].timestamp = timestamp;
      nloc.robot[team][i].cur.loc   = vrobot[team][j].loc;
      nloc.robot[team][i].cur.angle = vrobot[team][j].angle;
    }
    // printf("t[%d]<-v[%d]\n",i,j);
  }
}

void detect::find_calib_pattern(LowVision &vision,camera &cam,double timestamp)
// Note: expects no robots on the field (with white patches)
{
  const int pattern_row  = 3;
  const int pattern_size = pattern_row * pattern_row;

  // const double pattern_rad = 110; // pattern edge half length
  // const double marker_dist =  70; // distance to four neighbors
  const double pattern_rad = 100; // pattern edge half length
  const double marker_dist =  60; // distance to four neighbors

  const double side_x   = FIELD_LENGTH_H - pattern_rad;
  const double corner_x = side_x - CORNER_BLOCK_WIDTH;

  const int num_calib_pos = 12;
  const vector2d calib_pos[num_calib_pos] = {
    vector2d(-corner_x, FIELD_WIDTH_H       -pattern_rad),
    vector2d(        0, FIELD_WIDTH_H       -pattern_rad),
    vector2d( corner_x, FIELD_WIDTH_H       -pattern_rad),

    vector2d(  -side_x, GOAL_WIDTH_H        +pattern_rad),
    vector2d(        0, CENTER_CIRCLE_RADIUS+pattern_rad),
    vector2d(   side_x, GOAL_WIDTH_H        +pattern_rad),

    vector2d(  -side_x,-GOAL_WIDTH_H        -pattern_rad),
    vector2d(        0,-CENTER_CIRCLE_RADIUS-pattern_rad),
    vector2d(   side_x,-GOAL_WIDTH_H        -pattern_rad),

    vector2d(-corner_x,-FIELD_WIDTH_H       +pattern_rad),
    vector2d(        0,-FIELD_WIDTH_H       +pattern_rad),
    vector2d( corner_x,-FIELD_WIDTH_H       +pattern_rad)
  };

  const char *calib_pos_name[num_calib_pos] = {
    "Yellow top corner",
    "Center top",
    "Blue top corner",

    "Yellow top goal mouth",
    "Center circle top",
    "Blue top goal mouth",

    "Yellow bottom goal mouth",
    "Center circle bottom",
    "Blue bottom goal mouth",

    "Yellow bottom corner",
    "Center bottom",
    "Blue bottom corner",
  };

  int i,j,n;
  int x,y;

  vision_marker vmarker[pattern_size];
  vision_marker tmarker[pattern_row][pattern_row];
  vision_marker m;
  vector2d avg,p,cp;
  double d,md;
  int vnum;
  const char *name = "";
  bool oops,dups;
  int num_found = 0;

  printf("==== Find Patterns =========================================\n\n");

  // Find the nine (dots)
  for(i=0; i<orientation_markers.num_markers; i++){
    p = orientation_markers[i].loc;
    n = orientation_markers.count(p,pattern_rad);

    if(n >= pattern_size){
      mzero((vision_marker*)vmarker,pattern_size);
      vnum = orientation_markers.find(vmarker,pattern_size,
				      p,pattern_rad);
      if(vnum == pattern_size){
	// find centroid
	avg.set(0,0);
	for(j=0; j<vnum; j++) avg += vmarker[j].loc;
	avg = avg / vnum;
	printf("# center=(%8.2f,%8.2f) avg=(%8.2f,%8.2f)\n",
	       V2COMP(p),V2COMP(avg));

	// put the markers into the pattern array
	mzero((vision_marker*)tmarker,pattern_size);
	oops = dups = false;
	for(j=0; j<vnum; j++){
	  p = vmarker[j].loc - avg;
	  x = (int)(p.x/45);
	  y = (int)(p.y/45);
	  // printf("(%d,%d)\n",x,y);

	  if(abs(x)>1 || abs(y)>1){
	    // (o)ut (o)f (p)attern (s)afegaurd
	    // i.e. we're probably picking up dots not in our pattern
	    oops = true;
	  }else if(tmarker[x+1][y+1].conf > 0.0){
	    // this marker is a duplicate, so something must have failed
	    dups = true;
	  }else{
	    tmarker[x+1][y+1] = vmarker[j];
	  }
	}

	if(!oops && !dups){
	  // find out which calibration position this is
	  md = 10000;
	  cp.set(0,0);
	  for(j=0; j<num_calib_pos; j++){
	    d = Vector::distance(calib_pos[j],avg);
	    if(d < md){
	      cp = calib_pos[j];
	      name = calib_pos_name[j];
	      md = d;
	    }
	  }

	  // print it out if not too far
	  if(md < 1000){
	    p = avg - cp;
	    printf("# Pattern: %s (%8.2f,%8.2f)+<%8.3f,%8.3f>\n",
		   name,V2COMP(cp),V2COMP(p));
	    for(y=1; y>=-1; y--){
	      for(x=-1; x<=1; x++){
		m = tmarker[x+1][y+1];
		printf("(%8.2f,%8.2f,%8.2f) (%8.3f,%8.3f)\n",
		       cp.x + marker_dist*x,cp.y + marker_dist*y,0.0,
		       (double)m.reg->cen_x,(double)2*m.reg->cen_y);
	      }
	    }
	    printf("\n");
	    num_found++;
	  }
	}
      }
    }
  }

  printf("# found %d pattern%s\n",num_found,(num_found == 1)?"":"s");
}

void detect::find_common(LowVision &vision,camera &cam,double timestamp)
{
  vision_marker vmarker;
  region *reg;
  int cid;
  double height = find_calib_patterns? 0 : DIFFBOT_HEIGHT;

  rgb c;
  int i;

  //==== Get orientation/ID marker detections ====//

  mzero(vmarker);
  orientation_markers.clear();

  for(cid=0; cid<2; cid++){
    reg = vision.getRegions(cid? COLOR_WHITE : COLOR_BGREEN);

    while((reg != NULL) && (reg->area > 4)){
      if((reg->x2-reg->x1+1 >=  4) &&
	 (reg->x2-reg->x1+1 <  16) &&
	 (reg->y2-reg->y1+1 >=  2) &&
	 (reg->y2-reg->y1+1 <   8)){

	vmarker.conf  = gaussian((24 - reg->area) / 10.0);
	vmarker.loc   = cam.screenToWorld(reg->cen_x,reg->cen_y,height);
	vmarker.color = vision.getAverageColor(reg);
	vmarker.reg   = reg;

	if(save_debug_images){
	  c.red   = (int)(  0 * vmarker.conf);
	  c.green = (int)(255 * vmarker.conf);
	  c.blue  = (int)(255 * vmarker.conf);

	  i = vision.getRegionID(reg);
	  if(i < MAX_REGIONS) reg_color[i] = c;
	}

	orientation_markers.add(vmarker);
      }

      reg = reg->next;
    }
  }
  // orientation_markers.dump();
}

double sum = 0.0;

void detect::update(vlocations &loc,LowVision &vision,camera &cam,double timestamp)
{
  vlocations nloc;
  vector2d delta;
  double s;
  int team,i,n;
  // unsigned long start, end;

  cam.setField(2,vision.getField());

  // set up clean detection structure
  mzero(nloc);
  nloc.timestamp = timestamp;

  /*
  // find stuff
  for (team = 0; team < NUM_TEAMS; team++) {
    if (vconfig.teams[team].cover_type == VCOVER_NORMAL)
      find_our_robots(team, nloc, loc, vision, cam, timestamp);
    else
      find_their_robots(team, nloc, loc, vision, cam, timestamp);
  }
  */

  for(team=0; team<NUM_TEAMS; team++){
    n = 0;
    for(i=0; i<MAX_TEAM_ROBOTS; i++){
      n += (vconfig.teams[team].robots[i].id != NONE);
    }
    num_robots[team] = n;
  }

  // printf("num: %d %d\n",num_robots[0],num_robots[1]);
  if(false){
    for(team=0; team<NUM_TEAMS; team++){
      printf("vcfg: t[%d] cover=%d\n",team,
	     vconfig.teams[team].cover_type);
      for(i=0; i<MAX_TEAM_ROBOTS; i++){
	printf("vcfg: t[%d] id=%d type=%d\n",team,
	       vconfig.teams[team].robots[i].id,
	       vconfig.teams[team].robots[i].type);
      }
    }
  }

  // unused things are green
  if(save_debug_images){
    region *reg;
    rgb c;

    c.red = 32; c.green = 64; c.blue = 32;
    for(i=0; i<MAX_REGIONS; i++) reg_color[i] = c;

    if(debug_image_colorize_all){
      for(i=0; i<MAX_COLORS; i++){
	reg = vision.getRegions(i);

	while(reg != NULL){
	  c = YuvToRgb(vision.getAverageColor(reg));
	  reg_color[vision.getRegionID(reg)] = c;
	  reg = reg->next;
	}
      }
    }
  }

  find_common(vision,cam,timestamp);

  if(find_calib_patterns){
    if(frame % find_calib_patterns_rate == 0){
      find_calib_pattern(vision,cam,timestamp);
    }
  }else{
    for(team=0; team<NUM_TEAMS; team++){
      switch(vconfig.teams[team].cover_type){
        case VCOVER_NONE:
	  find_opp_robots(team, nloc, loc, vision, cam, timestamp);
	  break;
        case VCOVER_NORMAL:
	  // rdtsc(start);
	  find_our_robots(team, nloc, loc, vision, cam, timestamp);
	  // rdtsc(end);
	  // sum = 0.90*sum + 0.10*(end - start);
	  // printf("%f\n",sum);
	  break;
        default:
	  break;
      }
    }

    find_ball(nloc,loc,vision,cam,timestamp);
  }

  if(dump_vision_stats){
    printf("FRAME %d:\n",frame);
    vision.dump();
    printf("  markers: %d/%d (%1.2f%%)\n",
	   orientation_markers.num_markers,
	   orientation_markers.max_markers,
	   100.0*orientation_markers.num_markers/
	   orientation_markers.max_markers);
  }

  if(save_debug_images){
    // rate limit debug output
    char fname[32];
    if(frame % save_debug_image_rate == 0){
      sprintf(fname,"cimg%03d.ppm",frame/save_debug_image_rate);
      vision.saveColorizedImage(fname,reg_color);
      printf("saved %s...\n",fname);
    }
  }

  // this defintely needs to change to EKBF usage - BB

  // Prove to me that an EKBF will always converge back to correct
  // values for all possible values of its state matrix it could reach
  // and I'll switch it :) - JB

  if(nloc.ball.conf > 0){
    // use new ball detection if reasonable
    s = speed(loc.ball.cur.loc ,loc.ball.timestamp,
	      nloc.ball.cur.loc,nloc.ball.timestamp);
    if(s < MAX_BALL_VELOCITY){
      loc.ball = nloc.ball;
    }
  }

  // use new robot detections if reasonable
  for(team=0; team<NUM_TEAMS; team++){
    for(i=0; i<MAX_TEAM_ROBOTS; i++){
      if(nloc.robot[team][i].conf > 0.0){
	s = speed( loc.robot[team][i].cur.loc, loc.robot[team][i].timestamp,
		   nloc.robot[team][i].cur.loc,nloc.robot[team][i].timestamp);
	delta = nloc.robot[team][i].cur.loc - loc.robot[team][i].cur.loc;

	if(s < MAX_ROBOT_VELOCITY){ // || loc.robot[team][i].conf<0.1){
	  loc.robot[team][i] = nloc.robot[team][i];
	}else if(loc.robot[team][i].conf<0.1){
	  // printf("Here %d %d\n",team,i); fflush(stdout);
	  loc.robot[team][i].cur.loc += delta.norm()*10;
	  loc.robot[team][i].conf = 0.0;
	}
      }else{
	// loc.robot[team][i].conf = 0.0;
      }
    }
  }

  loc.timestamp = timestamp;

  if(print_coordinates){
    for(team=0; team<NUM_TEAMS; team++){
      for(i=0; i<MAX_TEAM_ROBOTS; i++){
	if(loc.robot[team][i].conf > 0.0){
	  printf("%s[%d] conf=%6.4f (%8.2f,%8.2f):%6.3f t=%8.2f\n",
		 (team == TEAM_BLUE)? "blue" : "yellow",i,
		 loc.robot[team][i].conf,
		 loc.robot[team][i].cur.loc.x,
		 loc.robot[team][i].cur.loc.y,
		 loc.robot[team][i].cur.angle,
		 timestamp-loc.robot[team][i].timestamp);
	}
      }
    }
    printf("\n");
  }

  frame++;
}
