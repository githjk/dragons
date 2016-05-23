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

#include "geometry.h"
#include "constants.h"

#include "obstacle.h"


//====================================================================//
//    Obstacle class implementation
//====================================================================//

vector2f obstacle::closest_point(state s)
{
  vector2f p,d;
  double o;

  switch(type){
    case OBS_CIRCLE:
      d = (s.pos - pos).norm();
      p = pos + d*rad.x;
      break;
    case OBS_RECTANGLE:
      p.set(bound(s.pos.x,pos.x-rad.x,pos.x+rad.x),
            bound(s.pos.y,pos.y-rad.y,pos.y+rad.y));
      break;
    case OBS_HALF_PLANE:
      o = (s.pos - pos).dot(rad);
      p = (o > 0)? s.pos - rad*o : p = s.pos;
      break;
    default:
      p.set(0,0);
  }

  return(p);
}

float obstacle::margin(state s)
{
  vector2f p;
  double d;

  switch(type){
    case OBS_CIRCLE:
      d = Vector::distance(pos,s.pos);
      return(d - rad.x - ROBOT_RADIUS);
    case OBS_RECTANGLE:
      p.set(bound(s.pos.x,pos.x-rad.x,pos.x+rad.x),
            bound(s.pos.y,pos.y-rad.y,pos.y+rad.y));
      d = Vector::distance(p,s.pos);
      return(d - ROBOT_RADIUS);
    case OBS_HALF_PLANE:
      d = (s.pos - pos).dot(rad);
      return(max(d - ROBOT_RADIUS,0.0));
  }

  return(0.0);
}

bool obstacle::check(state s)
{
  float dx,dy;

  if(type==OBS_CIRCLE || type==OBS_RECTANGLE){
    dx = fabs(s.pos.x - pos.x);
    dy = fabs(s.pos.y - pos.y);
    if(dx>rad.x+ROBOT_RADIUS || dy>rad.y+ROBOT_RADIUS) return(true);
  }

  return(margin(s) > 0.0);
}

bool obstacle::check(state s0,state s1)
{
  vector2f p;
  vector2f c[4];
  int i;
  float d;

  d = Vector::distance(s0.pos,s1.pos);
  if(d < EPSILON) return(check(s0));

  switch(type){
    case OBS_CIRCLE:
      p = Vector::point_on_segment(s0.pos,s1.pos,pos);

      if(false){
        printf("pos(%f,%f)-(%f,%f):(%f,%f)=(%f,%f)\n",
               s0.pos.x,s0.pos.y,
               s1.pos.x,s1.pos.y,
               pos.x,pos.y,
               p.x,p.y);
      }

      d = Vector::distance(p,pos);
      return(d > rad.x+ROBOT_RADIUS);

    case OBS_RECTANGLE:
      c[0].set(pos.x - rad.x,pos.y - rad.y);
      c[1].set(pos.x + rad.x,pos.y - rad.y);
      c[2].set(pos.x + rad.x,pos.y + rad.y);
      c[3].set(pos.x - rad.x,pos.y + rad.y);

      // check box against oriented sweep
      for(i=0; i<4; i++){
        d = Vector::distance_seg_to_seg(s0.pos,s1.pos,c[i],c[(i+1)%4]);
        // printf("%f \n",d);
        if(d < ROBOT_RADIUS) return(false);
      }
      // printf("\n");

      return(check(s0));

      /*
      n = (s1.pos - s0.pos).norm();
      n.set(-n.y,n.x);
      p = s0.pos;
      o = n.dot(p);

      p.set(pos.x - rad.x,pos.y - rad.y);
      if(n.dot(p) - o < ROBOT_RADIUS) return(false);
      p.set(pos.x + rad.x,pos.y - rad.y);
      if(n.dot(p) - o < ROBOT_RADIUS) return(false);
      p.set(pos.x - rad.x,pos.y + rad.y);
      if(n.dot(p) - o < ROBOT_RADIUS) return(false);
      p.set(pos.x + rad.x,pos.y + rad.y);
      if(n.dot(p) - o < ROBOT_RADIUS) return(false);

      return(check(s0) && check(s1));
      */
    case OBS_HALF_PLANE:
      return((s0.pos - pos).dot(rad)>ROBOT_RADIUS &&
             (s1.pos - pos).dot(rad)>ROBOT_RADIUS);
  }

  return(true);
}

vector2f obstacle::repulse(state s)
{
  vector2f p,v;
  float d,f;

  if(type==OBS_CIRCLE) return((s.pos-pos).norm());
  if(type==OBS_HALF_PLANE) return(rad);
  return((s.pos-pos).norm());

  p = closest_point(s);
  d = Vector::distance(p,s.pos);

  // if(type==OBS_RECTANGLE) printf("d=%f\n",d);

  if(d < EPSILON){
    if(type==OBS_HALF_PLANE) return(rad);
    // printf("ouch!\n");
    return((s.pos-pos).norm());
  }

  f = (d < ROBOT_RADIUS)? 1.0 : 1.0 / (1.0 + d/ROBOT_RADIUS);
  v = (s.pos - ((d > EPSILON)? p : pos)).norm();

  return(v * f);
}

//====================================================================//
//    Obstacles class implementation
//====================================================================//

void obstacles::add_rectangle(float cx,float cy,float w,float h,int mask)
{
  if(num >= MAX_OBSTACLES) return;

  obs[num].type = OBS_RECTANGLE;
  obs[num].mask = mask;
  obs[num].pos.set(cx,cy);
  obs[num].rad.set(w/2,h/2);
  obs[num].vel.set(0,0);

  num++;
}

void obstacles::add_circle(float x,float y,float radius,
			   float vx,float vy,int mask)
{
  if(num >= MAX_OBSTACLES) return;

  obs[num].type = OBS_CIRCLE;
  obs[num].mask = mask;
  obs[num].pos.set(x,y);
  obs[num].rad.set(radius,radius);
  obs[num].vel.set(vx,vy);

  num++;
}

void obstacles::add_half_plane(float x,float y,float nx,float ny,int mask)
{
  if(num >= MAX_OBSTACLES) return;

  obs[num].type = OBS_HALF_PLANE;
  obs[num].mask = mask;
  obs[num].pos.set(x,y);
  obs[num].rad.set(nx,ny);
  obs[num].vel.set(0,0);

  num++;
}

bool obstacles::check(vector2d p)
{
  state s;
  s.pos.set(p.x,p.y);
  s.parent = NULL;
  return(check(s));
}

bool obstacles::check(vector2d p,int &id)
{
  state s;
  s.pos.set(p.x,p.y);
  s.parent = NULL;
  return(check(s,id));
}

bool obstacles::check(state s)
{
  int i;

  i = 0;
  while(i<num && (obs[i].mask&current_mask==0 || obs[i].check(s))) i++;

  return(i == num);
}

bool obstacles::check(state s,int &id)
{
  int i;

  i = 0;
  while(i<num && (obs[i].mask&current_mask==0 || obs[i].check(s))) i++;
  if(i < num) id = i;

  return(i == num);
}

bool obstacles::check(state s0,state s1)
{
  int i;

  i = 0;
  while(i<num && (obs[i].mask&current_mask==0 || obs[i].check(s0,s1))){
    // printf("%d",obs[i].check(s0,s1));
    i++;
  }
  // printf("\n");

  return(i == num);
}

bool obstacles::check(state s0,state s1,int &id)
{
  int i;

  i = 0;
  while(i<num && (obs[i].mask&current_mask==0 || obs[i].check(s0,s1))) i++;
  if(i < num) id = i;

  return(i == num);
}

vector2f obstacles::repulse(state s)
{
  vector2f f;
  int i;

  f.set(0.0,0.0);
  for(i=0; i<num; i++){
    if((obs[i].mask & current_mask) && !obs[i].check(s)){
      f += obs[i].repulse(s);
    }
  }

  return(f);
}
