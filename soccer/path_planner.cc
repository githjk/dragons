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
#include "win.h"

#include "obstacle.h"
#include "path_planner.h"

/*
extern xwin field;
void DrawLine(xdrawable &w,double x1,double y1,double x2,double y2);
*/

const bool plan_print = false;
const double out_of_obs_dot = 0.40; // 0.1; // 0.7071;

double sdrand48()
{
  return(2*drand48()-1);
}

state path_planner::random_state()
{
  state s;

  s.pos.set((FIELD_LENGTH_H+GOAL_DEPTH)*sdrand48(),FIELD_WIDTH_H*sdrand48());
  // s.vel.set(0,0);
  s.parent = NULL;

  return(s);
}


void path_planner::init(int _max_nodes,int _num_waypoints,
                        double _goal_target_prob,
                        double _waypoint_target_prob,
                        double _step_size)
{
  int i;
  vector2f minv,maxv;

  max_nodes = _max_nodes;
  num_waypoints = _num_waypoints;
  goal_target_prob = _goal_target_prob;
  waypoint_target_prob = _waypoint_target_prob;
  step_size = _step_size;

  for(i=0; i<num_waypoints; i++){
    waypoint[i] = random_state();
  }

  mzero(out_of_obs);

  minv.set(-FIELD_LENGTH_H-GOAL_DEPTH,-FIELD_WIDTH_H);
  maxv.set( FIELD_LENGTH_H+GOAL_DEPTH, FIELD_WIDTH_H);
  tree.setdim(minv,maxv,16,8);
}

double path_planner::distance(state &s0,state &s1)
{
  float dx,dy;
  dx = s1.pos.x - s0.pos.x;
  dy = s1.pos.y - s0.pos.y;
  return(sqrt(dx*dx + dy*dy));
  // return(Vector::distance(s0.pos,s1.pos));
}

state *path_planner::add_node(state n,state *parent)
{
  if(num_nodes >= max_nodes) return(NULL);

  n.parent = parent;
  node[num_nodes] = n;
  tree.add(&node[num_nodes]);
  num_nodes++;

  /*
  if(parent){
    // field.setColor(0,0,0);
    // DrawLine(field,parent->pos.x,parent->pos.y,n.pos.x,n.pos.y);
  }
  */

  return(&node[num_nodes-1]);
}

state path_planner::choose_target(int &targ)
{
  double p = drand48();
  int i;

  if(p < goal_target_prob){
    targ = 0;
    return(goal);
  }else if(p < goal_target_prob+waypoint_target_prob){
    targ = 1;
    i = lrand48() % num_waypoints;
    return(waypoint[i]);
  }else{
    targ = 2;
    return(random_state());
  }
}

state *path_planner::find_nearest(state target)
{
  state *nearest;
  double d,nd;
  float td;
  int i;

  td = 0;
  nearest = tree.nearest(td,target.pos);

  if(!nearest){
    // NOTE: something bad must have happened if we get here.

    // find closest current state
    nearest = &node[0]; // num_nodes-1];
    nd = distance(*nearest,target);

    for(i=0; i<num_nodes; i++){
      d = distance(node[i],target);
      if(d < nd){
        nearest = &node[i];
        nd = d;
      }
    }
  }

  return(nearest);
}

int path_planner::extend(state *s,state target)
{
  state n;
  vector2f step,p;
  vector2f f,fg;
  int num = 0;
  int id;
  double d,a,r;

  step = target.pos - s->pos;
  d = step.length();
  if(d < step_size) return(0);
  step *= step_size / d;

  n.pos = s->pos + step;
  // n.vel = s->vel;

  if(!obs->check(*s,id)){
    f  = obs->repulse(*s);
    fg = n.pos - s->pos;
    if(f.dot(fg) > out_of_obs_dot){
      // leaving obstacle, ok
      // printf("out <%f,%f> <%f,%f> %f\n",V2COMP(f),V2COMP(fg),f.dot(fg));
    }else{
      // fail if would go into obstacle more
      return(num);
    }
  }else if(!obs->check(n,id)){
    if(obs->obs[id].type == OBS_CIRCLE){
      // find tangent angle
      p = obs->obs[id].pos;
      r = max(obs->obs[id].rad.x,obs->obs[id].rad.y);
      d = Vector::distance(n.pos,p);
      a = asin(max(r,d) / d);

      // try each one
      n.pos = s->pos + step.rotate(a);
      if(!obs->check(n)){
        n.pos = s->pos + step.rotate(-a);
        if(!obs->check(n)) return(num);
      }
    }
  }

  // add node
  s = add_node(n,s);
  num++;
  return(num);
}

state path_planner::plan(obstacles *_obs,int obs_mask,
                         state initial,state _goal,int &obs_id)
{
  state target,*nearest,*nearest_goal,*p,*head;
  vector2f f;
  double d,nd,s;
  int i,iter_limit;
  int targ_type;
  bool ok;
  bool inobs;

  goal = _goal;
  obs = _obs;
  obs->set_mask(obs_mask);
  inobs = false;

  tree.clear();

  // check for small trivial plan
  d = Vector::distance(initial.pos,goal.pos);
  if(d < NEAR){
    if(plan_print) printf("  PP: short\n");
    target = goal;
    s = 1.0;
    do{
      target.pos = initial.pos*(1-s) + goal.pos*s;
      ok = obs->check(initial,target);
      s -= 0.1;
    }while(s>0 && !ok);

    return(target);
  }

  if(obs->check(initial,goal)){
    if(plan_print) printf("  PP: no obs\n");
    // no obstacles in the way
    return(goal);
  /*
  }else if(!obs->check(initial)){
    if(plan_print) printf("  PP: obs avoid\n");
    // in an obstacle, use gradient to leave
    target = initial;
    f = obs->repulse(initial);
    fg = goal.pos - initial.pos;
    // printf("[%f,%f],[%f,%f]",V2COMP(f),V2COMP(fg));
    if(f.dot(fg) > 0){
      // already leaving the obstacle
      // printf(".");
      target.pos += fg.norm()*100;
      inobs = true;
    }else{
      // leave obstacle and move at target
      target.pos += f.norm()*100 + fg.norm()*100;
      inobs = true;
    }
    return(target);
  */
  }else{
    if(plan_print){
      printf("  PP: plan(%f,%f)->(%f,%f)\n",
             V2COMP(initial.pos),V2COMP(goal.pos));
    }

    i = num_nodes = 0;
    nearest = nearest_goal = add_node(initial,NULL);
    d = distance(*nearest,goal);

    // plan
    iter_limit = max_nodes;
    while(i<iter_limit && num_nodes<max_nodes && d>NEAR){
      target = choose_target(targ_type);
      nearest = (targ_type == 0)? nearest_goal : find_nearest(target);
      extend(nearest,target);

      nd = distance(node[num_nodes-1],goal);
      if(nd < d){
        nearest_goal = &node[num_nodes-1];
        d = nd;
      }
      i++;
    }

    inobs = !obs->check(initial);

    // trace back up plan to find simple path
    p = nearest_goal;
    if(!inobs){
      while(p!=NULL && !obs->check(initial,*p,obs_id)) p = p->parent;
    }else{
      f = obs->repulse(initial);

      if(false){
        printf("BACKTRACE:\n");
        while(p!=NULL){
          printf("  <%f,%f> %f\n",V2COMP((p->pos - initial.pos)),
                 f.dot(p->pos - initial.pos));
          p = p->parent;
        }
      }

      p = nearest_goal;
      while(p!=NULL && p->parent!=NULL &&
            f.dot(p->pos - initial.pos)<out_of_obs_dot){
        p = p->parent;
      }
    }
    head = p;

    if(head){
      target = *head;
    }else{
      target = initial;
    }

    if(plan_print){
      if(nearest_goal){
        printf("  nearest_goal(%f,%f)\n",V2COMP(nearest_goal->pos));
      }
      printf("  goal(%f,%f)\n",goal.pos.x,goal.pos.y);
      printf("  target(%f,%f)\n",target.pos.x,target.pos.y);
    }

    /*
    field.setColor(192,192,192);
    p = nearest_goal;
    while(p!=NULL && p->parent!=NULL){
      DrawLine(field,p->parent->pos.x,p->parent->pos.y,p->pos.x,p->pos.y);
      p = p->parent;
    }
    */

    // put in waypoint cache if solution
    if(num_waypoints > 0){
      if(p!=NULL && ((d < NEAR) || drand48()<0.1)){
        p = nearest_goal;
        while(p != NULL){
          i = lrand48()%num_waypoints;
          waypoint[i] = *p;
          waypoint[i].parent = NULL;
          if(p == head) break;
          p = p->parent;
        }
      }else{
        i = lrand48()%num_waypoints;
        waypoint[i] = random_state();
      }
    }

    return(target);
  }
}
