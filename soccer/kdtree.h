/*========================================================================
    KDTree.h : Template for fast online creation KD Trees
  ------------------------------------------------------------------------
    Copyright (C) 1999-2002  James R. Bruce
    School of Computer Science, Carnegie Mellon University
  ------------------------------------------------------------------------
    This software is distributed under the GNU General Public License,
    version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  ========================================================================*/

#ifndef __KD_TREE_H__
#define __KD_TREE_H__

#include "fast_alloc.h"
#include "geometry.h"

#define KDT_TEMP template <class state>
#define KDT_FUN  KDTree<state>

KDT_TEMP
class KDTree{
  struct node{
    vector2f minv,maxv; // bounding box of subtree
    state *states;      // list of states stored at this node
    int num_states;     // number of states of this subtree

    union{
      node *child[2]; // children of this tree
      node *next;
    };
  };

  node *root;
  int leaf_size,max_depth;
  int tests;
  fast_allocator<node> anode;

protected:
  inline bool inside(vector2f &minv,vector2f &maxv,state &s);
  inline float box_distance(vector2f &minv,vector2f &maxv,vector2f &p);

  void split(node *t,int split_dim);
  state *nearest(node *t,state *best,float &best_dist,vector2f &x);
  void clear(node *t);
public:
  KDTree() {root=NULL; leaf_size=max_depth=0;}

  bool setdim(vector2f &minv,vector2f &maxv,int nleaf_size,int nmax_depth);
  bool add(state *s);
  void clear();
  state *nearest(float &dist,vector2f &x);
};

KDT_TEMP
inline bool KDT_FUN::inside(vector2f &minv,vector2f &maxv,state &s)
{
  return(s.pos.x>minv.x && s.pos.y>minv.y &&
         s.pos.x<maxv.x && s.pos.y<maxv.y);
}

KDT_TEMP
inline float KDT_FUN::box_distance(vector2f &minv,vector2f &maxv,vector2f &p)
{
  float dx,dy;

  dx = p.x - bound(p.x,minv.x,maxv.x);
  dy = p.y - bound(p.y,minv.y,maxv.y);

  return(sqrt(dx*dx + dy*dy));
}

KDT_TEMP
void KDT_FUN::split(node *t,int split_dim)
{
  node *a,*b;
  state *p,*n;
  float split_val;

  // make new nodes
  a = anode.alloc();
  b = anode.alloc();
  if(!a || !b) return;
  a->child[0] = b->child[0] = NULL;
  a->child[1] = b->child[1] = NULL;
  a->states = b->states = NULL;
  a->num_states = b->num_states = 0;

  // determine split value
  a->minv = b->minv = t->minv;
  a->maxv = b->maxv = t->maxv;

  if(split_dim == 0){
    split_val = (t->minv.x + t->maxv.x) / 2;
    a->maxv.x = b->minv.x = split_val;
  }else{
    split_val = (t->minv.y + t->maxv.y) / 2;
    a->maxv.y = b->minv.y = split_val;
  }

  // separate children based on split
  n = t->states;
  while(p = n){
    n = n->next;

    if(((split_dim == 0)?p->pos.x : p->pos.y) < split_val){
      p->next = a->states;
      a->states = p;
      a->num_states++;
    }else{
      p->next = b->states;
      b->states = p;
      b->num_states++;
    }
  }

  // insert into tree
  t->states = NULL;
  t->child[0] = a;
  t->child[1] = b;
}

KDT_TEMP
bool KDT_FUN::setdim(vector2f &minv,vector2f &maxv,int nleaf_size,int nmax_depth)
{
  clear();
  if(!root) root = anode.alloc();
  if(!root) return(false);
  mzero(*root);
  root->minv = minv;
  root->maxv = maxv;
  leaf_size = nleaf_size;
  max_depth = nmax_depth;
  return(true);
}

KDT_TEMP
bool KDT_FUN::add(state *s)
{
  node *p;
  int c,level;

  level = 0;
  p = root;
  if(!p || !inside(p->minv,p->maxv,*s)) return(false);

  // go down tree to see where new state should go
  while(p->child[0]){ // implies p->child[1] also
    c = !inside(p->child[0]->minv,p->child[0]->maxv,*s);
    p = p->child[c];
    level++;
  }

  // add it to leaf; and split leaf if too many children
  s->next = p->states;
  p->states = s;
  p->num_states++;

  // split leaf if not too deep and too many children for one node
  if(level<max_depth && p->num_states>leaf_size){
    split(p,level % 2);
  }
  return(true);
}

KDT_TEMP
void KDT_FUN::clear(node *t)
{
  if(!t) return;
  if(t->child[0]) clear(t->child[0]);
  if(t->child[1]) clear(t->child[1]);

  t->child[0] = t->child[1] = NULL;
  t->states = NULL;
  t->num_states = 0;

  anode.free(t);
}

KDT_TEMP
void KDT_FUN::clear()
{
  if(!root) return;

  clear(root->child[0]);
  clear(root->child[1]);

  root->child[0] = root->child[1] = NULL;
  root->states = NULL;
  root->num_states = 0;
}

KDT_TEMP
state *KDT_FUN::nearest(node *t,state *best,float &best_dist,vector2f &x)
{
  float d,dc[2];
  state *p;
  int c;

  // look at states at current node
  p = t->states;
  while(p){
    d = Vector::distance(p->pos,x);
    if(d < best_dist){
      best = p;
      best_dist = d;
    }
    tests++;
    p = p->next;
  }

  // recurse on children (nearest first to maximize pruning)
  if(t->child[0]){ // implies t->child[1]
    dc[0] = box_distance(t->child[0]->minv,t->child[0]->maxv,x);
    dc[1] = box_distance(t->child[1]->minv,t->child[1]->maxv,x);
    c = dc[1] < dc[0]; // c indicates nearest lower bound distance child

    if(dc[ c] < best_dist) best = nearest(t->child[ c],best,best_dist,x);
    if(dc[!c] < best_dist) best = nearest(t->child[!c],best,best_dist,x);
  }

  return(best);
}

KDT_TEMP
state *KDT_FUN::nearest(float &dist,vector2f &x)
{
  state *best;

  best = NULL;
  dist = 4000;

  tests = 0;
  best = nearest(root,best,dist,x);
  // printf("tests=%d dist=%f\n\n",tests,best_dist);

  return(best);
}

#endif /*__KD_TREE_H__*/
