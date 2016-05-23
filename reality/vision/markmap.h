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

#ifndef __MARKMAP_H__
#define __MARKMAP_H__

#include <stdio.h>
#include "geometry.h"
#include "util.h"

#define MARKMAP_TEM template <class marker_t>
#define MARKMAP_FUN  MarkMap<marker_t>

/*
  needed support:
   - add region to markmap (after filtering out unwanted ones)
   - find closest N markers within certain distance of a point
  stats:
    low level:
      num_runs, num_regions, time
    high level:
      blue/yellow/white/green markers
*/

#define MM_POS(ix,iy,x,y) \
  (ix) = bound((int)(width  * ((x) + rad_x) / (2*rad_x)),0,width -1); \
  (iy) = bound((int)(height * ((y) + rad_y) / (2*rad_y)),0,height-1);

MARKMAP_TEM
class MarkMap{
  struct entry_t{
    int num;
    marker_t *list;
  };
public:
  entry_t *map;
  marker_t *marker;

  double rad_x,rad_y;
  int width,height;

  int max_markers,num_markers;
public:
  MarkMap() {map=NULL; width=height=0; rad_x=rad_y=1;
             marker=NULL; max_markers=num_markers=0;}
  ~MarkMap() {reset();}

  bool init(int _width,int _height,double _rad_x,double _rad_y,
	    int _max_markers);
  void reset();
  void clear();

  // add a marker to the map
  bool add(marker_t &m);

  // find closest <max> markers within radius <r> of point <p>
  int find(marker_t *arr,int max,vector2d p,double r);

  // find highest confidence <max> markers within radius <r> of point <p>
  int find_conf(marker_t *arr,int max,vector2d p,double r);

  // find upper bound on # of markers within radius <r> of point <p>
  int count(vector2d p,double r);

  marker_t &operator[](int index)
    {return(marker[index]);}

  void dump();
};

MARKMAP_TEM
bool MARKMAP_FUN::init(int _width,int _height,double _rad_x,double _rad_y,
		       int _max_markers)
{
  int size;

  width  = _width;
  height = _height;
  rad_x  = _rad_x;
  rad_y  = _rad_y;
  max_markers = _max_markers;
  num_markers = 0;

  size = width * height;
  delete[](map);
  delete[](marker);

  map = new entry_t[size];
  marker = new marker_t[max_markers];

  if(!map || !marker){
    reset();
    return(false);
  }

  return(true);
}

MARKMAP_TEM
void MARKMAP_FUN::reset()
{
  delete[](map);
  delete[](marker);

  rad_x = rad_y = 1;
  width = height = 0;
  max_markers = num_markers = 0;
}

MARKMAP_TEM
void MARKMAP_FUN::clear()
{
  int i,size;

  size = width * height;
  for(i=0; i<size; i++){
    map[i].num = 0;
    map[i].list = NULL;
  }

  num_markers = 0;
}

MARKMAP_TEM
bool MARKMAP_FUN::add(marker_t &m)
{
  int x,y,l;

  if(num_markers >= max_markers) return(false);

  MM_POS(x,y, m.loc.x,m.loc.y);
  l = y*width + x;

  marker[num_markers] = m;
  marker[num_markers].next = map[l].list;
  map[l].list = &marker[num_markers];
  map[l].num++;
  num_markers++;

  return(true);
}

MARKMAP_TEM
int MARKMAP_FUN::find(marker_t *arr,int max,vector2d p,double r)
{
  int x1,y1,x2,y2;
  marker_t *m;
  double d;
  int x,y,n,i,mi;

  MM_POS(x1, y1, p.x-r, p.y-r);
  MM_POS(x2, y2, p.x+r, p.y+r);

  n = mi = 0;
  for(y=y1; y<=y2; y++){
    for(x=x1; x<=x2; x++){
      m = map[y*width + x].list;
      while(m){
	d = Vector::distance(p,m->loc);
	if(d < r){
	  // add new marker if not set full, or nearer than furthest in set
	  if(n<max || d<arr[mi].dist){
	    arr[mi] = *m;
	    // printf("mi=%d (%f,%f) %X\n",mi,V2COMP(arr[mi].loc),arr[mi].reg);
	    arr[mi].dist = d;
	    if(n < max) n++;

	    // update index of furthest
	    if(n < max){
	      mi = n;
	    }else{
	      mi = 0;
	      for(i=1; i<n; i++){
		if(arr[i].dist > arr[mi].dist) mi = i;
	      }
	    }
	  }
	}
	m = m->next;
      }
    }
  }

  return(n);
}

// #include <stdio.h>

MARKMAP_TEM
int MARKMAP_FUN::find_conf(marker_t *arr,int max,vector2d p,double r)
{
  int x1,y1,x2,y2;
  marker_t *m;
  double d,c;
  int x,y,n,i,mi;

  MM_POS(x1, y1, p.x-r, p.y-r);
  MM_POS(x2, y2, p.x+r, p.y+r);

  // printf("[");
  n = mi = 0;
  for(y=y1; y<=y2; y++){
    for(x=x1; x<=x2; x++){
      m = map[y*width + x].list;
      while(m){
	c = m->conf;
	d = Vector::distance(p,m->loc);

	// printf("%4.2f,%f,%d,",c,d,mi);

	if(d < r){
	  // add new marker if not set full, or higher conf than min in set
	  if(n<max || c>arr[mi].conf){
	    arr[mi] = *m;
	    arr[mi].dist = d;
	    if(n < max) n++;

	    // update index of lowest conf
	    if(n < max){
	      mi = n;
	    }else{
	      mi = 0;
	      for(i=1; i<n; i++){
		if(arr[i].conf < arr[mi].conf) mi = i;
	      }
	    }
	  }
	}
	m = m->next;
      }
    }
  }

  if(false){
    printf("] => {");
    for(i=0; i<max; i++) printf("%4.2f ",arr[i].conf);
    printf("}\n");
  }

  return(n);
}

MARKMAP_TEM
int MARKMAP_FUN::count(vector2d p,double r)
{
  int x1,y1,x2,y2;
  int x,y,sum;

  MM_POS(x1, y1, p.x-r, p.y-r);
  MM_POS(x2, y2, p.x+r, p.y+r);

  sum = 0;
  for(y=y1; y<=y2; y++){
    for(x=x1; x<=x2; x++){
      sum += map[y*width + x].num;
    }
  }

  return(sum);
}

MARKMAP_TEM
void MARKMAP_FUN::dump()
{
  int x,y;

  printf("Total Markers = %d\n",num_markers);
  for(y=height-1; y>=0; y--){
    for(x=0; x<width; x++){
      printf("%2d",map[y*width + x].num);
    }
    printf("\n");
  }
  printf("\n");
}

#endif /* __MARKMAP_H__ */
