/*========================================================================
    Camera.cc: Internal/external geometric camera calibration for CMVision2
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

#include <stdlib.h>
#include <stdio.h>

#define CALIBRATE

#ifdef CALIBRATE
#include <vector>
#endif

#include "geometry.h"
#include "search.h"
#include "camera.h"


void camera::getState(double *param)
{
  param[ 0] = loc.x;
  param[ 1] = loc.y;
  param[ 2] = scale_x.x;
  param[ 3] = scale_x.y;
  param[ 4] = scale_x.z;
  param[ 5] = scale_y.x;
  param[ 6] = scale_y.y;
  param[ 7] = scale_y.z;
  param[ 8] = a;
  param[ 9] = b;

  y_mult = 1;
  field = 0;
}

void camera::setState(double *param)
{
  loc.set(param[0],param[1],loc.z);
  scale_x.set(param[2],param[3],param[4]);
  scale_y.set(param[5],param[6],param[7]);
  a = param[8];
  b = param[9];

  image = cross(scale_y,scale_x).norm();
  y_mult = 1;
  field = 0;
}

bool camera::loadParam(char *filename)
{
  FILE *in;

  in = fopen(filename,"rt");
  if(!in) return(false);

  fscanf(in,"l(%lf,%lf,%lf)\nx(%lf,%lf,%lf)\ny(%lf,%lf,%lf)\n",
         &loc.x,&loc.y,&loc.z,
         &scale_x.x,&scale_x.y,&scale_x.z,
         &scale_y.x,&scale_y.y,&scale_y.z);
  fscanf(in,"r(%lf,%lf) %dx%d a%lf\n",
         &a,&b,&width,&height,&aspect);

  image = cross(scale_y,scale_x).norm();
  y_mult = 1;
  field = 0;

  return(fclose(in) == 0);
}

bool camera::saveParam(char *filename)
{
  FILE *out;

  out = fopen(filename,"wt");
  if(!out) return(false);

  fprintf(out,"l(%f,%f,%f)\nx(%f,%f,%f)\ny(%f,%f,%f)\n",
          loc.x,loc.y,loc.z,
          scale_x.x,scale_x.y,scale_x.z,
          scale_y.x,scale_y.y,scale_y.z);
  fprintf(out,"r(%f,%f) %dx%d a%f\n",
          a,b,width,height,aspect);
  return(fclose(out) == 0);
}

void camera::print()
{
  printf("l(%f,%f,%f) x(%f,%f,%f) y(%f,%f,%f) r(%f,%f) %dx%d a%f \n",
         loc.x,loc.y,loc.z,
         scale_x.x,scale_x.y,scale_x.z,
         scale_y.x,scale_y.y,scale_y.z,
         a,b,width,height,aspect);
}

vector3d camera::screenToRay(double sx,double sy)
{
  vector2d p;
  vector3d d;
  double rr,rd;

  // map image coord into centered normalized cartesion coord
  sy = sy*y_mult + field;
  p.x =  (sx - (width /2)) / ((width /2));
  p.y = -(sy - (height/2)) / ((height/2) * aspect);

  // remove radial distortion
  rr = p.sqlength();
  rd = 1.0 + a*rr + b*sqrt(rr);
  p *= 1.0 / rd;

  // find ray such that w = camera + d*t
  d = image + scale_x*p.x + scale_y*p.y;

  return(d);
}

vector2d camera::screenToWorld(double sx,double sy,double wz)
{
  vector3d d;
  vector2d w;
  double t;
  // double r,rd,r1,r2;

  // get vector from camera origin
  d = screenToRay(sx,sy);

  // now find t using world coordinate z
  t = (wz - loc.z) / (d.z + EPSILON);

  // now project ray to world point
  w.x = loc.x + d.x * t;
  w.y = loc.y + d.y * t;

  /*
  // inverse of above radial distortion calculation
  r = p.sqlength();
  rc = sqrt(b*b + 4*a*r - 4*a);
  r1 = (-b + rc) / (2*a);
  r2 = (-b - rc) / (2*a);
  */

  return(w);
}

vector2d camera::worldToScreen(vector3d wp)
{
  vector3d r,sw;
  vector2d sp;

  // get ray to image plane
  r = wp - loc;
  r /= r.z;

  // pixel position (in world space) on image plane
  sw = r - image;

  sp.x = dot(sw,scale_x);
  sp.y = dot(sw,scale_y);
  sp.y = (sp.y - field) / y_mult;

  return(sp);
}

void erase_eol()
{
  printf("%c[K",0x1b);
}

void reset_cursor()
{
  printf("%c%c%c",
         0x1b,0x5b,0x48);
         // 0x1b,0x5b,0x32,0x4a);
}

void clear_screen()
{
  printf("%c%c%c%c%c%c%c",
         0x1b,0x5b,0x48,
         0x1b,0x5b,0x32,0x4a);
}

void camera::calibrate(vector2d *screen,vector3d *world,int num)
{
#ifdef CALIBRATE

  double state[10];
  camera_calibrate cam;
  vector2d wp;
  double ex,ey,err,sum;
  int i,j,e;
  int steps,iter;

  getState(state);
  cam.loc     = loc;
  cam.scale_x = scale_x;
  cam.scale_y = scale_y;
  cam.a = a;
  cam.b = b;

  cam.width  = width;
  cam.height = height;
  cam.aspect = aspect;

  cam.screen = screen;
  cam.world  = world;
  cam.num_points = num;

  clear_screen();
  steps = 10;
  iter  = 0;

  while(1){
    cam.print();
    Search::GradientSearch2<camera_calibrate,10>(cam,state,steps);
    iter += steps;
    if(steps < 1000) steps += 10;
    // Search::BlindSearch<camera_calibrate,10>(cam,state,10000);
    cam.setState(state);
    reset_cursor();

    sum = 0.0;
    for(i=0; i<num; i++){
      wp = cam.screenToWorld(screen[i].x,screen[i].y,world[i].z);
      ex = wp.x - world[i].x;
      ey = wp.y - world[i].y;
      err = ex*ex + ey*ey;
      sum += err;

      if(num<32 || ((i+5)%9) == 0){
	printf("(%8.2f, %8.2f) : %8.2f ",wp.x,wp.y,sqrt(err));
	e = min((int)(sqrt(err) + 0.5),46);
	for(j=0; j<e; j++) printf("*");
	erase_eol();
	printf("\n");
      }
    }

    printf("Avg Error: %g  \n",sqrt(sum/num));
    printf("Iterations: %d  \n",iter);
  }

  setState(state);

#endif
}


void camera::calibrate(char *filename)
{
#ifdef CALIBRATE

  char buf[256];
  FILE *in;

  vector<vector2d> screen;
  vector<vector3d> world;
  vector2d sp;
  vector3d wp;
  int num;


  in = fopen(filename,"rt");
  if(!in) return;

  num = 0;
  while(fgets(buf,256,in)){
    if(buf[0]!='#' && buf[0]!='\n'){
      if(sscanf(buf,"(%lf,%lf,%lf) (%lf,%lf)",
                &wp.x,&wp.y,&wp.z,
                &sp.x,&sp.y) == 5){
        screen.push_back(sp);
        world.push_back(wp);
        num++;
      }else if(sscanf(buf,"  (%lf,%lf) (%lf,%lf,%lf)",
                &sp.x,&sp.y,
		&wp.x,&wp.y,&wp.z) == 5){
	sp.y*=2;
        screen.push_back(sp);
        world.push_back(wp);
	printf("  (%f,%f) (%f,%f,%f)\n",sp.x,sp.y,wp.x,wp.y,wp.z);
        num++;
      }
    }
  }
  // exit(0);

  calibrate(&screen[0],&world[0],num);

#endif
}

double camera_calibrate::eval(double *state)
{
  vector2d p;
  double ex,ey,err,d;
  int i;

  setState(state);

  err = 0.0;
  for(i=0; i<num_points; i++){
    p = screenToWorld(screen[i].x,screen[i].y,world[i].z);
    ex = p.x - world[i].x;
    ey = p.y - world[i].y;
    // printf("(%f,%f)\n",ex,ey);
    d = ex*ex + ey*ey;
    // if(d > err) err = d;
    err += d; // sqrt(d); // pow(d,8);
  }

  return(err);
}
