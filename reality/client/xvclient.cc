/*
 * TITLE:	XClient.cc
 *
 * PURPOSE:	This file provides the ximple X gui to the vision/driving routines
 *
 * WRITTEN BY:  James R Bruce, Brett Browning
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

#include "win.h"
#include "constants.h"
#include "client.h"


#define DISPLAY_WIDTH    (400)
#define DISPLAY_HEIGHT   (300)

#define CONF_GRAPH_Y (DISPLAY_HEIGHT-NUM_TEAMS*MAX_TEAM_ROBOTS-1)

#define DISPLAY_WIDTH_H  (DISPLAY_WIDTH/2)
#define DISPLAY_HEIGHT_H (DISPLAY_HEIGHT/2)
#define FIELD_CENTER_Y   120

rgb black = {0,0,0};

const bool ball_image = true;

const int image_width  = 320;
const int image_height = 240;
const double image_scale = 0.1;

// Globals
rgb *ball_rgb;
int col;


void clear(rgb *img,rgb c)
{
  int i;

  for(i=0; i<image_width*image_height; i++){
    img[i] = c;
  }
}

void setpixel(rgb *img,double x,double y,rgb c)
{
  int ix,iy;

  ix = (int)( x * image_scale + image_width /2);
  iy = (int)(-y * image_scale + image_height/2);

  if(ix<0 || iy<0 || ix>=image_width || iy>=image_height) return;

  img[iy*image_width + ix] = c;
}

int WritePPM(char *filename,rgb *img,int width,int height)
{
  FILE *out;
  int wrote;

  out = fopen(filename,"wb");
  if(!out) return(0);

  fprintf(out,"P6\n%d %d\n%d\n",width,height,255);
  wrote = fwrite(img,sizeof(rgb),width*height,out);
  fclose(out);

  return(wrote);
}

bool save(char *filename,rgb *img)
{
  return(WritePPM(filename,img,image_width,image_height) ==
         image_width * image_height);
}


void DrawLine(xdrawable &w,double x1,double y1,double x2,double y2)
{
  w.drawLine((int)(DISPLAY_WIDTH_H  + x1/10 + 0.5),
             (int)(FIELD_CENTER_Y   - y1/10 + 0.5),
             (int)(DISPLAY_WIDTH_H  + x2/10 + 0.5),
             (int)(FIELD_CENTER_Y   - y2/10 + 0.5));
}

void DrawRobot(xdrawable &w,double x,double y,double a,int id,int team)
{
  XPoint p[8],r[8];
  double sn,cs;
  int i,num;
  char name[2];

  r[0].x =  53;  r[0].y = -75;
  r[1].x =  53;  r[1].y =  75;
  r[2].x = -42;  r[2].y =  75;
  r[3].x = -67;  r[3].y =  50;
  r[4].x = -67;  r[4].y = -50;
  r[5].x = -42;  r[5].y = -75;
  r[6].x =  53;  r[6].y = -75;
  num = 7;

  w.setColor(64,64,64);
  w.fillCircle((int) x, (int) y,10);

  sn = sin(a);
  cs = cos(a);

  // Rotate points
  for(i=0; i<num; i++){
    p[i].x = (int)(x + cs*r[i].x/10.0 - sn*r[i].y/10.0);
    p[i].y = (int)(y - sn*r[i].x/10.0 - cs*r[i].y/10.0);
    // printf("point[%d] = (%d,%d)\n",i,p[i].x,p[i].y);
  }

  // Draw center
  switch(team){
    case TEAM_BLUE:   w.setColor(  0,  0,192); break;
    case TEAM_YELLOW: w.setColor(192,192,  0); break;
    default: w.setColor(128);
  }
  w.fillPolygon(p,num);

  // Draw outline
  w.setColor(0);
  w.drawLines(p,num);
  w.setLineWidth(2);
  w.drawLine((int) x, (int) y,(int)(x + cs*5.3),(int)(y - sn*5.3));
  // (p[0].x+p[1].x+1)/2,(p[0].y+p[1].y+1)/2);
  w.setLineWidth(1);

  // w.fillCircle((int)((x + cs*4)/10),(int)((y - sn*4)/10),2);
  // w.fillCircle((int)(x + cs*4),(int)(y - sn*4),2);
  // w.fillCircle(x,y,4);

  // Draw number
  name[0] = '0' + id;
  name[1] = 0;
  w.setColor(255);
  w.print((int) x - 12, (int) y - 6,name);
  // w.print(x-1,y+3,name);
}

void DrawBall(xdrawable &w,int x,int y)
{
  w.setColor(0);
  w.fillCircle(x,y,3);

  w.setColor(192,96,0);
  w.fillCircle(x,y,2);

  w.setColor(255,128,0);
  w.fillCircle(x,y,1);

  //printf("(%d,%d)\n",x,y);
}

void DrawField(xdrawable &w,int x,int y)
{
  // clear to black
  w.setColor(0);
  w.fillRectangle(0,0,DISPLAY_WIDTH,DISPLAY_HEIGHT);

  // Draw outside walls
  w.setColor(192);
  w.fillRectangle(x - (FIELD_LENGTH/2 + WALL_WIDTH)/10,
		  y - (FIELD_WIDTH/2  + WALL_WIDTH)/10,
		  (FIELD_LENGTH + WALL_WIDTH*2)/10,
		  (FIELD_WIDTH  + WALL_WIDTH*2)/10);
  w.fillRectangle(x - (FIELD_LENGTH/2 + GOAL_DEPTH + WALL_WIDTH)/10,
		  y - (GOAL_WIDTH/2 + WALL_WIDTH)/10,
		  (GOAL_DEPTH + WALL_WIDTH)/10,
		  (GOAL_WIDTH + WALL_WIDTH*2)/10);
  w.fillRectangle(x + (FIELD_LENGTH/2)/10,
		  y - (GOAL_WIDTH/2 + WALL_WIDTH)/10,
		  (GOAL_DEPTH + WALL_WIDTH)/10,
		  (GOAL_WIDTH + WALL_WIDTH*2)/10);

  // Draw field
  w.setColor(0,96,48);
  w.fillRectangle(x - (FIELD_LENGTH_H)/10,
		  y - (FIELD_WIDTH_H)/10,
		  (FIELD_LENGTH)/10,
		  (FIELD_WIDTH)/10);

  // Draw lines
  w.setColor(0,80,40);
  w.fillRectangle(x - (10)/10,
		  y - (FIELD_WIDTH_H)/10,
		  (20)/10,
		  (FIELD_WIDTH)/10);

  /*
  // Draw lines
  w.setColor(192);
  w.drawRectangle(x - FIELD_LENGTH_H - 1,
                  y - DEFENSE_WIDTH_H,
                  DEFENSE_DEPTH + 1,
                  DEFENSE_WIDTH);
  w.drawRectangle(x + FIELD_LENGTH_H - DEFENSE_DEPTH,
                  y - DEFENSE_WIDTH_H,
                  DEFENSE_DEPTH + 1,
                  DEFENSE_WIDTH);
  w.drawLine(x, y - FIELD_WIDTH_H,
	     x, y + FIELD_WIDTH_H - 1);
  // w.drawCircle(x, y, CENTER_DIA/2);
  */

  // Draw goals
  w.setColor(192,192,0);
  w.fillRectangle(x - (FIELD_LENGTH_H + GOAL_DEPTH)/10,
		  y - (GOAL_WIDTH/2)/10,
		  (GOAL_DEPTH)/10,
		  (GOAL_WIDTH)/10);
  w.setColor(0,0,192);
  w.fillRectangle(x + (FIELD_LENGTH_H)/10,
		  y - (GOAL_WIDTH/2)/10,
		  (GOAL_DEPTH)/10,
		  (GOAL_WIDTH)/10);

  w.setColor(0,80,40);
  w.fillRectangle(x - FIELD_LENGTH_H/10,
		  y - DEFENSE_WIDTH_H/10,
		  DEFENSE_DEPTH/10,
		  DEFENSE_WIDTH/10);
  w.fillRectangle(x + (FIELD_LENGTH_H - DEFENSE_DEPTH)/10,
		  y - DEFENSE_WIDTH_H/10,
		  DEFENSE_DEPTH/10,
		  DEFENSE_WIDTH/10);

  /*
  // Draw Shadow Lines
  w.setColor(0);
  w.drawRectangle(x - FIELD_LENGTH/2 - GOAL_DEPTH, y - GOAL_WIDTH/2,
		  GOAL_DEPTH, GOAL_WIDTH);
  w.drawRectangle(x + FIELD_LENGTH/2, y - GOAL_WIDTH/2,
		  GOAL_DEPTH, GOAL_WIDTH);
  w.drawRectangle(x - FIELD_LENGTH/2, y - FIELD_WIDTH/2,
		  FIELD_LENGTH, FIELD_WIDTH);
  */
}

void DrawField(xdrawable &w)
{
  int cx,cy;

  cx = DISPLAY_WIDTH_H;
  cy = FIELD_CENTER_Y;

  DrawField(w,cx,cy);
}

void RedrawField(xwin &w)
{
  w.clearArea(0,0,DISPLAY_WIDTH,DISPLAY_HEIGHT,false);
}



vector2f ball_old;
double ball_old_time;

void DrawFieldObjects(xwin &w,xdrawable &wb, net_vframe &vf)
{
  vector2f p;
  int cx,cy;
  double dt,v;
  int team,i;
  double conf;
  rgb c;

  cx = DISPLAY_WIDTH_H;
  cy = FIELD_CENTER_Y;

  for(team=0; team<NUM_TEAMS; team++){
    if(team==TEAM_BLUE){
      c.red =   0; c.green =   0; c.blue = 255;
    }else{
      c.red = 255; c.green = 255; c.blue =   0;
    }

    for(i=0; i<MAX_TEAM_ROBOTS; i++){
      conf = vf.robots[team][i].vision.conf;
      dt = vf.timestamp - vf.robots[team][i].vision.timestamp;
      if(conf>0.05 && dt<2){
	p = vf.robots[team][i].vision.pos;
	DrawRobot(w,cx + p.x/10,cy - p.y/10,vf.robots[team][i].vision.angle,i,team);

        if(dt > 0.0) conf = 0.0;
        // printf("%d %d (%f,%f) c=%f\n",team,i,p.x,p.y,conf);
	wb.setColor((int)(c.red*conf),(int)(c.green*conf),(int)(c.blue*conf));
	wb.drawPoint(col,CONF_GRAPH_Y+team*MAX_TEAM_ROBOTS+i+1);
        // wb.drawPoint(col,CONF_GRAPH_Y-2-(int)(32*conf));
      }
    }
  }

  /*
  p = vf.ball.vision.pos;
  printf("%8.2f %8.2f %10.6f\n",p.x,p.y,
         vf.robots[TEAM_BLUE][0].vision.angle);
  */

  p = vf.ball.vision.pos;
  conf = vf.ball.vision.conf;
  c.red   = (int)(255 * conf);
  c.green = (int)(128 * conf);
  c.blue  = (int)(  0 * conf);
  dt = vf.timestamp - vf.ball.vision.timestamp;

  v = 0.0;
  if(conf > 0.45){
    if(dt < 1E-10){
      v = Vector::distance(ball_old,p) /
          (vf.ball.vision.timestamp - ball_old_time);
      // if(v > 1000) printf("v=%8.2f (%8.2f,%8.2f)\n",v,p.x,p.y);

      ball_old = p;
      ball_old_time = vf.ball.vision.timestamp;
    
      DrawBall(w,cx + (int)(p.x/10),cy - (int)(p.y/10));
    }

    if(ball_image){
      setpixel(ball_rgb,p.x,p.y,c);
    }
  }

  if(dt > 0.0) conf = 0.0;
  wb.setColor(c);
  wb.drawPoint(col,CONF_GRAPH_Y);

  v = bound(v,0,6000);
  wb.drawLine(col,CONF_GRAPH_Y-2,
              col,CONF_GRAPH_Y-2-(int)(32*(v/6000.0)));

  for(i=0; i<=6; i++){
    wb.setColor(64*(2-i%2));
    wb.drawPoint(col,CONF_GRAPH_Y-2-(int)(32*(i/6.0)));
  }


  col = (col + 1) % DISPLAY_WIDTH;
  wb.setColor(0);
  wb.fillRectangle(col,CONF_GRAPH_Y,4,NUM_TEAMS*MAX_TEAM_ROBOTS+1);
  wb.fillRectangle(col,CONF_GRAPH_Y-34,4,32);

  /*
  wb.setColor(96);
  wb.drawLine(0,CONF_GRAPH_Y-35,DISPLAY_WIDTH,CONF_GRAPH_Y-35);
  wb.drawLine(0,CONF_GRAPH_Y- 1,DISPLAY_WIDTH,CONF_GRAPH_Y- 1);
  */
}

int main(int argc,char *argv[])
{
  Client client;
  // tparam tp;
  vlocations loc;
  net_vframe vf;
  char hostname[256] = "calvin.prodigy.cs.cmu.edu";

  // GUI stuff
  xwindows windows;
  xwin field;
  xpixmap field_bg;
  // bool draw;

  XEvent xev;
  KeySym key;
  bool run;
  int frame;

  int c;
  bool setrobots = false;
  while ((c = getopt(argc, argv, "rH:")) >= 0) {
    switch (c) {
    case 'r':
      setrobots = true;
      break;
    case 'H':
      strcpy(hostname, optarg);
      break;
    default:
      printf("Bad option %c\n", c);
      printf("USAGE: xvclient [-r | -H <hostname>]\n");
      printf("\t-r\t: enable setting of robots\n");
      printf("\t-H\t: set hostname\n");
      exit(1);
    }
  }

  if(ball_image){
    ball_rgb = new rgb[image_width*image_height];
    clear(ball_rgb,black);
  }

  if(!windows.initialize()){
    printf("XWindows initialization failed!\n");
    exit(1);
  }

  windows.createWindow(field,DISPLAY_WIDTH,DISPLAY_HEIGHT,"Vision Spy");
  field_bg = field.createPixmap(DISPLAY_WIDTH,DISPLAY_HEIGHT);
  DrawField(field_bg);
  field.setBackground(field_bg);

  // Connect to vision server
  printf("using %s.\n",hostname);
  if (!client.Initialize(hostname)) {
    printf("Cannot open sockets to server %s\n", hostname);
    exit(1);
  }

  // test code
  if (setrobots) {
    net_vconfig vc;
    memset(&vc, 0, sizeof(net_vconfig));
    vc.msgtype = NET_VISION_CONFIG;

    for (int i = 0; i < 4; i++) {
      vc.teams[TEAM_BLUE].robots[i].id = i;
      vc.teams[TEAM_BLUE].robots[i].type = ROBOT_TYPE_DIFF;
    }
    vc.teams[TEAM_BLUE].robots[4].id = 5;
    vc.teams[TEAM_BLUE].robots[4].type = ROBOT_TYPE_OMNI;
    vc.teams[TEAM_BLUE].cover_type = VCOVER_NORMAL;

    vc.teams[TEAM_YELLOW].robots[0].id = -1;
    vc.teams[TEAM_YELLOW].robots[1].id = -1;
    vc.teams[TEAM_YELLOW].robots[2].id = -1;
    vc.teams[TEAM_YELLOW].robots[3].id = -1;
    vc.teams[TEAM_YELLOW].robots[4].id = -1;
    vc.teams[TEAM_YELLOW].cover_type = VCOVER_NONE;

    client.Configure(vc);
  }

  // Get vision updates for awhile and print them
  memset(&loc,0,sizeof(loc));

  frame = 0;
  run = true;

  while(run){
    client.GetUpdate(vf);

    // printf("%f\n",loc.timestamp-loc.ball.timestamp);

    frame++;

    RedrawField(field);
    //    DrawFieldObjects(field,field_bg, loc);
    DrawFieldObjects(field,field_bg, vf);

    // Check for user events
    while(windows.checkEvent(xev)){
      switch(xev.type){
        case KeyPress:
          key = XLookupKeysym(&xev.xkey,0);
          switch(key){
	    case(XK_R):
	    case(XK_r):
	      //	      client.reset();
	      break;

            case(XK_S):
            case(XK_s):
              if(ball_image) save("ball.ppm",ball_rgb);
              break;

            case(XK_Escape):
            case(XK_Q):
            case(XK_q):
              run = false;
              break;
          }
          break;

        case ButtonPress:
          printf("You pressed button %d at (%d,%d)\n",
                 xev.xbutton.button,xev.xbutton.x,xev.xbutton.y);
          break;
      }
    }
  }

  printf("Closing.\n");
  client.Close();
  windows.close();

  return(0);
}
/*
 angle standard deviation:
   0.0048250 radians (robot a = 0.0rad) 0.4429
   0.0077313 radians (robot a = 1.6rad) 0.2764
   avg: 0.3597 degrees
*/
