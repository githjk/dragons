/*
 * TITLE:       draw.cc
 *
 * PURPOSE:     This file encapsulates the classes needed to draw informaiton to the screen
 *
 * WRITTEN BY:  James Bruce, Brett Browning, Dinesh Govindaraju
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

#include <gtk/gtk.h>
#include <stdio.h>
#include <fstream.h>

#include "constants.h"
#include "geometry.h"

#include "threshold.h"
#include "draw.h"

// temp for now
#include "main.h"

#define DEBUG

/************************************* TYPES *************************************/
#define HIST_XSIZE         100
#define HIST_YSIZE         100


/************************************* DrawHistogram *****************************/
  
bool DrawHistogram::Initialize(GtkWidget *w, int nc, 
			       CMVision::color_class_state *colors)
{
#ifdef DEBUG
  fprintf(stderr, "Initializing histogram with %i colors\n", nc);
#endif

  num_colors = nc + 1;
  widget = w;
  width = w->allocation.width;
  height = w->allocation.height;

  /* set the scaling ratios */
  yperpix = (double) MAX_YSIZE / (double) width;
  uperpix = (double) MAX_USIZE / (double) height;

  /* set the widget size */
  gtk_widget_set_usize(w, HIST_XSIZE, HIST_YSIZE);

  /* create the color graphics contexts */
  for (int i = 0; i < num_colors; i++) {
    colorgc[i] = gdk_gc_new(w->window);

    guint32 col;
    col = (guint32) colors[i].color.blue;
    col |= ((guint32) colors[i].color.green) << 8;
    col |= ((guint32) colors[i].color.red) << 16;
    gdk_rgb_gc_set_foreground(colorgc[i], col);
  }

  backgc = gdk_gc_new(widget->window);
  gdk_rgb_gc_set_foreground(backgc, (guint32) 0);

  whitegc = gdk_gc_new(widget->window);
  gdk_rgb_gc_set_foreground(whitegc, (guint32) 0xFFFFFF);

  /* craete the colormap colors */
  colormap.init(MAX_YSIZE, MAX_USIZE, MAX_VSIZE);
  for (int y = 0; y < MAX_YSIZE; y++) {
    for (int u = 0; u < MAX_USIZE; u++) {
      for (int v = 0; v < MAX_VSIZE; v++) {
	rgb rgbcol = YuvToRgb((yuv) {y, u, v});
	GdkGC *gc = gdk_gc_new(w->window);

	guint32 col;
	col = (guint32) rgbcol.blue;
	col |= ((guint32) rgbcol.green) << 8;
	col |= ((guint32) rgbcol.red) << 16;

	gdk_rgb_gc_set_foreground(gc, col);
	colormap.set(y, u, v, gc);
      }
    }
  }

  /* create the field pixmap to draw on to */
  pixmap = gdk_pixmap_new(w->window, width, height, -1);

  // blank it all out at the start
  gdk_draw_rectangle(pixmap, backgc, true, 0, 0, width, height);

  /* redraw it for the first time */
  Redraw();

  /* all done */
  initialized = true;
  return (true);
}

void DrawHistogram::Reconfigure(void)
{
  if (!initialized)
    return;

  /* delete old pixmap */
  if (pixmap) {
    gdk_pixmap_unref(pixmap);
    pixmap = NULL;
  }

  // reset scaling parameters
  width = widget->allocation.width;
  height = widget->allocation.height;
  yperpix = (double) MAX_YSIZE / (double) width;
  uperpix = (double) MAX_USIZE / (double) height;

  /* create the field pixmap to draw on to */
  pixmap = gdk_pixmap_new(widget->window, width, height, -1);

  // redraw it now!!!
  Draw();
  Redraw();
}

void DrawHistogram::Redraw(void)
{
  Draw();
  /* draw the pixmap first time up */
  gdk_draw_pixmap(widget->window,  widget->style->fg_gc[GTK_WIDGET_STATE(widget)],
		  pixmap, 0, 0, 0, 0, width, height);
}


void DrawHistogram::Draw(void)
{
  int x,y;
  int sx,sy,sz;
  int c;
  GdkPoint p;

  sx = tmap.getSizeX();
  sy = tmap.getSizeY();
  sz = tmap.getSizeZ();

  // erase background 
  gdk_draw_rectangle(pixmap, backgc, true, 0, 0, width, height);

  //  int w = floor((double) width / (double) sx + 0.5);
  //  int h = floor((double) height / (double) sy + 0.5);
  int w = (int) floor(1.0 / yperpix + 0.99);
  int h = (int) floor(1.0 / uperpix + 0.99);

  for(y = 0; y < sy; y++){
    for(x = 0; x < sx; x++){
      c = tmap.get(x, y, display_level);
      p.x = yindx2pix(x);
      p.y = uindx2pix(y);

      if (c != 255)
	gdk_draw_rectangle(pixmap, colorgc[c], 1, p.x, p.y, w, h);
      else
	gdk_draw_rectangle(pixmap, backgc, 1, p.x, p.y, w, h);

      // draw the histogram data
      int hval  = histogram.get(x, y, display_level);
      int hw = MIN(3 * w / 4, hval);
      int hh = MIN(3 * h / 4, hval);

      // this is not working???
      //      GdkGC *gc = colormap.get(x, y, display_level);
      gdk_draw_rectangle(pixmap, whitegc, 1, p.x, p.y, hw, hh);
    }
  }
}


// operators on the map itself
void DrawHistogram::ResetMap(void)
{
  tmap.init(MAX_YSIZE, MAX_USIZE, MAX_VSIZE);
  tmap.set(0);
  histogram.init(MAX_YSIZE, MAX_USIZE, MAX_VSIZE); 
  histogram.set(0);

}

// file control
bool DrawHistogram::Load(char *filename)
{
  FILE *in;
  char buf[256];
  int nx,ny,nz;
  int size,read;

  in = fopen(filename,"r");
  if(!in) return(false);

  // read magic
  if(!fgets(buf,256,in)) goto error;
  buf[4] = 0;
  if(strcmp(buf,"TMAP")) goto error;

  // read type
  if(!fgets(buf,256,in)) goto error;
  // ignore for now

  // read size
  if(!fgets(buf,256,in)) goto error;
  nx = ny = nz = 0;
  sscanf(buf,"%d %d %d",&nz,&ny,&nx);
  size = nx * ny * nz;
  if(size == 0) goto error;

  tmap.init(nx,ny,nz);
  read = fread(tmap.getData(),sizeof(char),size,in);

  fclose(in);
  return(read == size);
error:
  if(in) fclose(in);
  return(false);
}

bool DrawHistogram::Save(char *filename)
{
  FILE *out;
  int size,wrote;

  out = fopen(filename,"w");
  if(!out) return(false);

  fprintf(out,"TMAP\nYUV8\n%d %d %d\n",
	  tmap.getSizeZ(),tmap.getSizeY(),tmap.getSizeX());
  size = tmap.getSize();
  wrote = fwrite(tmap.getData(),sizeof(char),size,out);
  fclose(out);

  return(wrote == size);
}

// drawing in screen coordingates
void DrawHistogram::DrawLine(GdkPoint p1, GdkPoint p2)
{
  yui yp1 = pix2indx(p1);
  yui yp2 = pix2indx(p2);

  tmap.setb(yp1.y, yp1.u, display_level, curr_color);
  tmap.setb(yp2.y, yp2.u, display_level, curr_color);

  if (yp2.u == yp1.u) {
    for (int i = yp1.y; i <= yp2.y; i++)
      tmap.setb(i, yp1.u, display_level, curr_color);
  } else {
    double m = (double) (yp2.u - yp1.u) / (double) (yp2.y - yp1.y);
    if (ABS(m) < 1.0) {
      double u = (double) yp1.u;
      for (int y = yp1.y; y <= yp2.y; y++) {
	tmap.setb(y, floor(u + 0.5), display_level, curr_color);
	u += m;
      }

    } else {
      double y = (double) yp1.y;
      m = 1.0 / m;
      for (int u = yp1.u; u <= yp2.u; u++) {
	tmap.setb(floor(y + 0.5), u, display_level, curr_color);
	y += m;
      }

    }
  }
}


void DrawHistogram::DrawPoint(GdkPoint p)
{
  yui yp = pix2indx(p);
  tmap.setb(yp.y, yp.u, display_level, curr_color);
}
  
  
// region filling, in array coords
int DrawHistogram::Paint(int x,int y, int src_color)
{
  int x1,x2,i;
  int num;

  // find current line extents
  x1 = x2 = x;
  while (x1>0 && tmap.get(x1,y,display_level) == src_color) 
    x1--;
  while(x2<tmap.getSizeX() && tmap.get(x2,y,display_level)==src_color) 
    x2++;
  x1 += (tmap.get(x1,y,display_level) != src_color);

  // paint it
  for (i=x1; i<x2; i++) 
    tmap.set(i,y, display_level, curr_color);
  num = x2 - x1;

  // recurse above
  if(y>0){
    for(i=x1; i<x2; i++){
      if(tmap.get(i,y-1, display_level) == src_color){
	num += Paint(i,y-1, src_color);
      }
    }
  }

  // recurse below
  if(y+1<tmap.getSizeY()){
    for(i=x1; i<x2; i++){
      if(tmap.get(i,y+1, display_level) == src_color){
	num += Paint(i,y+1, src_color);
      }
    }
  }

  return(num);
}

int DrawHistogram::Paint(int x,int y)
{
  int src_color;

  src_color = tmap.getb(x,y,display_level);

  if(x>=0 && x<tmap.getSizeX() &&
     y>=0 && y<tmap.getSizeY() &&
     display_level>=0 && display_level <tmap.getSizeZ() &&
     src_color != curr_color){
    return(Paint(x,y, src_color));
  }else
    return(0);
}

// region filling, in screen coordinates
int DrawHistogram::DrawPaint(int x,int y, int src_color)
{
  int yv = xpix2y(x);
  int uv = ypix2u(y);
  return (Paint(yv, uv, src_color));
}

int DrawHistogram::DrawPaint(int x,int y)
{
  int yv = xpix2y(x);
  int uv = ypix2u(y);
  return (Paint(yv, uv));
}

void DrawHistogram::SetColor(GdkPoint p)
{
  yui yu = pix2indx(p);

  curr_color = tmap.get(yu.y, yu.u, display_level);
}


/************************************* DrawRGB *****************************/

void DrawRGB::SetSize(int w, int h) 
{

  fprintf(stderr, "setsize...\n");

  if (data != NULL)
    delete data;

  fprintf(stderr, "set w %i h %i...\n", w, h);

  width = w; 
  height = h;
  size = w * h * 3;
  data = new char[size];

  fprintf(stderr, "alloc %i...\n", size);
}

void DrawRGB::Draw(char *imgdata)
{

  memcpy(data, imgdata, size);
  ReDraw();
}

void DrawRGB::ReDraw(void)
{
  gdk_draw_rgb_image (widget->window, widget->style->fg_gc[GTK_STATE_NORMAL],
		      0, 0, width, height, GDK_RGB_DITHER_MAX, (guchar *) data, width * 3);
}


/************************************* CParamDraw *****************************/



// initialize the pixmaps and store of the widget
bool CParamDraw::Initialize(GtkWidget *w, char *filename = NULL){

  widget = w;
  int width = w->allocation.width;
  int height = w->allocation.height;

  pixmap = gdk_pixmap_new(w->window, width, height, -1);

  whitegc = gdk_gc_new(w->window);
  gdk_rgb_gc_set_foreground(whitegc, (guint32) 0xFFFFFF);

  descrip[0] = "Field +y 1";
  descrip[1] = "Field +y 2";
  descrip[2] = "Field +y 3";
  descrip[3] = "Penalty +y 1";
  descrip[4] = "Penalty +y 2";
  descrip[5] = "Penalty +y 3";
  descrip[6] = "Penalty +y 4";
  descrip[7] = "Goals +y 1";
  descrip[8] = "Goals +y 2";
  descrip[9] = "Goals +y 3";
  descrip[10] = "Goals +y 4";
  descrip[11] = "Goals +y 5";
  descrip[12] = "Goals +y 6";
  descrip[13] = "Cent Cir 1";
  descrip[14] = "Cent Cir 2";
  descrip[15] = "Goals -y 1";
  descrip[16] = "Goals -y 2";
  descrip[17] = "Goals -y 3";
  descrip[18] = "Goals -y 4";
  descrip[19] = "Goals -y 5";
  descrip[20] = "Goals -y 6";
  descrip[21] = "Penalty -y 1";
  descrip[22] = "Penalty -y 2";
  descrip[23] = "Penalty -y 3";
  descrip[24] = "Penalty -y 4";
  descrip[25] = "Field -y 1";
  descrip[26] = "Field -y 2";
  descrip[27] = "Field -y 3";
  return true;
}


// load the parameter file
bool CParamDraw::LoadFile(char* filename)
{
  FILE *in;
  char buf[256];
  vector3d world;
  vector2d screen;
  int counter;

  in = fopen(filename,"r");
  if(!in) return(false);

  counter = 0;
  while(fgets(buf,256,in)){
    if ((buf[0]!='#') && (buf[0]!='\n')){
      sscanf(buf,"(%lf,%lf,%lf) (%lf,%lf)\n",
	     &world.x,&world.y,&world.z,
	     &screen.x,&screen.y);
      printf("(%f, %f, %f) (%f, %f)\n",
	     world.x,world.y,world.z,
	     screen.x,screen.y);

      yuv[counter] = world;

      // divide y value by 2 since it was interlaced
      // Brett or Jim fix this up one day!!!
      coord[counter].set(screen.x,screen.y/2);

      counter++;
    }
  }

  printf("counter = %d\n",counter);

  return (true);
}


//Save parameter file
bool CParamDraw::SaveFile(char *fname)
{
  ofstream efile;
  efile.open (fname, ios::out | ios::trunc);
  if (efile.is_open())
    {
      efile << "# Field +y" << endl;
      
      for(int i =0; i < 3; i++){
	
	efile << "( " << yuv[i].x << ",  " << yuv[i].y << ",    " << yuv[i].z;
	efile <<")  (  "<< coord[i].x <<",  "<< coord[i].y * 2.0 << ")" << endl;
      }
      
      efile << endl << "# Penalty +y" << endl;
      
      for(int i =3; i < 7; i++){
	
	efile << "( " << yuv[i].x << ",  " << yuv[i].y << ",    " << yuv[i].z;
	efile <<")  (  "<< coord[i].x <<",  "<< coord[i].y * 2.0 << ")"  <<endl;
      }
      
      efile << endl << "# Goals +y" << endl;
      for(int i =7; i < 13; i++){
	
	efile << "( " << yuv[i].x << ",  " << yuv[i].y << ",    " << yuv[i].z;
	efile <<")  (  "<< coord[i].x <<",  "<< coord[i].y * 2.0 << ")" <<endl;
      }
      
      efile << endl << "# Center circle +y/-y" << endl;
      for(int i =13; i < 15; i++){
	
	efile << "( " << yuv[i].x << ",  " << yuv[i].y << ",    " << yuv[i].z;
	efile <<")  (  "<< coord[i].x <<",  "<< coord[i].y * 2.0 << ")" <<endl;
      }
      
      efile << endl << "# Goals -y" << endl;
      for(int i =15; i < 21; i++){
	
	efile << "( " << yuv[i].x << ",  " << yuv[i].y << ",    " << yuv[i].z;
	efile <<")  (  "<< coord[i].x <<",  "<< coord[i].y * 2.0 << ")" <<endl;
      }
      
      efile << endl << "# Penalty -y" << endl;
      for(int i =21; i < 25; i++){
	
	efile << "( " << yuv[i].x << ",  " << yuv[i].y << ",    " << yuv[i].z;
	efile <<")  (  "<< coord[i].x <<",  "<< coord[i].y * 2.0 << ")" <<endl;
      }
      
      efile << endl << "# Field -y" << endl;
      for(int i =25; i < nr_points; i++){
	
	efile << "( " << yuv[i].x << ",  " << yuv[i].y << ",    " << yuv[i].z;
	efile <<")  (  "<< coord[i].x <<",  "<< coord[i].y * 2.0 << ")" <<endl;
      }
      efile.close();
    }
  // remember to double y values from interlacing

  return (true);
}



  // draw it
void CParamDraw::Draw(void)
{
  for(int i = 0;i < nr_points; i++){
    if(coord[i].x != 0 && coord[i].y != 0){
      //gdk_draw_line(pixmap,whitegc,(int)coord[i][0]- 4,(int)coord[i][1],(int)coord[i][0]+ 4,(int)coord[i][1]);
      //      gdk_draw_line(pixmap,whitegc,(int)coord[i][0],(int)coord[i][1] - 4 ,(int)coord[i][0],(int)coord[i][1] + 4);
    }
  }
  ReDraw();
}


  //redraw it
void CParamDraw::ReDraw(void)
{
  gdk_draw_pixmap(widget->window, widget->style->fg_gc[GTK_WIDGET_STATE(widget)],
  		  pixmap, 0, 0, 0, 0, widget->allocation.width, widget->allocation.height);
  
}

 
 // GetClosestPoint
bool CParamDraw::GetClosestPoint(int x, int y)
{
  if (!isClicked){
    for(int i = 0;i < nr_points; i++){
      vector2d p(x, y);
      //      if(((x-coord[i].x)*(x-coord[i].x)) + ((y-coord[i].y)*(y-coord[i].y)) < (range*range)){
      if ((p - coord[i]).sqlength() < 4.0 * 4.0) {
	isClicked = true;
	clickedI = i;

	coord[i] = p;

	//	coord[i].x = -10.0;
	//	coord[i].y = -10.0;
      }
    }
  } else
    coord[clickedI].set(x, y);

  return true;
}


bool CParamDraw::ClicknDragPoint(int x, int y)
{
  return false;
}


  // calibrate it -- leave for now
bool CParamDraw::StartCalibrate(void)
{
  return false;
}



bool CParamDraw::Draw(char *rgbdata)
{
  for(int i = 0;i < nr_points; i++){
    if ((coord[i].x >= 0) && (coord[i].y >= 0)) {
      DrawCrossHere(coord[i],rgbdata);
      // printf("cross %d drawn\n", i );
    }
  }
  // printf("23 : %d, %d\n", (int)coord[23].x, (int)coord[23].y);

  return true;
}


bool CParamDraw::DrawCrossHere(vector2d v, char *rgbdata)
{
  //draw a cross at this vector2d
  int lineWidth = 10;

  for(int d = ( -(lineWidth / 2)) ; d <= (lineWidth / 2) ; d++){
    DrawPointHere(vector2d(v.x + d, v.y), rgbdata);
    DrawPointHere(vector2d(v.x, v.y + d), rgbdata);
  }
  return true;
}


bool CParamDraw::DrawPointHere(vector2d v, char *rgbdata)
{
  // draw a red point at this vector2d
  rgbdata[((((int)v.y - 1) * width + (int)v.x) * 3) + 0] = 0xFF;
  rgbdata[((((int)v.y - 1) * width + (int)v.x) * 3) + 1] = 0; //0xA0;
  rgbdata[((((int)v.y - 1) * width + (int)v.x) * 3) + 2] = 0; //0xA0;
  return true;
}


