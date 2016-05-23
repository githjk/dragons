/*
 * TITLE:       draw.h
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

#ifndef __DRAW_H__
#define __DRAW_H__

#include <gtk/gtk.h>
#include "constants.h"
#include "geometry.h"
//#include "threshold.h"
#include "reality/cmvision/cmvision.h"

#include "array3d.h"


typedef array3d<unsigned char,0> tmap_t;
typedef array3d_def<GdkGC *> colormap_t;


/************************************* TYPES *************************************/
struct yui {
  int y, u;
};

class DrawHistogram {
protected:
  GdkGC *colorgc[MAX_COLORS], *backgc, *whitegc;
  int num_colors;
  GtkWidget *widget;
  GdkPixmap *pixmap;
  bool initialized;
  int width, height;

  double yperpix, uperpix;

  int display_level;
  int curr_color;
  
public:
  tmap_t tmap;
  tmap_t histogram;
  colormap_t colormap;

private:
  // region filling, in screen coordinates
  int Paint(int x,int y, int src_color);
  int Paint(int x,int y);

public:
  DrawHistogram(void) {
    initialized = false;
    pixmap = NULL;
    yperpix = uperpix = 0.0;
    num_colors = curr_color = 0;
    display_level = 0;
    ResetMap();
  }
  ~DrawHistogram(void) {
  }

  // initialization
  bool Initialize(GtkWidget *w,int nc, 
		  CMVision::color_class_state *colors);
  bool IsInitialized(void) {
    return (initialized);
  }

  // methods to actually draw the information
  void Draw(void);

  // configuration and redraw
  void Reconfigure(void);
  void Redraw(void);
  void Redraw(GdkEventExpose *event) {
    gdk_draw_pixmap(widget->window, widget->style->fg_gc[GTK_WIDGET_STATE(widget)],
		    pixmap, event->area.x, event->area.y,
		    event->area.x, event->area.y, event->area.width, event->area.height);
  }

  // operators on the map itself
  void ResetMap(void);

  // file control
  bool Load(char *filename);
  bool Save(char *filename);

  // drawing in screen coordingates
  void DrawLine(GdkPoint p1, GdkPoint p2);
  void DrawLine(int x1,int y1, int x2, int y2);
  void DrawPoint(GdkPoint p);
  void DrawPoint(int x, int y) {
    DrawPoint((GdkPoint) {x, y});
  }
  
  // region filling, in screen coordinates
  int DrawPaint(int x,int y, int src_color);
  int DrawPaint(int x,int y);
  int DrawPaint(GdkPoint p, int src_color) {
    return (DrawPaint(p.x, p.y, src_color));
  }
  int DrawPaint(GdkPoint p) {
    return (DrawPaint(p.x, p.y));
  }

  // z level control
  void SetDisplayLevel(int l) {
    display_level = l;
  }
  int GetDisplayLevel(void) {
    return (display_level);
  }
  void IncrementDisplayLevel(void) {
    display_level = MIN(display_level + 1, MAX_VSIZE - 1);
  }
  void DecrementDisplayLevel(void) {
    display_level = MAX(display_level - 1, 0);
  }

  // color control -- all coords in screen dimensions
  void SetColor(int c) {
    curr_color = c;
  }
  int GetColor(void) {
    return (curr_color);
  }
  void IncrementColor(void) {
    curr_color = MIN(curr_color + 1, num_colors - 1);
  }
  void DecrementColor(void) {
    curr_color = MAX(curr_color - 1, 0);
  }
  void SetColor(GdkPoint p);
  void SetColor(int xs, int ys) {
    SetColor((GdkPoint) {xs, ys});
  }
     
  // unit conversion functions from here out

  // convert from (y, u) -> (x, y) coords
  inline GdkPoint index2pix(yui p) {
    return (index2pix(p.y, p.u));
  }
  inline GdkPoint index2pix(int y, int u) {
    return ((GdkPoint) {yindx2pix(y), uindx2pix(u)});
  }

  // convert from (x, y) space -> (y, u) coords
  inline yui pix2indx(GdkPoint p) {
    return (pix2indx(p.x, p.y));
  }
  inline yui pix2indx(int x, int y) {
    return ((yui) {xpix2y(x), ypix2u(y)});
  }

  // raw conversions
  inline int yindx2pix(int y) {
    return ((int) floor((double) y / yperpix + 0.5));
  }
  inline int uindx2pix(int u) {
    return ((int) floor((double) u / uperpix + 0.5));
  }
  inline int xpix2y(int x) {
    return ((int) floor((double) x * yperpix + 0.5));
  }
  inline int ypix2u(int y) {
    return ((int) floor((double) y * uperpix + 0.5));
  }
};

class DrawRGB {
protected:
  GtkWidget *widget;
  int width, height;

  char *data;
  int size;

public:
  DrawRGB(void) {
    //    gdk_rgb_init();
    width = height = 0;
    data = NULL;
  }
  DrawRGB(int w, int h) {
    //    gdk_rgb_init();
    data = NULL;
    SetSize(w, h);
  }
  ~DrawRGB(void) {
    if (data)
      delete data;
  }

  // set the drawing area
  void SetWidget(GtkWidget *w) {
    widget = w;
    gtk_widget_set_usize(w, width, height);
  }
  void SetSize(int w, int h);
  int GetWidth(void) {
    return (width);
  }
  int GetHeight(void) {
    return (height);
  }

  // Draw the widget
  void Draw(char *imgdata);
  void ReDraw(void);
};


class CParamDraw {

protected:

  static const int nr_points = 28;

  vector2d coord[nr_points];
  vector3d yuv[nr_points];
  GtkWidget *widget;
  GdkPixmap *pixmap;
  GdkGC *whitegc;
  int width, height;
  
public:

  // constructor -- allocate something?
  bool isClicked;
  bool isShowing;
  char* descrip[nr_points];
  int clickedI;

  CParamDraw(void) {
    isClicked = false;
    isShowing = false;
  }

  // go and read in teh parameter file 
  CParamDraw(char *filename) {
  }

  // destructor -- delete what we allocated
  ~CParamDraw(void) {
  }

  // draw to RGB data
  bool SetSize(int w, int h) {
    width = w;
    height = h;
    return true;
  }
 
  //Draw to this raw data
  bool Draw(char *rgbdata);
  
  // initialize the pixmaps and store of the widget
  bool Initialize(GtkWidget *w, char *filename = NULL);

  // load the parameter file
  bool LoadFile(char *fname);
  bool SaveFile(char *fname);

  // draw it
  void Draw(void);

  // do fast redraw options
  void ReDraw(void);

    // draw to this x,y coordinate
  bool DrawCrossHere(vector2d v, char *rgbdata);


  // draw to this x,y coordinate
  bool DrawPointHere(vector2d v, char *rgbdata);


  // GetClosestPoint
  bool GetClosestPoint(int x, int y);
  bool ClicknDragPoint(int x, int y);

  // calibrate it -- leave for now
  bool StartCalibrate(void);
  
};

#endif /* __DRAW_H__ */

