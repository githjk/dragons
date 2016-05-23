/*
 * TITLE:        draw.h
 *
 * PURPOSE:      This is file contains the classes for drawing all of hte 
 *               field objects for the simulator.
 *
 * WRITTEN BY:   Michael Bowling, Brett Browning
 * code ported from small size 2001 GUI
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
#include <gdk-pixbuf/gdk-pixbuf.h>

#include "geometry.h"
#include "../utils/polygon.h"

#include "../reality/net_vision.h"
#include "../include/rtypes.h"

/**************************** TYPES ******************************************/

/***************************** CLASSES ***************************************/

class DrawField {
public:
  GtkWidget *darea;
  GdkPixmap *pixmap;
  GdkPixbuf *pixbuf, *pixbuf_scaled;
  GtkWidget *widget;
  GdkGC     *fieldgc;

  bool initialized;
  int width, height;
  double xcpix, ycpix;
  int pixwidth, pixheight;
  double xmmperpix, ymmperpix;
  double pixscalex, pixscaley;
public:
  DrawField(void) {
    initialized = false;
  }
  ~DrawField(void);

  bool Initialize(GtkWidget *w);
  bool IsInitialized(void) {
    return (initialized);
  }
  void Reconfigure(void);

  void Redraw(void);
  void Redraw(GdkEventExpose *event);

  // conversion functions for vectors
  inline GdkPoint mm2pix(vector2d p) {
    return ((GdkPoint) {int_xmm2pix(p.x), int_ymm2pix(p.y)});
  }
  inline vector2d pix2mm(GdkPoint p) {
    return (vector2d(xpix2mm(p.x), ypix2mm(p.y)));
  }

  inline GdkPoint dist_mm2pix(vector2d p) {
    return ((GdkPoint) {idist_xmm2pix(p.x), idist_ymm2pix(p.y)});
  }
  inline vector2d dist_pix2mm(GdkPoint p) {
    return (vector2d(xdist_pix2mm(p.x), ydist_pix2mm(p.y)));
  }

  // component wise conversion functions
  inline double xdist_mm2pix(double xdist) {
    return (xdist / xmmperpix);
  }
  inline double ydist_mm2pix(double ydist) {
    return (ydist / ymmperpix);
  }
  inline double xdist_pix2mm(double xpixdist) {
    return (xpixdist * xmmperpix);
  }
  inline double ydist_pix2mm(double ypixdist) {
    return (ypixdist * ymmperpix);
  }

  inline double xmm2pix(double x) {
    return (xcpix + x / xmmperpix);
  }
  inline double ymm2pix(double y) {
    return (ycpix - y / ymmperpix);
  }
  inline int idist_xmm2pix(double x) {
    return ((int) floor(x / xmmperpix + 0.5));
  }
  inline int idist_ymm2pix(double y) {
    return ((int) floor(y / ymmperpix + 0.5));
  }
  inline int int_xmm2pix(double x) {
    return ((int) floor(xcpix + x / xmmperpix + 0.5));
  }
  inline int int_ymm2pix(double y) {
    return ((int) floor(ycpix - (y / ymmperpix + 0.5)));
  }
  inline double xpix2mm(double x) {
    return ((x - xcpix) * xmmperpix);
  }
  inline double ypix2mm(double y) {
    return ((ycpix - y) * ymmperpix);
  }
  inline double xpix2mm(int x) {
    return (((double) x + xcpix) * xmmperpix);
  }
  inline double ypix2mm(int y) {
    return ((ycpix - (double) y) * ymmperpix);
  }
};

class DrawBall {
private:
  DrawField *field;
  GdkGC *ballgc, *outlinegc;

  bool isselected;
  vector2d pos, vel;
  GdkPoint spos, svel;
  bool initialized;

  GtkWidget *widget;
  int width, height;
public:
  DrawBall(void) {
    initialized = false;
  };
  ~DrawBall(void);

  void Initialize(DrawField *f, GtkWidget *w);

  void Reconfigure(void);

  void Draw(void);
  void Erase(void);

  void SetSelected(bool sel) {
    isselected = sel;
  }
  void SetPosition(vector2d p);
  void SetVelocity(vector2d v);

  void SetVisionInfo(net_vlocation *vb);
};

class DrawRobot {
private:
  GtkWidget *widget;
  DrawField *field;
  GdkGC *yellow_gc, *blue_gc;
  GdkGC *outline_gc, *selected_gc, *id_gc;
  GdkFont *font;
  
  Polygon polygon;
  Polygon scr_polygon;
  int width, height;

  int rtype;
  bool isselected;
  bool isyellow;

  double radius;

  RPosition pos;
  RVelocity vel;
  GdkPoint spos, svel;

  bool initialized;

  bool enabled;

public:
  DrawRobot(void);
  ~DrawRobot(void);

  void Initialize(DrawField *f, GtkWidget *w);
  void Reconfigure(void);

  void Draw(void);
  void Erase(void);

  void SetTeamColor(bool yellow) {
    isyellow = yellow;
  }
  void SetType(int t);

  void SetSelected(bool sel) {
    isselected = true;
  }

  void SetShow(bool shw) {
    enabled = shw;
  }
  void SetPosition(vector2d p, double a = 0.0);
  void SetDirection(double a);
  void SetVelocity(vector2d v, double va = 0.0);

  void SetVisionInfo(net_vlocation *vr);
};


#endif /* __DRAW_H__ */
