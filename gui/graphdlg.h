/*
 * TITLE:        graphdlg.h
 *
 * PURPOSE:      This fiel wraps the graph dispaly function. It displays the 
 *               graphics window with the array of data.
 *               
 * WRITTEN BY:   Brett Browning
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

#ifndef __GRAPHDLG_H__
#define __GRAPHDLG_H__

#include <stdio.h>
#include <string.h>
#include "../soccer/net_gui.h"
#include <gtk/gtk.h>

//#define DEBUG

// class for showing 1D graphs
class GraphDlg1D {
private:
  bool initialized;
  bool dtype;

  ushort *idata, *isrcdata;
  double *ddata, *dsrcdata;
  uint size;

  GdkPixmap *pixmap;
  GdkGC *gc, *fontgc, *backgc;
  GdkFont *font;

  // draw as a linegraph
  bool drawline;

  bool createWindow(void);
  int fontsize;

  void draw_ushort(void);
  void draw_double(void);

public:
  GtkWidget *wmain, *wdraw;

  GraphDlg1D(void) {
    initialized = false;
    wmain = wdraw = NULL;
    pixmap = NULL;
    idata = isrcdata = NULL;
    ddata = dsrcdata = NULL;
    size = 0;
    gc = NULL;
    drawline = false;
    font = NULL;
  }
  ~GraphDlg1D(void) {
    clear();
  }

  // wet the main window widget pointer 
  bool create(ushort *data, uint _size, bool _drawline = false);
  bool create(double *data, uint _size, bool _drawline = false);

  void clear(void);
  void close();


  // update and draw at the end of the frame
  void update(void);
  void update(ushort *data);
  void update(double *data);

  void draw(void);
  void redraw(GdkEventExpose *event);
  void redraw(void);

  bool realize(GtkWidget *widget, GdkEventConfigure *event);

};


// class for showing 2D graphs
class GraphDlg2D {
private:
  bool initialized;
  bool dtype;

  ushort *idata, *isrcdata;
  double *ddata, *dsrcdata;
  uint xsize, ysize, size;

  GdkPixmap *pixmap;
  GdkGC *gc, *fontgc, *backgc;
  GdkFont *font;

  bool createWindow(void);
  int fontsize;

  void draw_ushort(void);
  void draw_double(void);

public:
  GtkWidget *wmain, *wdraw;

  GraphDlg2D(void) {
    initialized = false;
    wmain = wdraw = NULL;
    pixmap = NULL;
    idata = isrcdata = NULL;
    ddata = dsrcdata = NULL;
    xsize = ysize = size = 0;
    gc = NULL;
    font = NULL;
  }
  ~GraphDlg2D(void) {
    clear();
  }

  // wet the main window widget pointer 
  bool create(ushort *data, uint _xsize, uint _ysize);
  bool create(double *data, uint _xsize, uint _ysize);

  void clear(void);
  void close(void);


  // update and draw at the end of the frame
  void update(void);
  void update(ushort *data);
  void update(double *data);

  void draw(void);
  void redraw(GdkEventExpose *event);
  void redraw(void);

  bool realize(GtkWidget *widget, GdkEventConfigure *event);

};

#endif

