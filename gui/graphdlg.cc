/*
 * TITLE:        graphdlg.cc
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

#include <stdio.h>
#include <string.h>
#include "../soccer/net_gui.h"
#include <deque>
#include <gtk/gtk.h>

#include "graphdlg.h"

#include "interface.h"
#include "support.h"

//#define DEBUG

/*********************** PROTOTYPES ******************************/
static gboolean GraphDlg1DExpose(GtkWidget *widget, GdkEventExpose  *event,
		gpointer user_data);
static void GraphDlg1DDestroy(GtkObject *object, gpointer user_data);
static gboolean GraphDlg1DConfigure(GtkWidget *widget, GdkEventConfigure *event,
				  gpointer user_data);
static void GraphDlg1DUpdateClicked(GtkButton *button,
				  gpointer user_data);
static void GraphDlg1DCloseClicked(GtkButton *button,
				 gpointer user_data);
static gboolean GraphDlg2DExpose(GtkWidget *widget, GdkEventExpose  *event,
		gpointer user_data);
static void GraphDlg2DDestroy(GtkObject *object, gpointer user_data);
static gboolean GraphDlg2DConfigure(GtkWidget *widget, GdkEventConfigure *event,
				  gpointer user_data);
static void GraphDlg2DUpdateClicked(GtkButton *button,
				  gpointer user_data);
static void GraphDlg2DCloseClicked(GtkButton *button,
				 gpointer user_data);

/*********************** GRAPHDLG1D ************************************/


bool GraphDlg1D::createWindow(void)
{
  // create the windows
  wmain = create_graph_display();
  if (wmain == NULL)
    return (false);

  wdraw = lookup_widget(wmain, "graph_drawarea");
  if (wdraw == NULL) {
    gtk_widget_destroy(wmain);
    return (false);
  }

  // connect the signals
  gtk_signal_connect(GTK_OBJECT(wdraw), "expose_event", GTK_SIGNAL_FUNC(GraphDlg1DExpose),
		     (gpointer) this);
  gtk_signal_connect(GTK_OBJECT(wmain), "destroy", GTK_SIGNAL_FUNC(GraphDlg1DDestroy),
		     (gpointer) this);
  gtk_signal_connect(GTK_OBJECT(wdraw), "configure_event", GTK_SIGNAL_FUNC(GraphDlg1DConfigure),
		     (gpointer) this);

  GtkWidget *w = lookup_widget(wmain, "GraphUpdate");
  gtk_signal_connect(GTK_OBJECT(w), "clicked", GTK_SIGNAL_FUNC(GraphDlg1DUpdateClicked),
		     (gpointer) this);
  w = lookup_widget(wmain, "GraphClose");
  gtk_signal_connect(GTK_OBJECT(w), "clicked", GTK_SIGNAL_FUNC(GraphDlg1DCloseClicked),
		     (gpointer) this);

  gtk_widget_show(wmain);
  return (true);
}

// wet the main window widget pointer 
bool GraphDlg1D::create(ushort *data, uint _size, bool _drawline)
{
  if (initialized) {
    fprintf(stderr, "WARNING: Calling GraphDialog::create multiple times\n");
    return (false);
  }

  drawline = _drawline;
  size = _size;
  isrcdata = data;
  idata = new ushort[size];
  memcpy(idata, isrcdata, size * sizeof(ushort));

  if (!createWindow()) {
    fprintf(stderr, "ERROR: Cannot create GraphDialog window\n");
    return (false);
  }

  initialized = true;
  draw_ushort();

  return (true);
}

bool GraphDlg1D::create(double *data, uint _size, bool _drawline)
{
  if (initialized) {
    fprintf(stderr, "WARNING: Calling GraphDialog::create multiple times\n");
    return (false);
  }

  if (!createWindow()) {
    fprintf(stderr, "ERROR: Cannot create GraphDialog window\n");
    return (false);
  }

  drawline = _drawline;
  size = _size;
  ddata = new double[size];
  dsrcdata = data;
  memcpy(ddata, dsrcdata, size * sizeof(double));
  draw_double();
  initialized = true;

  return (true);
}

// update and draw at the end of the frame
void GraphDlg1D::update(void)
{
  if (!initialized)
    return;

  fprintf(stderr, "Updateing graphdlg\n");

  if (dtype) {
    memcpy(ddata, dsrcdata, size * sizeof(double));
    draw_double();
  } else {
    memcpy(idata, isrcdata, size * sizeof(ushort));
    draw_ushort();
  }
}

void GraphDlg1D::update(ushort *data)
{
  if (!initialized || dtype)
    return;
  memcpy(idata, isrcdata, size * sizeof(ushort));
  draw_ushort();
}
  
void GraphDlg1D::update(double *data)
{
  if (!initialized || !dtype)
    return;
  memcpy(ddata, dsrcdata, size * sizeof(double));
  draw_double();
}

void GraphDlg1D::draw(void)
{
  if (!initialized)
    return;

  if (dtype)
    draw_double();
  else
    draw_ushort();
}

void GraphDlg1D::redraw(GdkEventExpose *event)
{
  if (!initialized)
    return;
  gdk_draw_pixmap(wdraw->window,
		  wdraw->style->fg_gc[GTK_WIDGET_STATE(wdraw)],
		  pixmap,
                  event->area.x, event->area.y,
                  event->area.x, event->area.y,
                  event->area.width, event->area.height);		  
}

void GraphDlg1D::redraw(void)
{
  if (!initialized)
    return;
  gdk_draw_pixmap(wdraw->window,
  		  wdraw->style->fg_gc[GTK_WIDGET_STATE(wdraw)],
  		  pixmap, 0, 0,
		  wdraw->allocation.width, wdraw->allocation.height,
		  wdraw->allocation.width, wdraw->allocation.height);
}

void GraphDlg1D::draw_ushort(void)
{
  if (!initialized) {
    fprintf(stderr, "GraphDLG: Draw_ushort not initialized\n");
    return;
  }

  int w = wdraw->allocation.width, h = wdraw->allocation.height;
  double scale, dx;
  ushort maxv = 0;

  // work out the scale
  for (ushort *ptr = idata; ptr < idata + size; ptr++) {
    if (*ptr > maxv)
      maxv = *ptr;
  }
  scale = (double) h / (double) maxv;
  dx = (double) w / (double) size;

  // draw the background
  gdk_draw_rectangle(pixmap, backgc, true, 0, 0, w, h);

  if (drawline) {
    vector2d p(0, 0);
    int oldx = 0, oldy = 0;
    double hd = (double) h;

    for (ushort *ptr = idata; ptr < idata + size; ptr++) {
      p.y = hd - (double) (*ptr) * scale;
      if (ptr != idata) {
	gdk_draw_line(pixmap, gc, oldx, oldy, (int) p.x, (int) p.y);
      }
      oldx = (int) p.x;
      oldy = (int) p.y;
      p.x += dx;
    }
  } else {
    int bwidth = max((int) ceil(dx), 1);
    int y = 0;
    double x = 0;
    for (ushort *ptr = idata; ptr < idata + size; ptr++) {
      y = (int) ((double) (*ptr) * scale);
      gdk_draw_rectangle(pixmap, gc, true, (int) x, h - y, bwidth, y);
      x += dx;
    }
  }

  // label some things
  static char str[20];
  sprintf(str, "%d", size);
  gdk_draw_text(pixmap, font, fontgc, w - fontsize * strlen(str), h, str, strlen(str));
  sprintf(str, "%d", maxv);
  gdk_draw_text(pixmap, font, fontgc, 0, fontsize, str, strlen(str));

}

void GraphDlg1D::draw_double(void)
{
}


void GraphDlg1D::clear(void)
{
  if (!initialized) 
    return;

  fprintf(stderr, "Clearing...\n");

  if (pixmap) {
    gdk_pixmap_unref(pixmap);
    pixmap = NULL;
  }
  if (font) {
    gdk_font_unref(font);
    font = NULL;
  }

  if (dtype) {
    delete ddata;
  } else {
    delete idata;
  }

  gtk_widget_destroy(wmain);
  initialized = false;
}

bool GraphDlg1D::realize(GtkWidget *widget, GdkEventConfigure *event)
{
  // creat ethe GC now that the window is realized
  gc = gdk_gc_new(wdraw->window);
  gdk_rgb_gc_set_foreground(gc, (guint32) 0xFF0000);

  backgc = gdk_gc_new(wdraw->window);
  gdk_rgb_gc_set_foreground(backgc, (guint32) 0xFFFFFF);
  gdk_rgb_gc_set_background(backgc, (guint32) 0xFFFFFF);

  fontgc = gdk_gc_new(wdraw->window);
  gdk_rgb_gc_set_foreground(fontgc, (guint32) 0x000000);

  // create the pixmap
  if (pixmap)
    gdk_pixmap_unref(pixmap);
  pixmap = gdk_pixmap_new(wdraw->window, wdraw->allocation.width,
			  wdraw->allocation.height, -1);
  if (pixmap == NULL) {
    gtk_widget_destroy(wmain);
    fprintf(stderr, "ERROR: GraphDlg CAnnot allocate pixmap\n");
    return (false);
  }

  // create the font
  if (font) 
    gdk_font_unref(font);
  fontsize = 10;

  static char fontstr[256];
  sprintf(fontstr, "-*-helvetica-medium-r-*-*-%d-*-*-*-*-*-*-*", fontsize);
  font = gdk_font_load(fontstr);

  if (initialized)
    draw();

  return (true);
}

/*********************** GRAPHDLG2D ************************************/


bool GraphDlg2D::createWindow(void)
{
  // create the windows
  wmain = create_graph_display();
  if (wmain == NULL)
    return (false);

  wdraw = lookup_widget(wmain, "graph_drawarea");
  if (wdraw == NULL) {
    gtk_widget_destroy(wmain);
    return (false);
  }

  // connect the signals
  gtk_signal_connect(GTK_OBJECT(wdraw), "expose_event", GTK_SIGNAL_FUNC(GraphDlg2DExpose),
		     (gpointer) this);
  gtk_signal_connect(GTK_OBJECT(wmain), "destroy", GTK_SIGNAL_FUNC(GraphDlg2DDestroy),
		     (gpointer) this);
  gtk_signal_connect(GTK_OBJECT(wdraw), "configure_event", GTK_SIGNAL_FUNC(GraphDlg2DConfigure),
		     (gpointer) this);

  GtkWidget *w = lookup_widget(wmain, "GraphUpdate");
  gtk_signal_connect(GTK_OBJECT(w), "clicked", GTK_SIGNAL_FUNC(GraphDlg2DUpdateClicked),
		     (gpointer) this);
  w = lookup_widget(wmain, "GraphClose");
  gtk_signal_connect(GTK_OBJECT(w), "clicked", GTK_SIGNAL_FUNC(GraphDlg2DCloseClicked),
		     (gpointer) this);

  gtk_widget_show(wmain);
  return (true);
}

// wet the main window widget pointer 
bool GraphDlg2D::create(ushort *data, uint _xsize, uint _ysize)
{
  if (initialized) {
    fprintf(stderr, "WARNING: Calling GraphDialog::create multiple times\n");
    return (false);
  }

  xsize = _xsize;
  ysize = _ysize;
  size = xsize * ysize;
  isrcdata = data;
  idata = new ushort[size];
  memcpy(idata, isrcdata, size * sizeof(ushort));

  if (!createWindow()) {
    fprintf(stderr, "ERROR: Cannot create GraphDialog window\n");
    return (false);
  }

  initialized = true;
  draw_ushort();

  return (true);
}

bool GraphDlg2D::create(double *data, uint _xsize, uint _ysize)
{
  if (initialized) {
    fprintf(stderr, "WARNING: Calling GraphDialog::create multiple times\n");
    return (false);
  }

  if (!createWindow()) {
    fprintf(stderr, "ERROR: Cannot create GraphDialog window\n");
    return (false);
  }

  xsize = _xsize;
  ysize = _ysize;
  size = xsize * ysize;
  ddata = new double[size];
  dsrcdata = data;
  memcpy(ddata, dsrcdata, size * sizeof(double));
  draw_double();
  initialized = true;

  return (true);
}

// update and draw at the end of the frame
void GraphDlg2D::update(void)
{
  if (!initialized)
    return;

  fprintf(stderr, "Updateing graphdlg\n");

  if (dtype) {
    memcpy(ddata, dsrcdata, size * sizeof(double));
    draw_double();
  } else {
    memcpy(idata, isrcdata, size * sizeof(ushort));
    draw_ushort();
  }
}

void GraphDlg2D::update(ushort *data)
{
  if (!initialized || dtype)
    return;
  memcpy(idata, isrcdata, size * sizeof(ushort));
  draw_ushort();
}
  
void GraphDlg2D::update(double *data)
{
  if (!initialized || !dtype)
    return;
  memcpy(ddata, dsrcdata, size * sizeof(double));
  draw_double();
}

void GraphDlg2D::draw(void)
{
  if (!initialized)
    return;

  if (dtype)
    draw_double();
  else
    draw_ushort();
}

void GraphDlg2D::redraw(GdkEventExpose *event)
{
  if (!initialized)
    return;
  gdk_draw_pixmap(wdraw->window,
		  wdraw->style->fg_gc[GTK_WIDGET_STATE(wdraw)],
		  pixmap,
                  event->area.x, event->area.y,
                  event->area.x, event->area.y,
                  event->area.width, event->area.height);		  
}

void GraphDlg2D::redraw(void)
{
  if (!initialized)
    return;
  gdk_draw_pixmap(wdraw->window,
  		  wdraw->style->fg_gc[GTK_WIDGET_STATE(wdraw)],
  		  pixmap, 0, 0,
		  wdraw->allocation.width, wdraw->allocation.height,
		  wdraw->allocation.width, wdraw->allocation.height);
}

void GraphDlg2D::draw_ushort(void)
{
  if (!initialized) {
    fprintf(stderr, "GraphDLG: Draw_ushort not initialized\n");
    return;
  }

  int w = wdraw->allocation.width, h = wdraw->allocation.height;
  vector2d delta;
  ushort maxv = 0;

  // work out the scale
  for (ushort *ptr = idata; ptr < idata + size; ptr++) {
    if (*ptr > maxv)
      maxv = *ptr;
  }


  delta.set((double) w / (double) xsize,(double) h / (double) ysize);
  double scale = 255.0 / (double) maxv;
  int bwidth = max(1, (int) delta.x - 1);
  int bheight = max(1, (int) delta.y - 1);
  vector2d p(0, h - bheight);

  // draw teh background
  gdk_draw_rectangle(pixmap, backgc, true, 0, 0, w, h);

  for (uint y = 0; y < ysize; y++) {
    for (uint x = 0; x < xsize; x++) {
      uint tmp = (uint) (idata[y * xsize + x] * scale);
      guint32 color = tmp | (tmp << 8) | (tmp << 16);
      gdk_rgb_gc_set_foreground(gc, color);

      // draw the background
      gdk_draw_rectangle(pixmap, gc, true, (int) p.x, (int) p.y, bwidth, bheight);
      p.x += delta.x;
    }
    p.y -= delta.y;
    p.x = 0;
  }
}

void GraphDlg2D::draw_double(void)
{
}


void GraphDlg2D::clear(void)
{
  if (!initialized) 
    return;

  fprintf(stderr, "Clearing...\n");

  if (pixmap) {
    gdk_pixmap_unref(pixmap);
    pixmap = NULL;
  }
  if (font) {
    gdk_font_unref(font);
    font = NULL;
  }

  if (dtype) {
    delete ddata;
  } else {
    delete idata;
  }

  gtk_widget_destroy(wmain);
  initialized = false;
}

bool GraphDlg2D::realize(GtkWidget *widget, GdkEventConfigure *event)
{
  // creat ethe GC now that the window is realized
  gc = gdk_gc_new(wdraw->window);
  gdk_rgb_gc_set_foreground(gc, (guint32) 0xFF0000);

  backgc = gdk_gc_new(wdraw->window);
  gdk_rgb_gc_set_foreground(backgc, (guint32) 0xFFFFFF);
  gdk_rgb_gc_set_background(backgc, (guint32) 0xFFFFFF);

  fontgc = gdk_gc_new(wdraw->window);
  gdk_rgb_gc_set_foreground(fontgc, (guint32) 0x000000);

  // create the pixmap
  if (pixmap)
    gdk_pixmap_unref(pixmap);
  pixmap = gdk_pixmap_new(wdraw->window, wdraw->allocation.width,
			  wdraw->allocation.height, -1);
  if (pixmap == NULL) {
    gtk_widget_destroy(wmain);
    fprintf(stderr, "ERROR: GraphDlg CAnnot allocate pixmap\n");
    return (false);
  }

  // create the font
  if (font) 
    gdk_font_unref(font);
  fontsize = 10;

  static char fontstr[256];
  sprintf(fontstr, "-*-helvetica-medium-r-*-*-%d-*-*-*-*-*-*-*", fontsize);
  font = gdk_font_load(fontstr);

  if (initialized)
    draw();

  return (true);
}

/******************************** LOCAL CODE **********************************/

static gboolean GraphDlg1DExpose(GtkWidget *widget, GdkEventExpose  *event,
		gpointer user_data)
{
  fprintf(stderr, "expose graphdlg...\n");

  GraphDlg1D *ptr = (GraphDlg1D *) user_data;
  ptr->redraw(event);
  return (false);
}

static void GraphDlg1DDestroy(GtkObject *object, gpointer user_data)
{
  fprintf(stderr, "destroy graphdlg...\n");

  GraphDlg1D *ptr = (GraphDlg1D *) user_data;
  ptr->clear();
}


static gboolean GraphDlg1DConfigure(GtkWidget *widget, GdkEventConfigure *event,
				  gpointer user_data)
{
  fprintf(stderr, "Configure graphdlg...\n");

  GraphDlg1D *ptr = (GraphDlg1D *) user_data;
  return (ptr->realize(widget, event));
}

static void GraphDlg1DUpdateClicked(GtkButton *button,
				  gpointer user_data)
{
  fprintf(stderr, "Update graphdlg...\n");

  GraphDlg1D *ptr = (GraphDlg1D *) user_data;
  return (ptr->update());
}

static void GraphDlg1DCloseClicked(GtkButton *button,
				  gpointer user_data)
{
  fprintf(stderr, "Configure close...\n");

  GraphDlg1D *ptr = (GraphDlg1D *) user_data;
  gtk_widget_destroy(ptr->wmain);
}


static gboolean GraphDlg2DExpose(GtkWidget *widget, GdkEventExpose  *event,
		gpointer user_data)
{
  fprintf(stderr, "expose graphdlg...\n");

  GraphDlg2D *ptr = (GraphDlg2D *) user_data;
  ptr->redraw(event);
  return (false);
}

static void GraphDlg2DDestroy(GtkObject *object, gpointer user_data)
{
  fprintf(stderr, "destroy graphdlg...\n");

  GraphDlg2D *ptr = (GraphDlg2D *) user_data;
  ptr->clear();
}


static gboolean GraphDlg2DConfigure(GtkWidget *widget, GdkEventConfigure *event,
				  gpointer user_data)
{
  fprintf(stderr, "Configure graphdlg...\n");

  GraphDlg2D *ptr = (GraphDlg2D *) user_data;
  return (ptr->realize(widget, event));
}

static void GraphDlg2DUpdateClicked(GtkButton *button,
				  gpointer user_data)
{
  fprintf(stderr, "Update graphdlg...\n");

  GraphDlg2D *ptr = (GraphDlg2D *) user_data;
  return (ptr->update());
}

static void GraphDlg2DCloseClicked(GtkButton *button,
				  gpointer user_data)
{
  fprintf(stderr, "Configure close...\n");

  GraphDlg2D *ptr = (GraphDlg2D *) user_data;
  gtk_widget_destroy(ptr->wmain);
}

