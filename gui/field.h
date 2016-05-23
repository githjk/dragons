/*
 * TITLE:        field.h
 *
 * PURPOSE:      This file handles all of the drawing of the field display.
 *               
 * WRITTEN BY:   Michael Bowling, Brett Browning
 *               Code ported from small size 2001 GUI
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

#ifndef __FIELD_H__
#define __FIELD_H__

#include <gtk/gtk.h>
#include <gdk-pixbuf/gdk-pixbuf.h>

#include <deque>

#include <polygon.h>
#include <reality/net_vision.h>
#include <soccer/net_gui.h>

class Field {
public:
  Field();

  void Initialize(GtkWidget *drawing_area,
		  GdkEventConfigure *event = NULL);
  bool IsInitialized() { return initialized; }

  void Update(net_vframe &frame);
  void Debug(net_gdebug &debug);

  void SetDebugLevel(int lvl) {
    current_level = lvl;
  }
  int GetDebugLevel(void) {
    return (current_level);
  }

  void ToggleDebugLevel(int lvl) {
    current_level = current_level ^ lvl;
  }

  guint32 GetDebugColor(int lvl);
    

  void Clean();

  // get operations
  GtkWidget *GetWidget(void) {
    return widget;
  }

  // Options
  void OptionTrails(bool b) { option_trails = b; }
  void OptionBlueIDs(bool b) { option_blue_ids = b; }
  void OptionYellowIDs(bool b) { option_yellow_ids = b; }

  // Selection... 
  //
  // Selection(id) returns the current selected type, and places its
  // id in the reference argument
  //
  // SelectBall() and SelectRobot() do the obvious things.
  //
  // Select(mask, sx, sy) selects the nearest object in the mask to
  // the given screen coordinate.
  //

  enum SelectionType { SelectionNone = 0, 
		       SelectionBall = (1 << 0),
		       SelectionBlue = (1 << 1),
		       SelectionYellow = (1 << 2),
                       SelectionAll = ~0 };

  void SelectBall();
  void SelectRobot(char team, char robot);
  void Select(SelectionType mask, int sx, int sy);

  inline bool Selected() { return (bool) selection; }
  inline bool SelectedBall() { return selection & SelectionBall; }
  inline bool SelectedRobot() { 
    return selection & (SelectionBlue | SelectionYellow); }
  inline char SelectedRobotTeam() { 
    return (selection & SelectionBlue) ? TEAM_BLUE : TEAM_YELLOW; }
  inline char SelectedRobotID() {
    return selection_id; }
  
  // Coordinate System Routines
  inline int screen_x(double field_x) { 
    return (int) rint(field_x * pixels_per_mm_x + center_pixel_x); }
  inline double field_x(double screen_x) {
    return (screen_x - center_pixel_x) / pixels_per_mm_x; }
  inline int screen_w(double field_w) {
    return (int) rint(field_w * pixels_per_mm_x); }
  inline double scale_x() {
    return widget->allocation.width / (double) field_pix_width; }


  inline int screen_y(double field_y) { 
    return (int) rint(-field_y * pixels_per_mm_y + center_pixel_y); }
  inline double field_y(double screen_y) {
    return - (screen_y - center_pixel_y) / pixels_per_mm_y; }
  inline int screen_h(double field_h) {
    return (int) rint(field_h * pixels_per_mm_y); }
  inline double scale_y() {
    return widget->allocation.height / (double) field_pix_height; }

  inline int fontsize() {
    return MAX((int) rint(16.0 * scale_y()), 8); }

private:
  bool initialized;

  // Display Details
  char current_level;

  // Drawing Details
  GtkWidget *widget;  // drawing_area widget
  GdkPixmap *pixmap;  // buffer pixmap for expose events

  GdkPixbuf *field_pix, *field_pix_scaled;
  double field_pix_height, field_pix_width;
  double field_pix_field_height, field_pix_field_width;
  GdkPixmap *field_tile;

  GdkGC *field_gc, *ball_gc, *robot_gc[2];
  GdkGC *outline_gc, *selected_gc, *id_gc, *debug_gc;

  GdkFont *id_font;

  // Drawing Options
  bool option_trails, option_blue_ids, option_yellow_ids;

  // Data to Draw
  net_vframe the_frame;
  deque<net_gdebug> the_debugs, new_debugs;

  SelectionType selection;
  char selection_id;

  deque<GdkRectangle> the_frame_clips;

  // Callbacks
  static void h_configure(GtkWidget *widget, GdkEventConfigure *event, 
			  Field *f);
  static gint h_expose(GtkWidget *widget, GdkEventExpose *event, Field *f);
  static gint h_configure_pixbuf(Field *f);

  // Coordinate System Info
  double pixels_per_mm_x, pixels_per_mm_y;
  int center_pixel_x, center_pixel_y;


  // Callback Routines
  void Configure(GtkWidget *w, GdkEventConfigure *event);
  gint ConfigurePixbuf();
  gint Expose(GtkWidget *w, GdkEventExpose *event);

  // Primitive Drawing Routines
  GdkRectangle DrawLine(GdkDrawable *drawable, GdkGC *gc,
			double x1, double y1, double x2, double y2);
  GdkRectangle DrawArc(GdkDrawable *drawable, GdkGC *gc,
		       double x, double y, double w, double h, 
		       int a1, int a2, bool fill);
  GdkRectangle DrawPolygon(GdkDrawable *drawable, GdkGC *gc,
			   Polygon &p, bool fill);
    
  GdkRectangle DrawField(GdkDrawable *drawable, GdkGC *field_gc,
			 GdkRectangle *r = NULL);
  GdkRectangle DrawRobot(GdkDrawable *drawable, 
			 GdkGC *robot_gc, GdkGC *outline_gc, GdkGC *id_gc,
			 char type, bool show_id, char id,
			 double x, double y, double h);
  GdkRectangle DrawBall(GdkDrawable *drawable, 
			GdkGC *ball_gc, GdkGC *outline_gc,
			double x, double y);
  GdkRectangle DrawDebug(GdkDrawable *drawable, GdkGC *gc, net_gdebug *d);

  // Frame Drawing
  void DrawFrame();
  void UndrawFrame();

  void DrawDebugs();   // Draws new_debugs.
  void UndrawDebugs(); // Undraws old_debugs.
};

#endif



