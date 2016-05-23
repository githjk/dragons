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

#include "field.h"
#include "utils/polygon.h"

#include "pixmaps/field-2002.xpm"

/*
#define FIELD_WIDE
#define FIELD_2002

#if defined(FIELD_2002)
#elif defined(FIELD_WIDE)
#include "pixmaps/field-wide.xpm"
#else
#include "pixmaps/field.xpm"
#endif
*/

// debug colors: ROYGBIV
static const guint32 debug_colors[] = {
  0xFF0000, 0xFF8000, 0xFFFF00, 0x00FF00, 0x000077, 0x00FFFF
};

Field::Field()
{
  initialized = false;
  current_level = 0;
  widget = NULL;
}

void Field::Initialize(GtkWidget *drawing_area, GdkEventConfigure *event)
{
  widget = drawing_area;

/*
#if defined(FIELD_2002)
  field_pix = gdk_pixbuf_new_from_xpm_data((const char **) field_2002_xpm);
  field_pix_width = 971;
  field_pix_field_width = 884;
  field_pix_height = 829;
  field_pix_field_height = 720;
#elif defined(FIELD_WIDE)
  field_pix = gdk_pixbuf_new_from_xpm_data((const char **) field_wide_xpm);
  field_pix_width = 959;
  field_pix_field_width = 875;
  field_pix_height = 813;
  field_pix_field_height = 703;
#else
  field_pix = gdk_pixbuf_new_from_xpm_data((const char **) field_xpm);
  field_pix_width = 959;
  field_pix_field_width = 875;
  field_pix_height = 589;
  field_pix_field_height = 483;
#endif
*/
  field_pix = gdk_pixbuf_new_from_xpm_data((const char **) field_2002_xpm);
  field_pix_width = 971;
  field_pix_field_width = 884;
  field_pix_height = 829;
  field_pix_field_height = 720;

  field_gc = gdk_gc_new(widget->window);
  gdk_rgb_gc_set_foreground(field_gc, (guint32) 0x484848);
  gdk_gc_set_fill(field_gc, GDK_TILED);

  ball_gc = gdk_gc_new(widget->window);
  gdk_rgb_gc_set_foreground(ball_gc, (guint32) 0xFF0000);

  robot_gc[0] = gdk_gc_new(widget->window);
  gdk_rgb_gc_set_foreground(robot_gc[0], (guint32) 0x0000FF);

  robot_gc[1] = gdk_gc_new(widget->window);
  gdk_rgb_gc_set_foreground(robot_gc[1], (guint32) 0xFFFF00);

  outline_gc = gdk_gc_new(widget->window);
  gdk_rgb_gc_set_foreground(outline_gc, (guint32) 0x000000);

  selected_gc = gdk_gc_new(widget->window);
  gdk_rgb_gc_set_foreground(selected_gc, (guint32) 0xFFFFFF);

  id_gc = gdk_gc_new(widget->window);
  gdk_rgb_gc_set_foreground(id_gc, (guint32) 0x000000);

  debug_gc = gdk_gc_new(widget->window);
  gdk_rgb_gc_set_foreground(debug_gc, (guint32) 0xFF00FF);

  gtk_widget_set_usize(widget, 
		       (int) field_pix_width / 3, 
		       (int) field_pix_height / 3);

  GTK_WIDGET_SET_FLAGS(widget, GTK_CAN_FOCUS);
  gtk_widget_grab_focus(widget);

  gtk_signal_connect(GTK_OBJECT(widget), "configure_event",
		     (GtkSignalFunc) h_configure, this);
  gtk_signal_connect(GTK_OBJECT(widget), "expose_event",
		     (GtkSignalFunc) h_expose, this);

  selection = SelectionNone;
  selection_id = 0;

  initialized = true;

  option_trails     = false;
  option_blue_ids   = true;
  option_yellow_ids = true;

  if (event) Configure(widget, event);
}

void Field::Clean()
{
  DrawField(pixmap, field_gc);
  DrawFrame();
  DrawDebugs();
  gtk_widget_queue_draw(widget);
}

void Field::Update(net_vframe &frame)
{
  UndrawFrame();
  UndrawDebugs();

  the_frame = frame;
  the_debugs = new_debugs; 
  new_debugs.clear();
  
  DrawFrame();
  DrawDebugs();
}

void Field::Debug(net_gdebug &debug)
{
  new_debugs.push_back(debug);
}

void Field::SelectBall()
{
  selection = SelectionBall;
}

void Field::SelectRobot(char team, char robot)
{
  selection = (team == 0) ? SelectionBlue : SelectionYellow;
  selection_id = robot;
}

void Field::Select(SelectionType mask, int sx, int sy)
{
  double found_dist = HUGE;

  selection = SelectionNone;
  selection_id = 0;

  vector2d click(field_x(sx), field_y(sy));

  // Check Ball
  if (mask & SelectionBall) {
    double d = (click - vector2d(the_frame.ball.state.x,
				 the_frame.ball.state.y)).length();
    if (d < BALL_RADIUS && d < found_dist) {
      selection = SelectionBall;
      selection_id = 0;
      found_dist = d;
    }
  }

  // Check Robots
  for(int i=0; i<NUM_TEAMS; i++) {
    if (i == 0 && !(mask & SelectionBlue)) continue;
    if (i == 1 && !(mask & SelectionYellow)) continue;

    for(int j=0; j<MAX_TEAM_ROBOTS; j++) {
      if (the_frame.config.teams[i].robots[j].id < 0) continue;

      double d = (click - vector2d(the_frame.robots[i][j].state.x,
				   the_frame.robots[i][j].state.y)).length();
      if (d < ROBOT_DEF_WIDTH_H && d < found_dist) {
	selection = (i == 0) ? SelectionBlue : SelectionYellow;
	selection_id = j;
	found_dist = d;
      }
    }
  }
}

void Field::h_configure(GtkWidget *widget, GdkEventConfigure *event,
			Field *f)
{
  f->Configure(widget, event);
}

gint Field::h_expose(GtkWidget *widget, GdkEventExpose *event, Field *f)
{ 
  return f->Expose(widget, event);
}

gint Field::h_configure_pixbuf(Field *f)
{
  return f->ConfigurePixbuf();
}

void Field::Configure(GtkWidget *w, GdkEventConfigure *event)
{
  // Setup field tile and backing pixmap
  if (field_tile) gdk_pixmap_unref(field_tile);
  field_tile = gdk_pixmap_new(widget->window,
			      widget->allocation.width,
			      widget->allocation.height,
			      -1);
  gdk_gc_set_tile(field_gc, field_tile);

  if (pixmap) gdk_pixmap_unref(pixmap);
  pixmap = gdk_pixmap_new(widget->window,
			      widget->allocation.width,
			      widget->allocation.height,
			      -1);
  // Reset font
  if (id_font) gdk_font_unref(id_font);
  static char fontstr[256];
  sprintf(fontstr, "-*-helvetica-medium-r-*-*-%d-*-*-*-*-*-*-*", fontsize());
  id_font = gdk_font_load(fontstr);

  // Setup coordinate system
  pixels_per_mm_x = 
    (widget->allocation.width * field_pix_field_width / field_pix_width) /
    FIELD_LENGTH;
  pixels_per_mm_y = 
    (widget->allocation.height * field_pix_field_height / field_pix_height) /
    FIELD_WIDTH;
  center_pixel_x = widget->allocation.width / 2;
  center_pixel_y = widget->allocation.height / 2;

  // When idle rescale pixbuf
  gtk_idle_add((GtkFunction) &Field::h_configure_pixbuf, this);
}

gint Field::ConfigurePixbuf()
{
  if (field_pix_scaled) gdk_pixbuf_unref(field_pix_scaled);
  if (field_pix && (widget->allocation.width > 1 || 
		    widget->allocation.height > 1)) {
    field_pix_scaled = gdk_pixbuf_new(GDK_COLORSPACE_RGB, 1, 8, 
				      widget->allocation.width, 
				      widget->allocation.height);
    gdk_pixbuf_scale(field_pix, field_pix_scaled,
		     0, 0, widget->allocation.width, widget->allocation.height,
		     0, 0, 
		     scale_x(), scale_y(),
		     GDK_INTERP_BILINEAR);

    gdk_pixbuf_render_to_drawable(field_pix_scaled, field_tile,
				  field_gc,
				  0, 0, 0, 0,
				  widget->allocation.width,
				  widget->allocation.height,
				  GDK_RGB_DITHER_NORMAL, 0, 0);

  } else field_pix_scaled = NULL;

  DrawField(pixmap, field_gc);
  gtk_widget_queue_draw(widget);

  return FALSE;  
}

gint Field::Expose(GtkWidget *w, GdkEventExpose *event)
{
  gdk_draw_pixmap(widget->window,
		  widget->style->fg_gc[GTK_WIDGET_STATE(widget)],
		  pixmap,
                  event->area.x, event->area.y,
                  event->area.x, event->area.y,
                  event->area.width, event->area.height);		  
  return FALSE;
}

GdkRectangle Field::DrawLine(GdkDrawable *drawable, GdkGC *gc,
			     double x1, double y1, double x2, double y2)
{
  GdkGCValues gc_values;
  gdk_gc_get_values(gc, &gc_values);
  int lw = gc_values.line_width;

  int _x1 = screen_x(x1);
  int _y1 = screen_y(y1);
  int _x2 = screen_x(x2);
  int _y2 = screen_y(y2);

  gdk_draw_line(drawable, gc, _x1, _y1, _x2, _y2);

  return (GdkRectangle) { MIN(_x1, _x2) - lw, MIN(_y1, _y2) - lw,
			    abs(_x1 - _x2) + 2*lw + 1,
			    abs(_y1 - _y2) + 2*lw + 1 };
}

GdkRectangle Field::DrawArc(GdkDrawable *drawable, GdkGC *gc,
		    double x, double y, double w, double h, 
		    int a1, int a2, bool fill)
{
  int _w = screen_w(w);
  int _h = screen_h(h);
  int _x = screen_x(x) - _w/2;
  int _y = screen_y(y) - _h/2;

  gdk_draw_arc(drawable, gc, fill, _x, _y, _w, _h, a1, a2);

  return (GdkRectangle) { _x, _y, _w+1, _h+1 };
}

GdkRectangle Field::DrawPolygon(GdkDrawable *drawable, GdkGC *gc,
			Polygon &polygon, bool fill)
{
  GdkGCValues gc_values;
  gdk_gc_get_values(gc, &gc_values);
  int lw = gc_values.line_width;

  // initialize them to something to avoid the warnings
  int minx = FIELD_LENGTH_H, maxx = 0;
  int miny = FIELD_WIDTH_H, maxy = 0;

  vector2d p[polygon.numPts];
  GdkPoint _p[polygon.numPts];

  polygon.GetVertices(p);

  for(int i=0; i<polygon.numPts; i++) {
    _p[i].x = screen_x(p[i].x);
    _p[i].y = screen_y(p[i].y);

    if (i == 0) {
      minx = maxx = _p[i].x;
      miny = maxy = _p[i].y;
    } else {
      if (_p[i].x < minx) minx = _p[i].x;
      if (_p[i].x > maxx) maxx = _p[i].x;
      if (_p[i].y < miny) miny = _p[i].y;
      if (_p[i].y > maxy) maxy = _p[i].y;
    }
  }

  gdk_draw_polygon(drawable, gc, fill, _p, polygon.numPts);

  return (GdkRectangle) { minx - lw, miny - lw, 
			    maxx - minx + 2*lw + 1, maxy - miny + 2*lw + 1 };
}

GdkRectangle Field::DrawField(GdkDrawable *drawable, GdkGC *field_gc,
			      GdkRectangle *r)
{
  if (r) {
    gdk_draw_rectangle(drawable, field_gc, true, 
		       r->x, r->y, r->width, r->height);
    return *r;
  } else {
    gdk_draw_rectangle(drawable, field_gc, true,
		       0, 0, 
		       widget->allocation.width, 
		       widget->allocation.height);
    return (GdkRectangle) { 0, 0, 
			      widget->allocation.width, 
			      widget->allocation.height };
  }
}

GdkRectangle Field::DrawRobot(GdkDrawable *drawable, 
			      GdkGC *robot_gc, GdkGC *outline_gc, GdkGC *id_gc,
			      char type, bool show_id, char id,
			      double x, double y, double h)
{
  static Polygon poly_diff = POLY_DIFF;
  static Polygon poly_omni = POLY_OMNI;
  GdkRectangle clip;

  if (type == ROBOT_TYPE_DIFF || type == ROBOT_TYPE_OMNI) {
    Polygon &p = (type == ROBOT_TYPE_DIFF) ? poly_diff : poly_omni;

    /*
    if(type == ROBOT_TYPE_DIFF){
      p.SetPolygon(poly_diff_pts,poly_diff_x,poly_diff_y);
    }else{
      p.SetPolygon(poly_omni_pts,poly_omni_x,poly_omni_y);
    }
    */

    p.RotateTranslate(h, vector2d(x, y));

    DrawPolygon(drawable, robot_gc, p, true);
    clip = DrawPolygon(drawable, outline_gc, p, false); 
  } else {
    DrawArc(drawable, robot_gc, x, y,
	    OPPONENT_EFFECTIVE_RADIUS * 2.0,
	    OPPONENT_EFFECTIVE_RADIUS * 2.0,
	    0, 360 * 64, true);
    clip = DrawArc(drawable, outline_gc, x, y,
		   OPPONENT_EFFECTIVE_RADIUS * 2.0,
		   OPPONENT_EFFECTIVE_RADIUS * 2.0,
		   0, 360 * 64, false);
  }	    

  if (show_id) {
    static char str[3];
    sprintf(str, "%d", id);
    gdk_draw_text(drawable, id_font, id_gc, 
		  screen_x(x) - (int) (fontsize() / 4.0),
		  screen_y(y) + (int) (fontsize() / 2.0),
		  str, 1);
  }

  return clip;
}

GdkRectangle Field::DrawBall(GdkDrawable *drawable, 
		     GdkGC *ball_gc, GdkGC *outline_gc,
		     double x, double y)
{
  // These are not used. No idea why!!!
  //  int _x = screen_x(x);
  //  int _y = screen_y(y);
  //  int _w = screen_w(BALL_DIAMETER);
  //  int _h = screen_h(BALL_DIAMETER);

  DrawArc(drawable, ball_gc, x, y, BALL_DIAMETER, BALL_DIAMETER,
  	  0, 360 * 64, 1);

  return DrawArc(drawable, 
		 (selection & SelectionBall) ? selected_gc : outline_gc, 
		 x, y, BALL_DIAMETER, BALL_DIAMETER,
		 0, 360 * 64, 0);
}

#define ARROW_TIP_LENGTH 50

GdkRectangle Field::DrawDebug(GdkDrawable *drawable, GdkGC *gc, 
			      net_gdebug *debug)
{
  if (!(debug->level & current_level)) 
    return ((GdkRectangle) {0,0,0,0});

  if (SelectedRobot() &&
      ((SelectedRobotTeam() != debug->team) ||
       (SelectedRobotID() != debug->robot)))
    return ((GdkRectangle) {0,0,0,0});

  // set the color of gc based on the debug level
  gdk_rgb_gc_set_foreground(gc, GetDebugColor(debug->level));

  switch(debug->msgtype) {
  case NET_GUI_DEBUG_LINE: {
    GdkRectangle clip, r;

    gdebug_line &d = debug->info.line;
    clip = DrawLine(drawable, gc, 
		    d.p[0].x, d.p[0].y, 
		    d.p[1].x, d.p[1].y);

    double dir = atan2(d.p[1].y - d.p[0].y, 
		       d.p[1].x - d.p[0].x);

    if (d.flags & G_ARROW_FORW) {
      r = DrawLine(drawable, gc, 
		   d.p[1].x, d.p[1].y, 
		   d.p[1].x - ARROW_TIP_LENGTH * cos(dir + M_PI_8),
		   d.p[1].y - ARROW_TIP_LENGTH * sin(dir + M_PI_8));
      gdk_rectangle_union(&r, &clip, &clip);
      DrawLine(drawable, gc, 
	       d.p[1].x, d.p[1].y, 
	       d.p[1].x - ARROW_TIP_LENGTH * cos(dir - M_PI_8),
	       d.p[1].y - ARROW_TIP_LENGTH * sin(dir - M_PI_8));
      gdk_rectangle_union(&r, &clip, &clip);
    }

    if (d.flags & G_ARROW_BACK) {
      r = DrawLine(drawable, gc, 
	       d.p[0].x, d.p[0].y, 
	       d.p[0].x + ARROW_TIP_LENGTH * cos(dir + M_PI_8),
	       d.p[0].y + ARROW_TIP_LENGTH * sin(dir + M_PI_8));
      gdk_rectangle_union(&r, &clip, &clip);
      r = DrawLine(drawable, gc, 
	       d.p[0].x, d.p[0].y, 
	       d.p[0].x + ARROW_TIP_LENGTH * cos(dir - M_PI_8),
	       d.p[0].y + ARROW_TIP_LENGTH * sin(dir - M_PI_8));
      gdk_rectangle_union(&r, &clip, &clip);
    }

    return clip;
    }

    break;
  case NET_GUI_DEBUG_ARC: {
    gdebug_arc &d = debug->info.arc;
    return DrawArc(drawable, gc, 
		   d.center.x, d.center.y,
		   d.dimens.x, d.dimens.y,
		   (int) (RAD2DEG(d.a1) * 64),
		   (int) (RAD2DEG(d.a2 - d.a1) * 64), 
		   d.flags & G_ARC_FILL);
    }
    break;
  default:
    break;
  }

  return (GdkRectangle) {0,0,0,0};
}

// For drawing the frame, we draw into the pixmap and then queue
//  redraws to get it to the widget's window.  This keeps flicker
//  low to have the robots look nice.
//
// For drawing debug information we draw directly into the widget
//  window and pixmap.  Undrawing is done by just drawing with 
//  the field as a tile on the GC.  This makes redraws fast, although
//  it causes flicker.

void Field::DrawFrame()
{
  // We draw into the backing pixmap and then queue up the redraw
  //  rectangles to go to the screen.
  GdkRectangle r;


  // Draw Robots
  for(int i=0; i< NUM_TEAMS; i++) {
    bool team_selected = 
      (i == 0 && selection == SelectionBlue) ||
      (i == 1 && selection == SelectionYellow);

    for(int j=0; j < MAX_TEAM_ROBOTS; j++) {
      if (the_frame.config.teams[i].robots[j].id < 0) continue;

      bool selected = (team_selected && selection_id == j);
      r = DrawRobot(pixmap, robot_gc[i], 
		    selected ? selected_gc : outline_gc, id_gc,
		    the_frame.config.teams[i].robots[j].type,
		    (i == 0 ? option_blue_ids : option_yellow_ids),
		    the_frame.config.teams[i].robots[j].id,
		    the_frame.robots[i][j].state.x, 
		    the_frame.robots[i][j].state.y,
		    the_frame.robots[i][j].state.theta);
      the_frame_clips.push_front(r);
      gtk_widget_queue_draw_area(widget, r.x, r.y, r.width, r.height);
    }
  }

  // Draw Ball
  r = DrawBall(pixmap, ball_gc, outline_gc, 
	       the_frame.ball.state.x, the_frame.ball.state.y);
  the_frame_clips.push_front(r);
  gtk_widget_queue_draw_area(widget, r.x, r.y, r.width, r.height);

}

void Field::UndrawFrame()
{
  while(!the_frame_clips.empty()) {
    GdkRectangle r = the_frame_clips.front(); the_frame_clips.pop_front();
    if (!option_trails) {
      DrawField(pixmap, field_gc, &r);
      gtk_widget_queue_draw_area(widget, r.x, r.y, r.width, r.height);
    }
  }
}

void Field::DrawDebugs()
{
  for(unsigned int i=0; i<the_debugs.size(); i++) {
    DrawDebug(pixmap, debug_gc, &the_debugs[i]);
    DrawDebug(widget->window, debug_gc, &the_debugs[i]);
  }
}

void Field::UndrawDebugs()
{
  for(unsigned int i=0; i<the_debugs.size(); i++) {
    DrawDebug(pixmap, field_gc, &the_debugs[i]);
    DrawDebug(widget->window, field_gc, &the_debugs[i]);
  }
}


guint32 Field::GetDebugColor(int lvl)
{
  int j, i;

  for (i = 1, j = 0; i <= GDBG_STRATEGY; i <<= 1, j++) {
    if (i & lvl)
      return (debug_colors[j]);
  }
  return (0);
}

