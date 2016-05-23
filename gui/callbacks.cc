/*
 * TITLE:        callbacks.cc
 *
 * PURPOSE:      This is the main callback file  for the smallsize GUI
 *               for 2002. The GUI was generated initially using the 
 *               glade program and it should be used to add any new signals
 *               Glade will only add ot this file however, so you should delete
 *               unwanted functions.
 *               
 * WRITTEN BY:   Michael Bowling, Brett Browning, 
 *               some code ported from small size 2001 GUI
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

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <gdk-pixbuf/gdk-pixbuf.h>
#include <stdio.h>
#include <ctype.h>
#include <unistd.h>

#include "callbacks.h"
#include "interface.h"
#include "support.h"

#include "gtkutils.h"

#include "field.h"
#include "graphdlg.h"
#include "main.h"

// compile controls
//#define DEBUG


/**************************** TYPES ***********************************/

#define CHOOSE_OUR_TEAM(t, b, y)      (((t) == TEAM_BLUE) ? (b) : (y))
#define CHOOSE_THEIR_TEAM(t, b, y)      (((t) == TEAM_BLUE) ? (y) : (b))


// speed increments for driving robot
#define SPEED_INCR    200.0
#define ROT_INCR      1.0 //0.25

struct Drive {
  double timestamp;
  bool driving;
  gint callbackid;
  int team, id;
  vector3d vcmd;
  vector3d gain;
  bool kick, drib;
  double oldx;
  double newx;
};

/**************************** PROTOTYPSE ******************************/
gint DriveCallback(void);

/**************************** GLOBALS *********************************/

GtkWidget *param_ctrl_widget = NULL;
GtkWidget *tactic_ctrl_widget = NULL;

static GtkWidget *rbox_restart, *rbox_kickoff[2];
static GtkWidget *rbox_penalty[2], *rbox_freekick[2];
static GtkToggleButton *enable_button[2], *cover_button[2];

static Drive drive;

// config parameters
CR_DECLARE(BLUE_COVERS);
CR_DECLARE(BLUE_ENABLE);
CR_DECLARE(BLUE_ROBOTS);
CR_DECLARE(YELLOW_COVERS);
CR_DECLARE(YELLOW_ENABLE);
CR_DECLARE(YELLOW_ROBOTS);

//GraphDlgs graphdlgs;

extern Modeller modeller;

/**************************** CODE ************************************/


/**************************** Menu Controls ***************************/

void OnFileActivate(GtkMenuItem *menuitem,gpointer user_data)
{

}


void on_FileConnect_activate(GtkMenuItem *menuitem, gpointer user_data)
{
  do_disconnect();
  do_connect();
}


void on_Refresh_activate(GtkMenuItem *menuitem, gpointer user_data)
{
  field.Clean();
}


void on_Trails_activate(GtkMenuItem *menuitem, gpointer user_data)
{
  field.OptionTrails(GTK_CHECK_MENU_ITEM(menuitem)->active);
}


void on_ID_activate(GtkMenuItem *menuitem, gpointer user_data)
{
  field.OptionBlueIDs(GTK_CHECK_MENU_ITEM(menuitem)->active);
  field.OptionYellowIDs(GTK_CHECK_MENU_ITEM(menuitem)->active);
}


void on_OpponentID_activate(GtkMenuItem *menuitem, gpointer xuser_data)
{
  field.OptionYellowIDs(GTK_CHECK_MENU_ITEM(menuitem)->active);
}

/**************************** Debug Toolbar Controls **********************/

void OnDebugToolBarClicked(GtkToggleButton *togglebutton,
			   gpointer user_data)
{
  
  field.ToggleDebugLevel(GPOINTER_TO_INT(user_data));

  gint d = GPOINTER_TO_INT(user_data);

  fprintf(stderr, "Debug level %x:%x\n", d, field.GetDebugLevel());
}


void OnDebugToolbarRealize(GtkWidget *widget, gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "DebugToolbar realize\n");
#endif

  // set default settings here
}


/**************************** Drawing Controls ****************************/


/*
 * on_darea_configure_Event -
 *
 * This function is called when configuring the drawing area widget
 * that is used to draw the field and the robots. It sets the size
 * opens the pixbufs and draws the widget for the first time.
 */
gboolean on_darea_configure_event(GtkWidget *widget, GdkEventConfigure *event,
				  gpointer user_data)
{
  if (!field.IsInitialized()) {
    field.Initialize(widget, event);
  }

  // initialize the drive structure
  drive.callbackid = -1;
  drive.driving = false;
  drive.oldx = drive.newx = 0;
  drive.vcmd.set(0, 0, 0);
  drive.gain.set(0.99, 0.99, 0.05);

  return FALSE;
}

gboolean on_darea_expose_event(GtkWidget *widget, GdkEventExpose  *event,
			       gpointer user_data)
{
  return TRUE;
}


gboolean on_darea_motion_notify_event(GtkWidget *widget, GdkEventMotion  *event,
				      gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "Moving mouse\n");
#endif

  // if we are over the field then we should grab the focus
  if (widget == field.GetWidget())
    gtk_widget_grab_focus(widget);


  // if we are driving we do something different
  if (0 && drive.driving) {
    // if we are over our screen then grab the focus
    if (widget == field.GetWidget())
      gtk_widget_grab_focus(widget);

    drive.newx = event->x;
    drive.vcmd.z = (drive.oldx - drive.newx) * drive.gain.z;
    //      drive.vcmd.z = (field.screen_x(0) - event->x) * drive.gain.z;
    drive.vcmd.z = bound(drive.vcmd.z, -6.0, 6.0);

  } else {

    selection_frozen = true;

    if ((selection_button == 1 && !(event->state & GDK_BUTTON1_MASK)) ||
	(selection_button == 2 && !(event->state & GDK_BUTTON2_MASK)) ||
	(selection_button == 3 && !(event->state & GDK_BUTTON3_MASK))) {
      selection_button = 0;
    }
    
    switch(selection_button) {
      
    case 1: // left button
      if (field.SelectedBall()) {
	frame.ball.state.x = field.field_x(event->x);
	frame.ball.state.y = field.field_y(event->y);
	field.Update(frame);
      } else if (field.SelectedRobot()) {
	char team = field.SelectedRobotTeam();
	char id = field.SelectedRobotID();
	frame.robots[team][id].state.x = field.field_x(event->x);
	frame.robots[team][id].state.y = field.field_y(event->y);
	field.Update(frame);
      }
      
      break;
      
    case 2: // middle button
      if (field.SelectedRobot()) {
	char team = field.SelectedRobotTeam();
	char id = field.SelectedRobotID();
	frame.robots[team][id].state.x = field.field_x(event->x);
	frame.robots[team][id].state.y = field.field_y(event->y);
	field.Update(frame);
      }

      break;
    
    case 3: // right button
      if (field.SelectedBall()) {
	net_gdebug debug = {NET_GUI_DEBUG_LINE, -1, 0};
	debug.info.line.p[0].x = (double) frame.ball.state.x; 
	debug.info.line.p[0].y = (double) frame.ball.state.y;
	debug.info.line.p[1].x = (double) field.field_x(event->x); 
	debug.info.line.p[1].y = (double) field.field_y(event->y);
	debug.info.line.flags = G_ARROW_FORW;
	field.Debug(debug);
      } else if (field.SelectedRobot()) {
	char team = field.SelectedRobotTeam();
	char id = field.SelectedRobotID();
	double newtheta = 
	  atan2(field.field_y(event->y) - frame.robots[team][id].state.y,
		field.field_x(event->x) - frame.robots[team][id].state.x);
	frame.robots[team][id].state.theta = newtheta;
	field.Update(frame);
      }
      
      break;
      
    default:
      selection_frozen = false;
      break;
    }
  }
  return FALSE;
}


gboolean on_darea_button_press_event(GtkWidget *widget, GdkEventButton *event,
				     gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "mouse click button @ (%f, %f)\n", event->x, event->y);
#endif


  int select_flags = Field::SelectionNone;

  switch(event->button) {
  case 1: // left button
  case 3: // right button
    select_flags = Field::SelectionAll; break;
  case 2: // middle button
    select_flags = Field::SelectionYellow | Field::SelectionBlue; break;
  default:
    break;
  }

  field.Select((Field::SelectionType) select_flags, 
	       (int) event->x, (int) event->y);

  selection_button = event->button;

  int id = field.SelectedRobotID();
  int team = field.SelectedRobotTeam();

  // handle double click events
  if (event->type == GDK_2BUTTON_PRESS) {
    switch (event->button) {
    case 1: 
      // if nothing is selected then we will kickup strategy
      if (!field.Selected()) {
	if (strategydlg.win != NULL) {
	  gtk_widget_show(strategydlg.win);
	} else {
	  GtkWidget *w = create_robotdlg();

	  GtkWidget *wbox = lookup_widget(w, "robottext");
	  strategydlg.Set(w, wbox);

	  char winname[256];
	  sprintf(winname, "Strategy Dialog");
	  gtk_window_set_title(GTK_WINDOW(strategydlg.win), winname);
	}	  
      } else {
	printf("selecting %i team %i\n", id, team);

	// we need to show the dialog box for this robot if it is still around
	// otherwise we need to remake it
	if (robotdlg[team][id].win != NULL)
	  gtk_widget_show(robotdlg[team][id].win);
	else {
	  robotdlg[team][id].win = create_robotdlg();
	  char winname[256];
	  sprintf(winname, "Robot Dialog Team %i, ID %i", team, id);
	  gtk_window_set_title(GTK_WINDOW(robotdlg[team][id].win), winname);
	}
      }
      break;
    case 2: {
      if (field.SelectedRobot()) {
	net_gtactic c = { NET_GUI_TACTIC, id, "stop"};
	if (soccer_s.get_status() == Socket::Client)
	  soccer_s.send(&c, c.size());
      }
    } break;
    }
  }

  return FALSE;
}


gboolean on_darea_button_release_event(GtkWidget *widget, GdkEventButton  *event,
				       gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "mouse release button %i\n", event->button);
#endif

  // make sure we clear out the button selection
  selection_button = 0;

  // if we are over our screen then grab the focus
  if (widget == field.GetWidget())
    gtk_widget_grab_focus(widget);

  // reset teh driving speed for now
  if (0 && drive.driving) {
    drive.vcmd.z = 0;
    return true;
  }

  if (!selection_frozen) 
    return TRUE;

  // unfreeze whatever was frozen
  selection_frozen = false;
  switch (event->button) {
  case 1: // left button

    // move the ball or the robot around the field to the new position
    if (field.SelectedBall()) {
      client.MoveBall(field.field_x(event->x), field.field_y(event->y),
		      frame.ball.state.vx, frame.ball.state.vy);
    } else if (field.SelectedRobot()) {

      // shift modified turns it into a movement command
      if (event->state & GDK_SHIFT_MASK) {
	net_gtactic c = { NET_GUI_TACTIC, field.SelectedRobotID(), ""};
	
	int team = field.SelectedRobotTeam();
	int id = field.SelectedRobotID();
	
	sprintf(c.string, "position {G %f %f} %f", 
		field.field_x(event->x), field.field_y(event->y),
		frame.robots[team][id].state.theta);
	
	UpdateStatusBar(statusbar, "send to %s", c.string);
	if (soccer_s.get_status() == Socket::Client)
	  soccer_s.send(&c, c.size());
      } else {
	int team = field.SelectedRobotTeam();
	int id = field.SelectedRobotID();

	client.MoveRobot(team, id, field.field_x(event->x), field.field_y(event->y),
		  frame.robots[team][id].state.theta);
      }
    }    
    break;

  case 2: // middle button
    if (field.SelectedRobot()) {
      net_gtactic c = { NET_GUI_TACTIC, field.SelectedRobotID(), ""};

      int team = field.SelectedRobotTeam();
      int id = field.SelectedRobotID();
      
      sprintf(c.string, "position {G %f %f} %f", 
	      field.field_x(event->x), field.field_y(event->y),
	      frame.robots[team][id].state.theta);
      
      UpdateStatusBar(statusbar, "send to %s", c.string);
    
      if (soccer_s.get_status() == Socket::Client)
	soccer_s.send(&c, c.size());
    }

    break;

  case 3: // right button

    // move the ball or the robot around the field to the new position
    if (field.SelectedBall()) {
      double vx = field.field_x(event->x) - (double) frame.ball.state.x;
      double vy = field.field_y(event->y) - (double) frame.ball.state.y;

      client.MoveBall(frame.ball.state.x, frame.ball.state.y, vx, vy);
    } else if (field.SelectedRobot()) {
      if (event->state & GDK_SHIFT_MASK) {
	net_gtactic c = { NET_GUI_TACTIC, field.SelectedRobotID(), ""};

	int team = field.SelectedRobotTeam();
	int id = field.SelectedRobotID();
	
	sprintf(c.string, "position {G %f %f} %f", 
		frame.robots[team][id].state.x, frame.robots[team][id].state.y, 
		frame.robots[team][id].state.theta);
	
	UpdateStatusBar(statusbar, "send to %s", c.string);
    
	if (soccer_s.get_status() == Socket::Client)
	  soccer_s.send(&c, c.size());
      } else {
	int team = field.SelectedRobotTeam();
	int id = field.SelectedRobotID();

	client.MoveRobot(team, id, frame.robots[team][id].state.x, 
			 frame.robots[team][id].state.y,
			 frame.robots[team][id].state.theta);
      }
    }    
    break;

  case 4: // roller forward button
    break;
  case 5: // roller backward button
    break;
  }

  return FALSE;
}


/**************************** Toolbar Controls **************************/

void on_tactic_ctrl_changed(GtkEditable *editable,gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "on_tactic_ctrl_changed\n");
#endif
}



void on_debuglevel_changed(GtkEditable *editable, gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "on_debuglevel_changed\n");
#endif

  // depricated
  return;


  int new_level = 0;
  char *lvl = gtk_editable_get_chars(GTK_EDITABLE(editable), 0, -1);
  if (isdigit(lvl[0]))
    new_level = atoi(lvl);

  field.SetDebugLevel(new_level);

  /* a little user output */
  UpdateStatusBar(statusbar, "Debug Level set to %i", new_level);
}


void on_param_ctrl_activate(GtkEditable *editable, gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "on_param_ctrl_activate\n");
#endif

  /* a little user output */
  UpdateStatusBar(statusbar, "Parameters set to ");
}


void on_vcr_clicked(GtkButton *button, gpointer user_data)
{
  int key = GPOINTER_TO_INT(user_data);

#ifdef DEBUG
  fprintf(stderr, "on_vcr_clicked with key %i\n", key);
#endif

  // push all the dirty work onto the client
  client.RadioControl(key);
}

// clicked whenever the apply tactic or stop tactic buttons are clicked
void on_tactic_button_clicked(GtkButton *button, gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "on_tactic_button_clicked\n");
#endif

  // find the widgets if we haven't already
  if (param_ctrl_widget == NULL) {
    if ((param_ctrl_widget = lookup_widget(GTK_WIDGET(mainwin), "param_ctrl")) == NULL) {
      fprintf(stderr, "ERROR: Cannot find param_ctrl widget handle!!!\n");
      exit(1);
    }
  }
  if (tactic_ctrl_widget == NULL) {
    if ((tactic_ctrl_widget = lookup_widget(GTK_WIDGET(mainwin), "tactic_ctrl")) == NULL) {
      fprintf(stderr, "ERROR: Cannot find tactic_ctrl widget handle!!!\n");
      exit(1);
    }
  }

  // what button was it? - 1 is apply, 0 is stop
  if (GPOINTER_TO_INT(user_data)) {

    // work out what tactic and parameters we are passing
    char *param_str = gtk_entry_get_text(GTK_ENTRY(param_ctrl_widget));
    char *tactic_str = gtk_editable_get_chars(GTK_EDITABLE(tactic_ctrl_widget), 0, -1);
    
    // nothing to do
    if (strcmp(tactic_str, "None") == 0)
      return;
  
    // create the soccer packet
    int id = -1;
    if (field.SelectedRobot())
      id = field.SelectedRobotID();
    net_gtactic nt = {NET_GUI_TACTIC, id, ""};

    strncpy(nt.string, tactic_str, G_TACTIC_MAXLENGTH);
    if (*param_str != '\0') {
      strncat(nt.string, " ", G_TACTIC_MAXLENGTH);
      strncat(nt.string, param_str, G_TACTIC_MAXLENGTH);
    }

    // send it!
    if (soccer_s.get_status() == Socket::Client)
      soccer_s.send(&nt, nt.size());
  } else {

    // create a soccer packet that has NULL in it for all robots
    net_gtactic nt = {NET_GUI_TACTIC, -1, "NULL"};
    for (int i = 0; i < MAX_TEAM_ROBOTS; i++) {
      nt.robot = i;
      if (soccer_s.get_status() == Socket::Client)
	soccer_s.send(&nt, nt.size());
    }
  }
}

void on_gameinfo_button_clicked(GtkButton *button, gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "on_gameinfo_button_clicked\n");
#endif

  GtkWidget *w = lookup_widget(GTK_WIDGET(button), "gameinfo");
  if (GTK_WIDGET_VISIBLE(w))
    gtk_widget_hide(w);
  else
    gtk_widget_show(w);
}

void on_set_button_clicked(GtkButton *button, gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "on_set_button_clicked\n");
#endif

  static GtkWidget *w = NULL;
  if (w == NULL)
    w = lookup_widget(GTK_WIDGET(button), "SetCtrlBox");

  if (GTK_WIDGET_VISIBLE(w))
    gtk_widget_hide(w);
  else
    gtk_widget_show(w);
}



gboolean on_darea_key_press_event(GtkWidget *widget, GdkEventKey *event,
				  gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "on_key press\n");
#endif

  // check this is the window we should be using
  if (field.GetWidget() != widget)
    return FALSE; 

  //  if (!field.SelectedRobot())
  if (!drive.driving)
    return false;

#ifdef DEBUG
  fprintf(stderr, "driving t%d, robot %d key %c\n", ourteam, id, event->keyval);
#endif

  //  printf("key press...state: %i, val %x\n", event->state, event->keyval);


  switch (gdk_keyval_to_upper(event->keyval)) {
  case GDK_Left: drive.vcmd.y += SPEED_INCR; break;
  case GDK_Right: drive.vcmd.y -= SPEED_INCR; break;

  case GDK_Up: drive.vcmd.x += SPEED_INCR; break;
  case GDK_Down: drive.vcmd.x -= SPEED_INCR; break;

  case GDK_I: drive.vcmd.x += SPEED_INCR; break;
  case GDK_K: drive.vcmd.x -= SPEED_INCR; break;

  case GDK_L: drive.vcmd.y -= SPEED_INCR; break;
  case GDK_J: drive.vcmd.y += SPEED_INCR; break;

  case GDK_A: drive.vcmd.z += ROT_INCR; break;
  case GDK_S: drive.vcmd.z -= ROT_INCR; break;
  case GDK_F: 
    drive.kick = true;
    printf("kick!!!\n");
    break;
  case GDK_C: 
    drive.drib = !drive.drib; 
    if (drive.drib) printf("dribble, dribble, dribble...\n");
    break;
  case GDK_D:
  case GDK_space:
    drive.vcmd.x = drive.vcmd.y = drive.vcmd.z = 0.0;
    break;
  case GDK_Escape:
    drive.driving = false;

    
  }
  drive.vcmd.x = bound(drive.vcmd.x, -2000.0, 2000.0);
  drive.vcmd.y = bound(drive.vcmd.y, -2000.0, 2000.0);
  drive.vcmd.z = bound(drive.vcmd.z, -10.0, 10.0);

  // set the command
  client.SendCommand(frame.timestamp, drive.team, drive.id, 
		     drive.vcmd.x, drive.vcmd.y, drive.vcmd.z, 
		     drive.kick, drive.drib, 2);

#ifdef DEBUG
  printf("sending t %i id %i cmd (%f %f %f) k %i d %i p %i\n",
	 drive.team, drive.id, drive.vcmd.x, drive.vcmd.y, drive.vcmd.z, 
	 drive.kick, drive.drib, 2);
#endif


  return FALSE;
}

void on_SetCtrlBox_realize(GtkWidget *widget, gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "on_SetCtrlBox_realize\n");
#endif

  // read from teh config reader the values to set things to
  CR_SETUP(gui, BLUE_ENABLE, CR_INT);
  CR_SETUP(gui, BLUE_COVERS, CR_INT);
  CR_SETUP(gui, BLUE_ROBOTS, CR_INT);
  CR_SETUP(gui, YELLOW_ENABLE, CR_INT);
  CR_SETUP(gui, YELLOW_COVERS, CR_INT);
  CR_SETUP(gui, YELLOW_ROBOTS, CR_INT);

  // blue team first
  covers[TEAM_BLUE] = IVAR(BLUE_COVERS);
  enable[TEAM_BLUE] = IVAR(BLUE_ENABLE);

  fprintf(stderr, "Setting team blue: covers %i, enabled %i\n", 
	 IVAR(BLUE_COVERS), IVAR(BLUE_ENABLE));
  fprintf(stderr, "\trobots enabled : ");

  for (uint i = 0; i < VARSIZE(BLUE_ROBOTS); i++) {
    int id = VIVAR(BLUE_ROBOTS)[i];
    if ((id >= 0) && (id < MAX_ROBOT_ID))
      enable_ids[TEAM_BLUE][id] = true;
    else
      enable_ids[TEAM_BLUE][id] = false;

    fprintf(stderr, "%i ", VIVAR(BLUE_ROBOTS)[i]);
  }
  fprintf(stderr, "\n");

  // yellow team
  covers[TEAM_YELLOW] = IVAR(YELLOW_COVERS);
  enable[TEAM_YELLOW] = IVAR(YELLOW_ENABLE);
  fprintf(stderr, "Setting team yellow: covers %i, enabled %i\n", 
	 IVAR(YELLOW_COVERS), IVAR(YELLOW_ENABLE));
  fprintf(stderr, "\trobots enabled : ");

  for (uint i = 0; i < VARSIZE(YELLOW_ROBOTS); i++) {
    int id = VIVAR(YELLOW_ROBOTS)[i];
    if ((id >= 0) && (id < MAX_ROBOT_ID))
      enable_ids[TEAM_YELLOW][id] = true;
    else
      enable_ids[TEAM_YELLOW][id] = false;

    fprintf(stderr, "%i ", VIVAR(YELLOW_ROBOTS)[i]);
  }
  fprintf(stderr, "\n");

  // now set the buttons
  for (int t = 0; t < NUM_TEAMS; t++) {
    char buff[256];

    sprintf(buff, "enable_%s", (t ? "yellow" : "blue"));
    enable_button[t] = GTK_TOGGLE_BUTTON(lookup_widget(mainwin, buff));
    sprintf(buff, "enable_%s_covers", (t ? "yellow" : "blue"));
    cover_button[t] = GTK_TOGGLE_BUTTON(lookup_widget(mainwin, buff));

    gtk_toggle_button_set_active(enable_button[t], enable[t]);
    gtk_toggle_button_set_active(cover_button[t], covers[t]);
    
    for (int i = 0; i < MAX_ROBOT_ID_OURS; i++) {
      if (t == TEAM_BLUE)
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(wblue[i]), enable_ids[t][i]);
      else
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(wyellow[i]), enable_ids[t][i]);
    }
  }
}

void on_enable_set_clicked(GtkButton *button, gpointer user_data)
{
#ifdef DEBUG
  printf("enabling\n");
#endif
  GtkToggleButton *wb = NULL;

  for (int t = 0; t < NUM_TEAMS; t++) {
    enable[t] = gtk_toggle_button_get_active(enable_button[t]);
    covers[t] = gtk_toggle_button_get_active(cover_button[t]);

    // we don't have MAX_ROBOT_ID checkboxes so use wy to see if it is real
    if (enable[t]) {
      for (int i = 0; i < MAX_ROBOT_ID_OURS; i++) {
	if (t == TEAM_BLUE)
	  wb = GTK_TOGGLE_BUTTON(wblue[i]);
	else
	  wb = GTK_TOGGLE_BUTTON(wyellow[i]);
	enable_ids[t][i] = gtk_toggle_button_get_active(wb);
      }
    }
  }

  // go do the grunt work
  do_vision_config();
}


void on_general_help_activate(GtkMenuItem *menuitem, gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "Exec browser program here\n");
#endif

  UpdateStatusBar(statusbar, "Execing help file viewer. Please wait...");
  fprintf(stderr, "Starting Help\n");
  if (fork() == 0) {
    fprintf(stderr, "execing runhelp.sh\n");
    execv("runhelp.sh", NULL);
    fprintf(stderr, "trying bin/runhelp.sh\n");
    execv("bin/runhelp.sh", NULL);
    fprintf(stderr, "trying ../bin/runhelp.sh\n");
    execv("../bin/runhelp.sh", NULL);

    fprintf(stderr, "Giving up...where is runhelp.sh?\n");

    // if we got here we can't find it...how to exit safely?
    exit(1);
  }
}


void on_about_activate(GtkMenuItem *menuitem, gpointer user_data)
{
  gtk_widget_show(aboutdlg);
}


void on_aboutdlg_ok(GtkButton *button, gpointer user_data)
{
  gtk_widget_hide(aboutdlg);
}


/************************************ REFBOX DIALOG **************************/

void on_refbox_button_clicked(GtkButton *button, gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "REFBOX button click: %i\n", GPOINTER_TO_INT(user_data));
#endif
    
#ifdef DEBUG
  fprintf(stderr, "REFBOX toggle\n");
#endif

  char refstate = (char) GPOINTER_TO_INT(user_data);
  client.SendRef(refstate);

  fprintf(stderr, "sending to vision %c\n", refstate);
  UpdateStatusBar(statusbar, "sending to vision %c\n", refstate);
}

void on_refbox_toggled(GtkToggleButton *togglebutton, gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "REFBOX toggle\n");
#endif

  if (!gtk_toggle_button_get_active(togglebutton)) return;

  char refstate = (char) GPOINTER_TO_INT(user_data);
  client.SendRef(refstate);

  fprintf(stderr, "sending to vision %c\n", refstate);
  UpdateStatusBar(statusbar, "sending to vision %c\n", refstate);
}


gboolean on_refbox_configure_event(GtkWidget *widget, GdkEventConfigure *event,
				   gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "REFBOX configure\n");
#endif

  // find the radio button widgets
  rbox_restart = lookup_widget(widget, "RBRestart");
  rbox_kickoff[TEAM_BLUE] = lookup_widget(widget, "RBKickoffblue");
  rbox_kickoff[TEAM_YELLOW] = lookup_widget(widget, "RBKickoffyellow");
  rbox_penalty[TEAM_BLUE] = lookup_widget(widget, "RBPenaltyblue");
  rbox_penalty[TEAM_YELLOW] = lookup_widget(widget, "RBPenaltyyellow");
  rbox_freekick[TEAM_BLUE] = lookup_widget(widget, "RBFreekickblue");
  rbox_freekick[TEAM_YELLOW] = lookup_widget(widget, "RBFreekickyellow");
  
  return FALSE;
}



void on_refbox_ctrl_clicked(GtkButton *button, gpointer user_data)
{
#ifdef DEBUG
  fprintf(stderr, "REFBOX ctrl button clicked\n");
#endif

  if (refboxwin == NULL)
    refboxwin = create_refbox();

  // toggle the hide state of the refbox
  if (GTK_WIDGET_VISIBLE(refboxwin))
    gtk_widget_hide(refboxwin);
  else
    gtk_widget_show(refboxwin);
}


void on_refbox_destroy(GtkObject *object, gpointer user_data)
{
  printf("destroyed...\n");
  refboxwin = NULL;
}



/************************************ Robot DIALOG **************************/

void on_robotdlg_close_clicked(GtkButton *button, gpointer user_data)
{

  GtkWidget *w = gtk_widget_get_toplevel(GTK_WIDGET(button));

  fprintf(stderr, "window name %s\n", gtk_widget_get_name(w));

  //  w = gtk_widget_get_parent_window(button->window->widget));

  //  w = gtk_widget_get_toplevel(button->window->widget);

  gtk_widget_hide(w);
}


void on_robotdlg_destroy(GtkObject *object, gpointer user_data)
{
  // set the pointer to NULL in the array list
  for (int t = 0; t < NUM_TEAMS; t++) {
    for (int i  = 0; i < MAX_TEAM_ROBOTS; i++) {
      if (GTK_WIDGET(object) == robotdlg[t][i].win) {
	robotdlg[t][i].win = NULL;
	return;
      }
    }
  }
}



void on_drive_ctrl_clicked(GtkButton *button, gpointer user_data)
{
  fprintf(stderr, "Drive ctrl clicked\n");

  if (!drive.driving) {
    if (field.SelectedRobot()) {
      drive.driving = true;
      drive.vcmd.set(0, 0, 0);
      drive.team = field.SelectedRobotTeam();
      drive.id = field.SelectedRobotID();
      drive.kick = false;
      drive.drib = false;

      drive.oldx = field.screen_x(0.0);

      drive.callbackid = gtk_timeout_add(50, (GtkFunction) DriveCallback, NULL);

      //      if (gdk_pointer_grab(mainwin->window, true, (GdkEventMask) 0, 
      //			   field.GetWidget()->window, NULL, GDK_CURRENT_TIME))
      //	fprintf(stderr, "Canont grab pointer\n");

      UpdateStatusBar(statusbar, "Driving mode enabled");
    } else {
      UpdateStatusBar(statusbar, "Driving mode not enabled: select robot first");
    }
  } else {
    drive.driving = false;
  }

  gtk_window_set_title(GTK_WINDOW(mainwin), 
		       (drive.driving ? "CMDragons 02: Driving Mode" : "CMDragons\'02"));
}


gint DriveCallback(void)
{
  //  printf("driving...\n");
  if (drive.driving == false) {
    gtk_timeout_remove(drive.callbackid);
    drive.callbackid = -1;
    UpdateStatusBar(statusbar, "Driving mode disabled");
    if (gdk_pointer_is_grabbed())
      gdk_pointer_ungrab(GDK_CURRENT_TIME);

    drive.vcmd.set(0, 0, 0);
    client.SendCommand(frame.timestamp, drive.team, drive.id, drive.vcmd,
		       drive.kick, drive.drib, 2);
  }

  drive.vcmd.x *= drive.gain.x;
  drive.vcmd.y *= drive.gain.y;

  //  drive.oldx += (drive.newx - drive.oldx) * 0.3;
  //  drive.vcmd.z *= 0.7;

  if (drive.kick)
    drive.kick = false;

  client.SendCommand(frame.timestamp, drive.team, drive.id, drive.vcmd,
		     drive.kick, drive.drib, 2);

  return 1;
}


/**************************8 Graphics function **************************/

void on_ball_graph_clicked(GtkButton *button, gpointer user_data)
{
  switch (GPOINTER_TO_INT(user_data)) {
  case 0:
    graphdlgs.ball_position.create(modeller.ball_position.getData(), 
    			      modeller.ball_position.getXSize(),
    			      modeller.ball_position.getYSize());
    break;
  case 1:
    graphdlgs.ball_velocity.create(modeller.ball_speed.getData(), 
			      modeller.ball_speed.getSize());
    break;
  default:
    fprintf(stderr, "Unknown graph...\n");
    UpdateStatusBar(statusbar, "Unknown graph...");
  }
}


void on_blue_graph_clicked(GtkButton *button, gpointer user_data)
{
  fprintf(stderr, "graph blue cliked...\n");

  switch (GPOINTER_TO_INT(user_data)) {
  case 0:
    graphdlgs.position[0].create(modeller.position[0].getData(), 
    			      modeller.position[0].getXSize(),
    			      modeller.position[0].getYSize());
    break;
  case 1:
    graphdlgs.velocity[0].create(modeller.speed[0].getData(), 
			      modeller.speed[0].getSize());
    break;
  case 2:
    graphdlgs.acceleration[0].create(modeller.acceleration[0].getData(), 
			      modeller.acceleration[0].getSize());
    break;
  case 3 :
    graphdlgs.near_ball[0].create(modeller.robots_near_ball[0].getData(), 
			      modeller.robots_near_ball[0].getSize());
    break;
  case 4:
    graphdlgs.shoot[0].create(modeller.shots_on_goal[0].getData(), 
    			      modeller.shots_on_goal[0].getXSize(),
    			      modeller.shots_on_goal[0].getYSize());
    break;
  default:
    fprintf(stderr, "Unknown graph...\n");
    UpdateStatusBar(statusbar, "Unknown graph...");
  }
}


void on_yellow_graph_clicked(GtkButton *button, gpointer user_data)
{
  fprintf(stderr, "graph yellow cliked...\n");

  switch (GPOINTER_TO_INT(user_data)) {
  case 0:
    graphdlgs.position[1].create(modeller.position[1].getData(), 
    			      modeller.position[1].getXSize(),
    			      modeller.position[1].getYSize());
    break;
  case 1:
    graphdlgs.velocity[1].create(modeller.speed[1].getData(), 
			      modeller.speed[1].getSize());
    break;
  case 2:
    graphdlgs.acceleration[1].create(modeller.acceleration[1].getData(), 
			      modeller.acceleration[1].getSize());
    break;
  case 3 :
    graphdlgs.near_ball[1].create(modeller.robots_near_ball[1].getData(), 
			      modeller.robots_near_ball[1].getSize());
    break;
  case 4:
    graphdlgs.shoot[1].create(modeller.shots_on_goal[1].getData(), 
    			      modeller.shots_on_goal[1].getXSize(),
    			      modeller.shots_on_goal[1].getYSize());
    break;
  default:
    fprintf(stderr, "Unknown graph...\n");
    UpdateStatusBar(statusbar, "Unknown graph...");
  }
}


void on_graph_ctrl_clicked(GtkButton *button, gpointer user_data)
{
  fprintf(stderr, "graph control cliked...\n");

  static GtkWidget *w = NULL;
  if (w == NULL)
    w = lookup_widget(GTK_WIDGET(button), "GraphCtrl");

  if (GTK_WIDGET_VISIBLE(w))
    gtk_widget_hide(w);
  else
    gtk_widget_show(w);

}


