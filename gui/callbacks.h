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

#include <gtk/gtk.h>

#include "reality/net_radio.h"

/* ints for game control */
#define STEPBACK_BUTTON       -1
#define PAUSE_BUTTON          0
#define STEPFORWARD_BUTTON    1
#define PLAY_BUTTON           2


void
OnFileActivate                         (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_FileConnect_activate                (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_Refresh_activate                    (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_Trails_activate                     (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_ID_activate                         (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_OpponentID_activate                 (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

gboolean
on_darea_configure_event               (GtkWidget       *widget,
                                        GdkEventConfigure *event,
                                        gpointer         user_data);

gboolean
on_darea_expose_event                  (GtkWidget       *widget,
                                        GdkEventExpose  *event,
                                        gpointer         user_data);

gboolean
on_debuglevel_configure_event          (GtkWidget       *widget,
                                        GdkEventConfigure *event,
                                        gpointer         user_data);

void
on_debuglevel_realize                  (GtkWidget       *widget,
                                        gpointer         user_data);

void
on_tacticcombo_set_focus_child         (GtkContainer    *container,
                                        GtkWidget       *widget,
                                        gpointer         user_data);

void
on_tactic_ctrl_activate                (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_tactic_ctrl_changed                 (GtkEditable     *editable,
                                        gpointer         user_data);

gboolean
on_tactic_ctrl_enter_notify_event      (GtkWidget       *widget,
                                        GdkEventCrossing *event,
                                        gpointer         user_data);

gboolean
on_tactic_ctrl_selection_notify_event  (GtkWidget       *widget,
                                        GdkEventSelection *event,
                                        gpointer         user_data);

void
on_debuglevel_changed                  (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_param_ctrl_activate                 (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_param_ctrl_changed                  (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_vcr_clicked                         (GtkButton       *button,
                                        gpointer         user_data);


void
on_gameinfo_button_clicked             (GtkButton       *button,
                                        gpointer         user_data);

void
on_darea_realize                       (GtkWidget       *widget,
                                        gpointer         user_data);

gboolean
on_darea_motion_notify_event           (GtkWidget       *widget,
                                        GdkEventMotion  *event,
                                        gpointer         user_data);

gboolean
on_darea_button_press_event            (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

gboolean
on_darea_button_release_event          (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

void
on_set_button_clicked                  (GtkButton       *button,
                                        gpointer         user_data);

gboolean
on_darea_key_press_event               (GtkWidget       *widget,
                                        GdkEventKey     *event,
                                        gpointer         user_data);


void
on_vcr_clicked                         (GtkButton       *button,
                                        gpointer         user_data);

void
on_blue_clicked                        (GtkButton       *button,
                                        gpointer         user_data);

void
on_enable_covers_clicked               (GtkButton       *button,
                                        gpointer         user_data);

void
on_yellow_clicked                      (GtkButton       *button,
                                        gpointer         user_data);

void
on_enable_set_clicked                  (GtkButton       *button,
                                        gpointer         user_data);

void
on_tactic_button_clicked               (GtkButton       *button,
                                        gpointer         user_data);

void
on_tactic_button_clicked               (GtkButton       *button,
                                        gpointer         user_data);

void
on_general_help_activate               (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_about_activate                      (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_aboutdlg_ok                         (GtkButton       *button,
                                        gpointer         user_data);

void
on_refbox_button_clicked               (GtkButton       *button,
                                        gpointer         user_data);

gboolean
on_refbox_configure_event              (GtkWidget       *widget,
                                        GdkEventConfigure *event,
                                        gpointer         user_data);

void
on_refbox_toggled                      (GtkToggleButton *togglebutton,
                                        gpointer         user_data);


void
on_refbox_ctrl_clicked                 (GtkButton       *button,
                                        gpointer         user_data);

void
on_robotdlg_close_clicked              (GtkButton       *button,
                                        gpointer         user_data);

void
on_robotdlg_destroy                    (GtkObject       *object,
                                        gpointer         user_data);

void
on_SetCtrlBox_realize                  (GtkWidget       *widget,
                                        gpointer         user_data);


void
on_drive_ctrl_clicked                  (GtkButton       *button,
                                        gpointer         user_data);

void
OnDebugToolBarClicked                  (GtkToggleButton *togglebutton,
                                        gpointer         user_data);

void
OnDebugToolBarClicked                  (GtkToggleButton *togglebutton,
                                        gpointer         user_data);

void
OnDebugToolBarClicked                  (GtkToggleButton *togglebutton,
                                        gpointer         user_data);

void
OnDebugToolbarRealize                  (GtkWidget       *widget,
                                        gpointer         user_data);

gboolean
on_refbox_destroy_event                (GtkWidget       *widget,
                                        GdkEvent        *event,
                                        gpointer         user_data);

void
on_refbox_destroy                      (GtkObject       *object,
                                        gpointer         user_data);

void
on_graph_area_realize                  (GtkWidget       *widget,
                                        gpointer         user_data);

gboolean
on_graph_area_expose                   (GtkWidget       *widget,
                                        GdkEventExpose  *event,
                                        gpointer         user_data);

void
on_graph_update_clicked                (GtkButton       *button,
                                        gpointer         user_data);

void
on_graph_close_clicked                 (GtkButton       *button,
                                        gpointer         user_data);

void
on_graph_destroy                       (GtkObject       *object,
                                        gpointer         user_data);

void
on_show_graph_clicked                  (GtkButton       *button,
                                        gpointer         user_data);

void
on_blue_graph_clicked                  (GtkButton       *button,
                                        gpointer         user_data);

void
on_blue_graph_clicked                  (GtkButton       *button,
                                        gpointer         user_data);

void
on_yellow_graph_clicked                (GtkButton       *button,
                                        gpointer         user_data);

void
on_graph_ctrl_clicked                  (GtkButton       *button,
                                        gpointer         user_data);

void
on_ball_graph_clicked                  (GtkButton       *button,
                                        gpointer         user_data);

void
on_vcr_clicked                         (GtkButton       *button,
                                        gpointer         user_data);

void
on_vcr_clicked                         (GtkButton       *button,
                                        gpointer         user_data);
