/*
 * TITLE:        main.h
 *
 * PURPOSE:      This file exports the globals stored in teh main program
 *               
 * WRITTEN BY:   Brett Browning, Michael Bowling, James R Bruce
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

#ifndef __MAIN_H__
#define __MAIN_H__

#include <gtk/gtk.h>

#include <socket.h>
#include <constants.h>

#include "robotdlg.h"
#include "../utils/configreader.h"
#include "../reality/client/client.h"
#include "graphdlg.h"
#include "soccer/modeller.h"

/**************************** TYPES ******************************************/
struct GraphDlgs {
  GraphDlg2D ball_position;
  GraphDlg1D ball_velocity;
  GraphDlg2D position[NUM_TEAMS];
  GraphDlg1D velocity[NUM_TEAMS], acceleration[NUM_TEAMS];
  GraphDlg1D near_ball[NUM_TEAMS];
  GraphDlg2D shoot[NUM_TEAMS];
};

/**************************** GLOBALS ****************************************/
extern GtkWidget *statusbar;
extern GtkWidget *mainwin, *aboutdlg;
extern GtkWidget *refboxwin;
extern RobotDialog robotdlg[NUM_TEAMS][MAX_TEAM_ROBOTS];
extern StrategyDialog strategydlg;

extern Client client;

//extern Socket vision_s;
//extern Socket radio_s;
extern Socket soccer_s;

extern Field field;
extern net_vframe frame;
extern bool selection_frozen;
extern int selection_button;

extern char config_file[];
extern ConfigReader configreader;

extern bool enable_ids[NUM_TEAMS][MAX_ROBOT_ID];
extern int map_id2index[NUM_TEAMS][MAX_ROBOT_ID];
extern int map_index2id[NUM_TEAMS][MAX_TEAM_ROBOTS];
extern bool covers[NUM_TEAMS], enable[NUM_TEAMS];


extern GtkWidget *wyellow[MAX_ROBOTS];
extern GtkWidget *wblue[MAX_ROBOTS];

extern GraphDlgs graphdlgs;

/**************************** FUNCTIONS **************************************/
void do_disconnect(void);
gint do_connect(void);
void do_vision_config(void);

gint do_radio_connect(void);
gint do_radio_disconnect(void);
void do_radio_send(uchar r, double vx, double vy, double va, bool k = false, bool drib = false);
void do_radio_control(char cmd);
void UpdateRobotDlg(net_vframe &f, int t, int i);

#endif /* __MAIN_H__ */
