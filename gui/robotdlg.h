/*
 * TITLE:        robotdlg.h
 *
 * PURPOSE:      This fiel wraps the robotdlg class that takes care of
 *               managing the text list and display status of the robot dialog boxes
 *               
 * WRITTEN BY:   Brett Browning, some code ported from small size 2001 GUI
 *               written by Michael Bowling
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

#ifndef __ROBOTDLG_H__
#define __ROBOTDLG_H__

#include <stdio.h>
#include <string.h>
#include "../soccer/net_gui.h"
#include <deque>
#include <gtk/gtk.h>

//#define DEBUG

#define MAX_STRING      2056

// text box class for wrapping string chunks for display
class RobotDialog {
private:
  GtkText *wtextbox;
  deque<net_gdebug> display_messages, new_messages;

  char buffer[MAX_STRING];
  bool initialized;

public:
  GtkWidget *win;

  RobotDialog(void) {
    initialized = false;
    win = NULL;
    wtextbox = NULL;
  }

  // wet the main window widget pointer 
  void Set(GtkWidget *main, GtkWidget *textbox);

  // update and draw at the end of the frame
  void Update(void);

  void Add(net_gdebug &debug);
  void Add(char* statmsg, double tmestmp);
  void Draw(void);

  // erase everything
  void Clear(void);
};


// text box class for wrapping string chunks for display
class StrategyDialog {
private:
  GtkText *wtextbox;
  deque<net_gdebug> display_messages, new_messages;

  char buffer[MAX_STRING];
  bool initialized;

  uint nr_chars;
  uint char_limit;

public:
  GtkWidget *win;

  StrategyDialog(void) {
    initialized = false;
    win = NULL;
    wtextbox = NULL;
    nr_chars = 0;
    char_limit = 0;
  }

  // wet the main window widget pointer 
  void Set(GtkWidget *main, GtkWidget *textbox, uint _char_limit = 4096);

  // update and draw at the end of the frame
  void Update(void);

  void Add(net_gdebug &debug);
  void Add(char* statmsg, double tmestmp);
  void Draw(void);

  // erase everything
  void Clear(void);
};


#endif

