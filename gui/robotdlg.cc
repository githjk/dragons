/*
 * TITLE:        robotdlg.cc
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

#include <stdio.h>
#include <string.h>
#include "../soccer/net_gui.h"
#include <deque>
#include "robotdlg.h"
#include "gtkutils.h"

#include "field.h"
#include "main.h"

// wet the main window widget pointer 
void RobotDialog::Set(GtkWidget *main, GtkWidget *textbox)
{
  initialized = true;
  win = main;
  wtextbox = GTK_TEXT(textbox);
}

void RobotDialog::Add(net_gdebug &debug) 
{
  if (debug.msgtype != NET_GUI_DEBUG_MSG)
    return;

  new_messages.push_back(debug);

  // pop anything with an old timestamp
  double t = debug.timestamp;
  
  while (!new_messages.empty() && (new_messages[0].timestamp < t))
    new_messages.pop_front();

  // do we need to worry about too many messages?
  //  while (new_messages.size() > 30)
  //    new_messages.pop_front();
}

void RobotDialog::Add(char* statmsg, double tmestmp)
{
  net_gdebug debug;

  debug.msgtype = NET_GUI_DEBUG_MSG;
  
  int len = strlen(statmsg);
  
  debug.timestamp = tmestmp;

  for(int i=0; i<len; i += G_MSG_MAXLENGTH) {
    strncpy(debug.info.msg.msg, statmsg + i, G_MSG_MAXLENGTH);
    debug.info.msg.msg[G_MSG_MAXLENGTH - 1] = '\0';

    Add(debug);
  }
}

void RobotDialog::Update(void)
{
  display_messages.clear();
  display_messages = new_messages;
  if (initialized)
    Draw();
  new_messages.clear();
}

void RobotDialog::Draw(void) 
{
#ifdef DEBUG
  fprintf(stderr, "Drwaing...\n");
  GtkSetText(wtextbox, "testing...\n");
#endif

  gtk_text_freeze(wtextbox);

  gtk_text_backward_delete(wtextbox, 
			   gtk_text_get_length(wtextbox));

  for (uint i = 0; i < display_messages.size(); i++) {
    gulong cl = field.GetDebugColor(display_messages[i].level);
    GdkColor c;

    c.red = bound((cl & 0xFF0000) >> 8, 
		  (unsigned long) 0, (unsigned long) 50000);
    c.green = bound((cl & 0x00FF00),
		  (unsigned long) 0, (unsigned long) 50000);
    c.blue = bound((cl & 0x0000FF) << 8,
		  (unsigned long) 0, (unsigned long) 50000);

    gtk_text_insert(wtextbox, NULL, &c, NULL,
		    display_messages[i].info.msg.msg, -1);
  }

  gtk_text_thaw(wtextbox);

}

void RobotDialog::Clear(void) 
{
  display_messages.clear();
}

/********************** STRATEGY DIALGO CLASS *********************/

// wet the main window widget pointer 
void StrategyDialog::Set(GtkWidget *main, GtkWidget *textbox, uint _char_limit)
{
  initialized = true;
  win = main;
  wtextbox = GTK_TEXT(textbox);
  char_limit = _char_limit;
}

void StrategyDialog::Add(net_gdebug &debug) 
{
  if (debug.msgtype != NET_GUI_DEBUG_MSG)
    return;

  new_messages.push_back(debug);

  // pop anything with an old timestamp
  double t = debug.timestamp;
  
  while (!new_messages.empty() && (new_messages[0].timestamp < t))
    new_messages.pop_front();

  // do we need to worry about too many messages?
  //  while (new_messages.size() > 30)
  //    new_messages.pop_front();
}

void StrategyDialog::Add(char* statmsg, double tmestmp)
{
  net_gdebug debug;

  debug.msgtype = NET_GUI_DEBUG_MSG;
  
  int len = strlen(statmsg);
  
  debug.timestamp = tmestmp;

  for(int i=0; i<len; i += G_MSG_MAXLENGTH) {
    strncpy(debug.info.msg.msg, statmsg + i, G_MSG_MAXLENGTH);
    debug.info.msg.msg[G_MSG_MAXLENGTH - 1] = '\0';

    Add(debug);
  }
}

void StrategyDialog::Update(void)
{
  display_messages.clear();
  display_messages = new_messages;
  if (initialized)
    Draw();
  new_messages.clear();
}

void StrategyDialog::Draw(void) 
{
#ifdef DEBUG
  fprintf(stderr, "Drwaing...\n");
  GtkSetText(wtextbox, "testing...\n");
#endif

  if (display_messages.size() == 0) return;

  gtk_text_freeze(wtextbox);

  //  gtk_text_backward_delete(GTK_TEXT(wtextbox), 
  //			   gtk_text_get_length(GTK_TEXT(wtextbox)));

  // figuire out if we are at the end

  // bool atend = ((int) gtk_text_get_length(wtextbox) <=
  // (int) gtk_editable_get_position(GTK_EDITABLE(wtextbox)) + 10);

  //  printf("%i %i %i\n", (int) gtk_text_get_length(wtextbox),
  //	 (int) gtk_editable_get_position(GTK_EDITABLE(wtextbox)),
  //	 atend);

  for (uint i = 0; i < display_messages.size(); i++) {
    gulong cl = field.GetDebugColor(display_messages[i].level);
    GdkColor c;

    c.red = bound((cl & 0xFF0000) >> 8, 
		  (unsigned long) 0, (unsigned long) 50000);
    c.green = bound((cl & 0x00FF00),
		  (unsigned long) 0, (unsigned long) 50000);
    c.blue = bound((cl & 0x0000FF) << 8,
		  (unsigned long) 0, (unsigned long) 50000);

    gtk_text_insert(wtextbox, NULL, &c, NULL,
		    display_messages[i].info.msg.msg, -1);
  }

  // if we are over our char limit then kill it off
  if (gtk_text_get_length(wtextbox) > char_limit) {
      gtk_text_set_point(wtextbox, gtk_text_get_length(wtextbox) - char_limit);
      gtk_text_backward_delete(wtextbox, gtk_text_get_length(wtextbox) - char_limit);
      gtk_text_set_point(wtextbox, gtk_text_get_length(wtextbox));
  }

  // make it visible again
  gtk_text_thaw(wtextbox);

  //if (!atend)
  //   gtk_editable_set_position(GTK_EDITABLE(wtextbox), gtk_text_get_length(wtextbox));
}

void StrategyDialog::Clear(void) 
{
  display_messages.clear();
}

