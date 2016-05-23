/*
 * TITLE:        gtkutils.cc
 *
 * PURPOSE:      This file contains some useful utility functions for working
 *               with GTK.
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

#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#include "gtkutils.h"

//#define DEBUG


/**************************** TYPES ***********************************/

/* max status bar message */
#define MAX_STATUS_BAR     256


/**************************** CODE ************************************/


/*
 * StatusBarUpdate -
 *
 * this funciton updates the status bar with teh formatted string
 */
void UpdateStatusBar(GtkWidget *sb, const char *fmt, ...)
{
  char buffer[MAX_STATUS_BAR];

  if (sb == NULL) {
    fprintf(stderr, "GtkUtils.cc: ERROR: Status bar is nul!!!\n");
    return;
  }

  va_list ap;
  va_start(ap, fmt);
  vsprintf(buffer, fmt, ap);
  gtk_statusbar_pop(GTK_STATUSBAR(sb), 1);
  gtk_statusbar_push(GTK_STATUSBAR(sb), 1, buffer);

#ifdef DEBUG
  strcat(buffer, "\n");
  fprintf(stderr, buffer);
#endif
}


/*
 * GtkSetText -
 *
 * This function wraps the painful set text operation required 
 * by GTK for text boxes 
 */
void GtkSetText(GtkWidget *wid, char *string)
{
  if (wid == NULL) {
    fprintf(stderr, "GtkUtils.cc: ERROR: text widget is nul!!!\n");
    return;
  }

  GtkStyle *style = gtk_widget_get_style(wid);
  
  gtk_text_freeze(GTK_TEXT(wid));
  gtk_text_backward_delete(GTK_TEXT(wid), 
			   gtk_text_get_length(GTK_TEXT(wid)));
  gtk_text_insert(GTK_TEXT(wid), style->font, &style->text[GTK_STATE_NORMAL], 
		  &style->base[GTK_STATE_NORMAL], string, strlen(string));
  gtk_text_thaw(GTK_TEXT(wid));
}
