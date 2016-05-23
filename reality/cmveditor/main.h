/*
 * TITLE:       main.h
 *
 * PURPOSE:     This file exports the main globals for use in callbacks.cc
 *
 * WRITTEN BY:  Brett Browning
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

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <gtk/gtk.h>
#include <gtk/gtkwidget.h>

#include "interface.h"
#include "support.h"
#include <reality/cmvision/cmvision.h>

#include "constants.h"

#include "draw.h"


/************************************* TYPES *************************************/

/************************************* GLOBALS ***********************************/
extern GtkWidget *statusbar, *cmapstatusbar;
extern GtkWidget *colorfile, *paramsfile;
extern GtkWidget *cmveditor, *colormapwin, *display;

extern CMVision::color_class_state color[MAX_COLORS];
extern int num_colors;
extern DrawHistogram dhistogram;
extern DrawRGB drawrgb;
extern CParamDraw dparam;

extern bool grabbing, grabone, showsegmentation;

extern char configdir[];

/************************************* PROTOTYPES ********************************/


// temp for now
#include "colors.h"
rgb YuvToRgb(uyvy p);
rgb YuvToRgb(yuv p);

yuv GetLocation(int x,int y, rgb &c);
void Quit(void);


#endif /* __MAIN_H__ */
