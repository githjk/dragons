/*
 * TITLE:       threshold.h
 *
 * PURPOSE:     This file encapsulates the threshold class. It wraps the functions written
 *              by Jim Bruce.
 *
 * WRITTEN BY:  Brett Browning, modified from {James Bruce} Small Size code 2001
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

#ifndef __THRESHOLD_H__
#define __THRESHOLD_H__

#include <gtk/gtk.h>
#include "array3d.h"


typedef array3d<unsigned char,0> tmap_t;


class Threshold {
public:
  tmap_t tmap;
public:
  Threshold(void) {
    ResetMap();
  };

  ~Threshold(void) {
  };

  void ResetMap(void);

  bool Load(char *filename);
  bool Save(char *filename);

  void Draw(GtkWidget *w, int z,int zoom);
  void DrawLine(int x1,int y1,int z1,
	      int x2,int y2,int z2,
		int color);
  int Paint(int x,int y,int z,
	    int src_color,int dest_color);
  int Paint(int x,int y,int z,
	    int dest_color);



};

#endif /* __THRESHOLD_H__ */


