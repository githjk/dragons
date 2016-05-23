/*
 * TITLE:        hooks.h
 *
 * PURPOSE:      This class wraps the hooks functionality of Glib
 *               
 * WRITTEN BY:   Michael Bowling, Brett Browning, some code ported from small size 2001 GUI
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

#ifndef __HOOKS_H__
#define __HOOKS_H__


#include <gtk/gtk.h>


typedef void (*HookFunc)(void *netdata, void *data);

class Hooks {
protected:
  GHookList hooks;

public:
  Hooks(void);
  ~Hooks(void);

  int AddHook(HookFunc hf, void *data);
  void RemoveHook(int id);

  void Marshall(gpointer data);

};


#endif /* __HOOKS_H__ */

