/*
 * TITLE:        hooks.cc
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

#include "hooks.h"
#include <gtk/gtk.h>

 
static void Marshaller(GHook *hook, gpointer data)
{
  ((HookFunc) (hook->func))(data, hook->data);
}

Hooks::Hooks(void)
{
  g_hook_list_init(&hooks, sizeof(GHook));
}

Hooks::~Hooks(void)
{
}

int Hooks::AddHook(HookFunc hf, void *data)
{
  GHook *hook = g_hook_alloc(&hooks);

  hook->data = data;
  hook->func = (void *) hf;
  g_hook_prepend(&hooks, hook);
  return (hook->hook_id);
}


void Hooks::RemoveHook(int id)
{
  g_hook_destroy(&hooks, id);
}

void Hooks::Marshall(gpointer data)
{
  g_hook_list_marshal(&hooks, FALSE, Marshaller, &data);
}




