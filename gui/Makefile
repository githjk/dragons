# /* LICENSE:
#   =========================================================================
#     CMDragons'02 RoboCup F180 Source Code Release
#   -------------------------------------------------------------------------
#     Copyright (C) 2002 Manuela Veloso, Brett Browning, Mike Bowling,
#                        James Bruce; {mmv, brettb, mhb, jbruce}@cs.cmu.edu
#     School of Computer Science, Carnegie Mellon University
#   -------------------------------------------------------------------------
#     This software is distributed under the GNU General Public License,
#     version 2.  If you do not have a copy of this licence, visit
#     www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
#     Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
#     in the hope that it will be useful, but WITHOUT ANY WARRANTY,
#     including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#   ------------------------------------------------------------------------- */

TOPDIR=..

include $(TOPDIR)/Makefile.common

TARGET = gui

SRCS= main.c callbacks.cc gtkutils.cc interface.cc hooks.cc support.cc 
SRCS+= field.cc graphdlg.cc ../utils/stats.cc robotdlg.cc 
SRCS+= ../reality/client/client.cc

# temp:
SRCS+= ../soccer/modeller.cc

#OBJS= main.o $(SRCS:.cc=.o)
OBJS= $(SRCS:.cc=.o) ../utils/libutils.a

CFLAGS += -g `gtk-config --cflags` 

LIBS += -lpthread `gtk-config --libs` -lgdk_pixbuf -lutils

all:: gui

gui: $(OBJS)
	$(CC) -o $@ $(CFLAGS) $(LDFLAGS) $^ $(LIBS)
	ln -f -s ../gui/$(TARGET)  $(BINDIR)/$(TARGET)


