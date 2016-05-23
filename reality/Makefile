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
UTILDIR = $(TOPDIR)/utils

# CC = g++-3.0 -I/usr/include/g++-v3/
CC = g++

CFLAGS = -fno-rtti -O2 -ffast-math -march=pentium -Wall
# CFLAGS = -fno-rtti -O2 -ffast-math -march=athlon -Wall
# CFLAGS = -fno-rtti -O2 -ffast-math -march=pentium -pg
# CFLAGS += -g
# CFLAGS += -pg -DPROFILE

# target files
SERVER   = rserver
VCLIENT  = vclient
XVCLIENT = xvclient
RCLIENT  = rclient
XDRIVE   = xdrive
CMVEDIT  = cmvedit
GEOCAL   = geocal

CMVSRC      := $(wildcard cmvision/*.cc)
VISIONSRC   := vision/camera.cc vision/detect.cc vision/vision.cc
RADIOSRC    := $(wildcard radio/*.cc)
SERVERSRC   := $(wildcard server/*.cc) $(CMVSRC) $(VISIONSRC) $(RADIOSRC)
VCLIENTSRC  := client/vclient.cc client/client.cc
XDRIVESRC   := client/xdriver.cc client/client.cc
XVCLIENTSRC := client/xvclient.cc client/client.cc
CMVEDITSRC  := $(wildcard cmveditor/*.cc)
GEOCALSRC   := vision/geocal.cc vision/camera.cc

ALLSRC := $(RADIOSRC) $(SERVERSRC) $(VCLIENTSRC) $(XDRIVESRC) $(XVCLIENTSRC) \
          $(CMVEDITSRC) $(GEOCALSRC)
DEPENDS = Makefile.dep

#object files
UTILOBJ     := $(TOPDIR)/utils/libutils.a
SERVEROBJ   := $(SERVERSRC:%.cc=%.o)
VCLIENTOBJ  := $(VCLIENTSRC:%.cc=%.o)
XVCLIENTOBJ := $(XVCLIENTSRC:%.cc=%.o)
CMVEDITOBJ  := cmveditor/main.o $(CMVEDITSRC:%.cc=%.o) $(CMVSRC:%.cc=%.o)
GEOCALOBJ   := $(GEOCALSRC:%.cc=%.o)
XDRIVEOBJ   := $(XDRIVESRC:%.cc=%.o)


INC = -I$(TOPDIR)/include -I$(TOPDIR) -I$(UTILDIR) -I/usr/include/g++-3/ -I/usr/include/X11
DEF = -DUSE_METEOR -DCAPTURE
LIB = -lpthread
XLIB = -L/usr/X11R6/lib -lX11

CMVCFLAGS = $(CFLAGS) `gtk-config --cflags`
CMVLIB = `gtk-config --libs` -lgdk_pixbuf -lpthread


# Make the core files
.SUFFIXES: .cc .c
%.o: %.cc
	$(CC) $(CMVCFLAGS) $(DEF) $(INC) -c $< -o $@
# 	$(CC) -MD $(CMVCFLAGS) $(DEF) $(INC) -c $< -o $@

# include main.c files for cmveditor
%.o: %.c
	$(CC) $(CMVCFLAGS) $(DEF) $(INC) -c $< -o $@
#	$(CC) -MD $(CMVCFLAGS) $(DEF) $(INC) -c $< -o $@

ifneq ($(shell /sbin/lsmod | grep videodev),)
all:: $(SERVER) $(VCLIENT) $(XVCLIENT) $(CMVEDIT) $(GEOCAL) $(XDRIVE) bin
else
all:: $(VCLIENT) $(XVCLIENT) $(XDRIVE) bin
endif


# Make each of the targets respecitvley
$(CMVEDIT): $(CMVEDITOBJ) $(UTILOBJ)
	$(CC) $(CMVCFLAGS) $(OPTFLAGS) -o $@ $(OBJ) $(CMVEDITOBJ) $(UTILOBJ) $(CMVLIB)

$(SERVER): $(SERVEROBJ) $(UTILOBJ)
	$(CC) $(CFLAGS) $(OPTFLAGS) -o $@ $(OBJ) $(SERVEROBJ) $(UTILOBJ) $(LIB)

$(VCLIENT): $(VCLIENTOBJ) $(UTILOBJ)
	$(CC) $(CFLAGS) $(OPTFLAGS) -o $@ $(OBJ) $(VCLIENTOBJ) $(UTILOBJ) $(LIB)

$(XVCLIENT): $(XVCLIENTOBJ) $(UTILOBJ)
	$(CC) $(CFLAGS) $(OPTFLAGS) -o $@ $(OBJ) $(XVCLIENTOBJ) $(UTILOBJ) $(LIB) $(XLIB)

$(GEOCAL): $(GEOCALOBJ) $(UTILOBJ)
	$(CC) $(CFLAGS) $(OPTFLAGS) -o $@ $(OBJ) $(GEOCALOBJ) $(UTILOBJ) $(LIB)

$(XDRIVE): $(XDRIVEOBJ) $(UTILOBJ)
	$(CC) $(CFLAGS) $(OPTFLAGS) -o $@ $(OBJ) $(XDRIVEOBJ) $(UTILOBJ) $(LIB) $(XLIB)

# $(SERVER) $(VCLIENT) $(XVCLIENT) $(CMVEDIT) $(GEOCAL)
bin:
	ln -f -s ../reality/$(SERVER)   $(BINDIR)/$(SERVER)
	ln -f -s ../reality/$(VCLIENT)  $(BINDIR)/$(VCLIENT)
	ln -f -s ../reality/$(XVCLIENT) $(BINDIR)/$(XVCLIENT)
	ln -f -s ../reality/$(RCLIENT)  $(BINDIR)/$(RCLIENT)
	ln -f -s ../reality/$(XDRIVE)   $(BINDIR)/$(XDRIVE)
	ln -f -s ../reality/$(CMVEDIT)  $(BINDIR)/$(CMVEDIT)

dep:: $(DEPENDS)

depend:: $(DEPENDS)

$(DEPENDS)::
	$(CC) -M $(CFLAGS) $(CMVCFLAGS) $(INC) $(ALLSRC) > $(DEPENDS)

clean::
	rm -f *~ *.o server/*~ server/*.o client/*~ client/*.o
	rm -f vision/*~ vision/*.o radio/*~ radio/*.o cmvision/*~ cmvision/*.o

include $(DEPENDS)
