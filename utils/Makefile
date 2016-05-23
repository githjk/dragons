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

# CC = g++-3.0 -I/usr/include/g++-v3/
CC = g++

SRCS= $(wildcard *.cc) 
CSRCS= $(wildcard *.c)
OBJS= $(SRCS:.cc=.o) $(CSRCS:.c=.o)
DEPENDS := $(SRCS:%.cc=.%.dep)

# set the testing file to be the socket test
TESTS=socket_test configreader_test

all:: libutils.a

libutils.so: $(OBJS)
	$(CC) -Wl,-shared $^ -o $@

libutils.a: $(OBJS)
	$(AR) -r $@ $^

# the testing program compilation
test::
	@echo "Making tests..."
	@make $(TESTS)
	@for t in $(TESTS); do \
		echo "Running $$t..."; \
		./$$t; \
	done
