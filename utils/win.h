/*========================================================================
    Win.h : Simple C++ wrapper interface for XLib
  ------------------------------------------------------------------------
    Copyright (C) 1999-2002  James R. Bruce
    School of Computer Science, Carnegie Mellon University
  ------------------------------------------------------------------------
    This software is distributed under the GNU General Public License,
    version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  ========================================================================*/

#ifndef __WIN_H__
#define __WIN_H__

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <X11/cursorfont.h>

#include <stdio.h>

#ifndef RGB_STRUCT
#define RGB_STRUCT
struct rgb{
  unsigned char red,green,blue;
};
#endif

//==== Graphics/Display Routines =====================================//

class xwin;

class xwindows{
  friend class xdrawable;
  friend class xpixmap;
  friend class xwin;

  Display *disp;
  int screen;
  int depth;
  Window root;
  Visual *vis;

  Window win;
  GC gc;

  xwin *winlist;

  int red_lshift,green_lshift,blue_lshift;
  int red_rshift,green_rshift,blue_rshift;
  int red_mask,green_mask,blue_mask;

  Cursor null_cursor;
  Pixmap null_bm;

public:
  bool initialize();
  void close();
  // xwin createWindow(int width,int height,char *title);
  bool xwindows::createWindow(xwin &win,int width,int height,char *title);
  // xpixmap createPixmap(int width,int height);

  bool checkEvent(XEvent &xev);
  void getEvent(XEvent &xev);
  xwin *getEventWindow(XEvent &xev);
  void waitEvent();
  void flush();
};

class ximage{
  XImage img;
  bool alloc; // true if we allocated pixel buffer
public:
  ximage()
    {alloc = false;}

  bool initialize(void *buf,int nw,int nh,int bpp);
  bool initialize(int nw,int nh)
    {return(initialize(NULL,nw,nh,32));}
  bool initialize(int nw,int nh,int bpp)
    {return(initialize(NULL,nw,nh,bpp));}

  void close();

  rgb *getData()
    {return((rgb*)img.data);}

  /*
  XImage *newImage(int w,int h);
  void deleteImage(XImage *img)
    {XDestroyImage(img);}
  rgb *getImageBuffer(XImage *img)
    {return(img->data);}
  void putImage(XImage *img,int x,int y)
    {XPutImage(xw->disp,win,gc,img,0,0,x,y,img->width,img->height);}
  */

  friend class xdrawable;
};

class xdrawable{
protected:
  xwindows *xw;
  union{
    Drawable draw;
    Window win;
    Pixmap pix;
  };
  Colormap cmap;
  GC gc;
public:
  void setColor(rgb color);
  void setColor(int red,int green,int blue);
  void setColor(int gray)
    {setGray(gray);}
  void setGray(int gray);

  void fillRectangle(int x,int y,int w,int h);
  void fillRectangle(XRectangle r);
  void fillCircle(int x,int y,int r);
  void fillPolygon(XPoint *pts,int num);

  void drawRectangle(int x,int y,int w,int h);
  void drawRectangle(XRectangle r);
  void drawCircle(int x,int y,int r);
  void drawLine(int x1,int y1,int x2,int y2);
  void drawLines(XPoint *pts,int num);
  void drawPoint(int x,int y);
  void drawPoints(XPoint *pts,int num);

  void setLineWidth(int width);
  void setFunction(int func);

  void copyArea(xdrawable &src,int src_x,int src_y,
		int w,int h,int dest_x,int dest_y);
  void copyArea(ximage &src,int src_x,int src_y,
		int w,int h,int dest_x,int dest_y)
    {XPutImage(xw->disp,draw,gc,&src.img,src_x,src_y,dest_x,dest_y,w,h);}

  void copy(ximage &src,int dest_x,int dest_y)
    {XPutImage(xw->disp,draw,gc,&src.img,0,0,dest_x,dest_y,
	       src.img.width,src.img.height);}

  void print(int x,int y,char *str);

  bool loadImage(char *filename);

  void flush()
    {xw->flush();}
};

class xpixmap : public xdrawable{
  friend class xwin;
public:
  void initialize(xwin *nxw,int width,int height);
  void close();
};

#define AllEventMask ((1L<<24)-1)

class xwin : public xdrawable{
  xwin *prev,*next;
  friend class xwindows;
  friend class xpixmap;
public:
  void initialize(xwindows *nxw,int width,int height,const char *title);
  void close();

  void setName(const char *name)
    {XStoreName(xw->disp,win,name);}
  bool equal(Window w)
    {return(win == w);}
  void clearArea(int x,int y,int w,int h,bool exposures)
    {XClearArea(xw->disp,win,x,y,w,h,exposures);}
  void clear()
    {XClearWindow(xw->disp,win);}

  bool checkEvent(XEvent &xev,int mask);
  void getEvent(XEvent &xev,int mask);

  bool checkEvent(XEvent &xev)
    {return(checkEvent(xev,AllEventMask));}
  void getEvent(XEvent &xev)
    {return(getEvent(xev,AllEventMask));}

  void setCursor(int shape);
  void setCursor(int shape,rgb foreground,rgb background);
  // valid shapes defined in X11/cursorfont.h
  void unsetCursor();
  void hideCursor();

  void movePointer(int x,int y);

  void grabPointer()
    {XGrabPointer(xw->disp,win,True,0,GrabModeAsync,GrabModeAsync,win,None,CurrentTime);}
  void ungrabPointer()
    {XUngrabPointer(xw->disp,CurrentTime);}

  xpixmap createPixmap(int width,int height);
  void setBackground(xpixmap &p);
};

#endif
// __WIN_H__
