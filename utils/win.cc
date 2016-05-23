/*========================================================================
    Win.cc : Simple C++ wrapper interface for XLib
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

#include "win.h"

#include "stdlib.h"
#include "string.h"

int LeastSetBitNum(int n)
{
  int i = 1;

  if(!n) return(0);

  while(!(n&1)){
    n >>= 1;
    i++;
  }
  return(i);
}

int MostSetBitNum(int n)
{
  int i = 1;

  if(!n) return(-1);

  while(n){
    n >>= 1;
    i++;
  }

  return(i);
}



//==== XWindows Class Implementation =================================//

bool xwindows::initialize()
{
  // General initialization of X
  disp = XOpenDisplay(NULL);
  if(!disp){
    printf("Cannot open display.\n");
    return(false);
  }

  screen = DefaultScreen(disp);
  depth = DefaultDepth(disp,screen);
  root = RootWindow(disp,screen);

  XVisualInfo xvi;
  if(!XMatchVisualInfo(disp,screen,24,TrueColor,&xvi) &&
     !XMatchVisualInfo(disp,screen,32,TrueColor,&xvi) &&
     !XMatchVisualInfo(disp,screen,16,TrueColor,&xvi)){
    printf("This program requires 32, 24, or 16 bit color (%d).\n",depth);
    exit(0);
  }

  red_mask    = xvi.red_mask;
  green_mask  = xvi.green_mask;
  blue_mask   = xvi.blue_mask;

  red_lshift   = LeastSetBitNum(red_mask  ) - 1;
  green_lshift = LeastSetBitNum(green_mask) - 1;
  blue_lshift  = LeastSetBitNum(blue_mask ) - 1;

  red_rshift   = 8 - (MostSetBitNum(red_mask  ) - LeastSetBitNum(red_mask  ));
  green_rshift = 8 - (MostSetBitNum(green_mask) - LeastSetBitNum(green_mask));
  blue_rshift  = 8 - (MostSetBitNum(blue_mask ) - LeastSetBitNum(blue_mask ));

  /*
  printf("shift: L(%d,%d,%d) R:(%d,%d,%d)\n",
	 red_lshift, green_lshift, blue_lshift,
	 red_rshift, green_rshift, blue_rshift);
  */

  vis = xvi.visual;
  winlist = NULL;

  // Create empty cursor
  char bmdata[] = {0};
  XColor c;

  c.red = c.green = c.blue = 0;
  null_bm = XCreateBitmapFromData(disp,root,bmdata,1,1);
  null_cursor = XCreatePixmapCursor(disp,null_bm,null_bm,&c,&c,0,0);

  return(true);
}

void xwindows::close()
{
  if(disp) XCloseDisplay(disp);
  disp = NULL;
}

bool xwindows::createWindow(xwin &win,int width,int height,char *title)
{
  win.initialize(this,width,height,title);
  return(true);
}

/*
xpixmap xwindows::createPixmap(int width,int height)
{
  xpixmap xp;
  xp.initialize(this,width,height);
  return(xp);
}
*/

bool xwindows::checkEvent(XEvent &xev)
{
  bool r = (XPending(disp) != 0);
  if(r) XNextEvent(disp,&xev);
  return(r);
}

void xwindows::getEvent(XEvent &xev)
{
  XNextEvent(disp,&xev);
}

xwin *xwindows::getEventWindow(XEvent &xev)
{
  xwin *p = winlist;
  Window w;

  w = xev.xany.window;
  while(p && p->win!=w) p = p->next;

  return(p);
}

void xwindows::waitEvent()
{
  XEvent xev;
  XPeekEvent(disp,&xev);
}

void xwindows::flush()
{
  XFlush(disp);
}


//==== XImage Class Implementation ===================================//

/*
void printXImage(XImage *i)
{
  printf("width: %d\n",i->width);
  printf("height: %d\n",i->height);
  printf("xoffset: %d\n",i->xoffset);
  printf("format: %d\n",i->format);
  printf("data: 0x%X\n",i->data);
  printf("byte_order: %d\n",i->byte_order);
  printf("bitmap_unit: %d\n",i->bitmap_unit);
  printf("bitmap_bit_order: %d\n",i->bitmap_bit_order);
  printf("bitmap_pad: %d\n",i->bitmap_pad);
  printf("depth: %d\n",i->depth);
  printf("bytes_per_line: %d\n",i->bytes_per_line);
  printf("bits_per_pixel: %d\n",i->bits_per_pixel);
  printf("red_mask: 0x%08X\n",i->red_mask);
  printf("green_mask: 0x%08X\n",i->green_mask);
  printf("blue_mask: 0x%08X\n",i->blue_mask);
}
*/

bool ximage::initialize(void *buf,int nw,int nh,int bpp)
{
  char *data;

  if(buf){
    data = (char*)buf;
  }else{
    data = new char[nw * nh * ((bpp + 7) / 8)];
    alloc = (data != NULL);
    if(!data) return(false);
  }

  img.width  = nw;
  img.height = nh;
  img.xoffset = 0;
  img.format = ZPixmap;
  img.data = data;
  img.byte_order = MSBFirst;
  img.bitmap_unit = 8;
  img.bitmap_bit_order = LSBFirst;
  img.bitmap_pad = 8;
  img.depth = 24;
  img.bytes_per_line = nw * (bpp/8);
  img.bits_per_pixel = bpp; // 32;
  img.red_mask   = 0xFF <<  0;
  img.green_mask = 0xFF <<  8;
  img.blue_mask  = 0xFF << 16;
  // there, wasn't that easy?
  XInitImage(&img);

  // printf("Status = %d\n",XInitImage(img));
  // img = XGetImage(disp,root,0,0,width,height,AllPlanes,ZPixmap);
  // img = XCreateImage(disp,vis,24,ZPixmap,0,data,w,h,8,0);

  return(true);
}

/*
bool ximage::initialize(int nw,int nh)
{
  char *data;
  bool b;

  data = new char[nw * nh * 3];
  if(!data) return(false);

  b = initialize(data,nw,nh,24);

  alloc = b;
  if(!b) delete(data);

  return(b);
}
*/

void ximage::close()
{
  if(alloc && img.data) delete(img.data);
  img.data = NULL;
  // XDestroyImage(&img); // always crashes
}


//==== XDrawable Class Implementation ================================//

void xdrawable::setColor(int red,int green,int blue)
{
  int id;

  id = (((red   << xw->red_lshift  ) >> xw->red_rshift)   & xw->red_mask  ) |
       (((green << xw->green_lshift) >> xw->green_rshift) & xw->green_mask) |
       (((blue  << xw->blue_lshift ) >> xw->blue_rshift)  & xw->blue_mask );

  XSetForeground(xw->disp,gc,id);
}

void xdrawable::setColor(rgb color)
{
  int id;

  id = (((color.red   << xw->red_lshift  ) >> xw->red_rshift)   & xw->red_mask  ) |
       (((color.green << xw->green_lshift) >> xw->green_rshift) & xw->green_mask) |
       (((color.blue  << xw->blue_lshift ) >> xw->blue_rshift)  & xw->blue_mask );

  XSetForeground(xw->disp,gc,id);
}

void xdrawable::setGray(int intensity)
{
  int id;

  id = (((intensity << xw->red_lshift  ) >> xw->red_rshift)   & xw->red_mask  ) |
       (((intensity << xw->green_lshift) >> xw->green_rshift) & xw->green_mask) |
       (((intensity << xw->blue_lshift ) >> xw->blue_rshift)  & xw->blue_mask );

  XSetForeground(xw->disp,gc,id);
}

void xdrawable::fillRectangle(int x,int y,int w,int h)
{
  XFillRectangle(xw->disp,draw,gc,x,y,w,h);
}

void xdrawable::fillCircle(int x,int y,int r)
{
  XFillArc(xw->disp,draw,gc,x-r,y-r,2*r+1,2*r+1,0,360*64);
}

void xdrawable::fillPolygon(XPoint *pts,int num)
{
  XFillPolygon(xw->disp,draw,gc,pts,num,Convex,CoordModeOrigin);
}


void xdrawable::drawRectangle(int x,int y,int w,int h)
{
  XDrawRectangle(xw->disp,draw,gc,x,y,w,h);
}

void xdrawable::drawCircle(int x,int y,int r)
{
  XDrawArc(xw->disp,draw,gc,x-r,y-r,2*r+1,2*r+1,0,360*64);
}

void xdrawable::drawLine(int x1,int y1,int x2,int y2)
{
  XDrawLine(xw->disp,draw,gc,x1,y1,x2,y2);
}

void xdrawable::drawLines(XPoint *pts,int num)
{
  XDrawLines(xw->disp,draw,gc,pts,num,CoordModeOrigin);
}

void xdrawable::drawPoint(int x,int y)
{
  XDrawPoint(xw->disp,draw,gc,x,y);
}

void xdrawable::drawPoints(XPoint *pts,int num)
{
  XDrawPoints(xw->disp,draw,gc,pts,num,CoordModeOrigin);
}

void xdrawable::setLineWidth(int width)
{
  XGCValues values;

  values.line_width = width;
  XChangeGC(xw->disp,gc,GCLineWidth,&values);
}

void xdrawable::setFunction(int func)
// sets the X graphics function, as GX* in X.h, GC.alu
{
  XGCValues values;

  values.function = func;
  XChangeGC(xw->disp,gc,GCFunction,&values);
}

void xdrawable::copyArea(xdrawable &src,int src_x,int src_y,
			int w,int h,int dest_x,int dest_y)
{
  XCopyArea(xw->disp,src.draw,draw,gc,src_x,src_y,w,h,dest_x,dest_y);
}

void xdrawable::print(int x,int y,char *str)
{
  XTextItem t;

  t.chars  = str;
  t.nchars = strlen(str);
  t.delta  = 0;
  t.font   = None;

  XDrawText(xw->disp,draw,gc,x,y,&t,1);
}

bool xdrawable::loadImage(char *filename)
{
  FILE *in;
  int w,h,m,x,y;
  rgb c;

  in = fopen(filename,"rb");
  if(!in) return(false);

  w = h = 0;
  fscanf(in,"P6\n%d %d\n%d\n",&w,&h,&m);
  // printf("w=%d h=%d max=%d\n",w,h,m);

  for(y=0; y<h; y++){
    printf("\rloading '%s' %2d%%",filename,(y*100)/h);
    fflush(stdout);
    for(x=0; x<w; x++){
      fread(&c,sizeof(rgb),1,in);
      setColor(c);
      XDrawPoint(xw->disp,draw,gc,x,y);
    }
  }
  fclose(in);
  printf("\rloading '%s' done.\n",filename);

  return(true);
}


//==== XPixmap Class Implementation ==================================//

void xpixmap::initialize(xwin *nxw,int w,int h)
{
  xw = nxw->xw;

  // Create and initialize pixmap
  unsigned valuemask = 0;
  XGCValues values;

  pix = XCreatePixmap(xw->disp,xw->root,w,h,24);

  cmap = nxw->cmap; // XCreateColormap(xw->disp,win,xw->vis,AllocNone);
  gc = XCreateGC(xw->disp,nxw->win,valuemask,&values);
  // XCopyGC(xw->disp, src, valuemask, dest)nxw->gc);
}

void xpixmap::close()
{
  XFreePixmap(xw->disp,pix);
}

//==== XWin Class Implementation =====================================//

void xwin::initialize(xwindows *nxw,int width,int height,const char *title)
{
  xw = nxw;

  // Create and initialize window
  XSizeHints size_hints;
  unsigned valuemask = 0;
  XGCValues values;
  XEvent xev;

  // printf("Disp=%X Root=%X size=%dx%d\n",xw->disp,xw->root,width,height);

  win = XCreateSimpleWindow(xw->disp,xw->root,0,0,width,height,0,
    WhitePixel(xw->disp,xw->screen), BlackPixel(xw->disp,xw->screen));
  cmap = XCreateColormap(xw->disp,win,xw->vis,AllocNone);
  gc = XCreateGC(xw->disp,win,valuemask,&values);

  size_hints.flags = PSize | PMinSize | PMaxSize;
  size_hints.min_width = width;
  size_hints.max_width = width;
  size_hints.min_height = height;
  size_hints.max_height = height;
  XSetStandardProperties(xw->disp,win,title,title,None,0,0,&size_hints);

  XSelectInput(xw->disp,win,
    ButtonPressMask |
    ButtonMotionMask |
    Button1MotionMask | Button2MotionMask | Button3MotionMask |
    PointerMotionMask |
    KeyPressMask | KeyReleaseMask |
    EnterWindowMask | LeaveWindowMask |
    ExposureMask);
  XMapWindow(xw->disp,win);

  // insert into window list
  if(xw->winlist){
    prev = xw->winlist;
    next = prev->next;
    prev->next = this;
    next->prev = this;
  }else{
    xw->winlist = this;
    prev = next = this;
  }

  // Block until window is mapped
  getEvent(xev,ExposureMask);
}

void xwin::close()
{
  // remove from window list
  if(next != this){
    // other windows, so remove from list
    xw->winlist = next;
    prev->next = next;
    next->prev = prev;
  }else{
    // last window, make winlist empty
    xw->winlist = NULL;
  }
  prev = next = NULL;

  // destroy window
  XDestroyWindow(xw->disp,win);
}

bool xwin::checkEvent(XEvent &xev,int mask)
{
  return(XCheckWindowEvent(xw->disp,win,mask,&xev));
}

void xwin::getEvent(XEvent &xev,int mask)
{
  XWindowEvent(xw->disp,win,mask,&xev);
}

void xwin::setCursor(int shape)
{
  Cursor c;

  c = XCreateFontCursor(xw->disp,shape);
  XDefineCursor(xw->disp,win,c);
}

void xwin::setCursor(int shape,rgb foreground,rgb background)
{
  Cursor c;
  XColor f,b;

  f.red   = foreground.red   * 257;
  f.green = foreground.green * 257;
  f.blue  = foreground.blue  * 257;

  b.red   = background.red   * 257;
  b.green = background.green * 257;
  b.blue  = background.blue  * 257;

  c = XCreateFontCursor(xw->disp,shape);
  XRecolorCursor(xw->disp,c,&f,&b);
  XDefineCursor(xw->disp,win,c);
}

void xwin::unsetCursor()
{
  XUndefineCursor(xw->disp,win);
}

void xwin::hideCursor()
{
  XDefineCursor(xw->disp,win,xw->null_cursor);
}

void xwin::movePointer(int x,int y)
{
  XWarpPointer(xw->disp,None,win,0,0,0,0,x,y);
}


xpixmap xwin::createPixmap(int width,int height)
{
  xpixmap xp;
  xp.initialize(this,width,height);
  return(xp);
}

void xwin::setBackground(xpixmap &p)
{
  XSetWindowBackgroundPixmap(xw->disp,win,p.pix);
}

//==== Event Handling ====//

/*
bool xwin::checkEvent(XEvent &xev)
{
  bool r = (XPending(disp) != 0);
  if(r) XNextEvent(disp,&xev);
  return(r);
}

void xwin::getEvent(XEvent &xev)
{
  XNextEvent(disp,&xev);
}

void xwin::flush()
{
  XFlush(disp);
}
*/
