/*========================================================================
    capture.h : Video4Linux2 raw video capture class for CMVision2
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

#ifndef __CAPTURE_H__
#define __CAPTURE_H__

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <fcntl.h>

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/videodev.h>

#define DEFAULT_VIDEO_DEVICE  "/dev/video"
#define VIDEO_STANDARD        "NTSC"

#ifdef USE_METEOR
#define DEFAULT_VIDEO_FORMAT  V4L2_PIX_FMT_UYVY
#else
#define DEFAULT_VIDEO_FORMAT  V4L2_PIX_FMT_YUYV
#endif

#define DEFAULT_IMAGE_WIDTH   320
#define DEFAULT_IMAGE_HEIGHT  240
// if you get a message like "DQBUF returned error", "DQBUF error: invalid"
// then you need to use a higher value for STREAMBUFS or process frames faster
#define STREAMBUFS            4

class capture {
  struct vimage_t {
    v4l2_buffer vidbuf;
    char *data;
  };

  int vid_fd;                    // video device
  vimage_t vimage[STREAMBUFS];   // buffers for images
  struct v4l2_format fmt;        // video format request

  unsigned char *current; // most recently captured frame
  stamp_t timestamp;      // frame time stamp
  int width,height;       // dimensions of video frame
  struct v4l2_buffer tempbuf;
  bool captured_frame;
public:
  capture() {vid_fd = 0; current=NULL; captured_frame = false;}
  ~capture() {close();}

  bool initialize(char *device,int nwidth,int nheight,int nfmt);
  bool initialize(int nwidth,int nheight)
    {return(initialize(NULL,nwidth,nheight,0));}
  bool initialize()
    {return(initialize(NULL,0,0,0));}

  void close();

  unsigned char *captureFrame(int &index,int &field);
  unsigned char *captureFrame();
  void releaseFrame(unsigned char* frame, int index);

  unsigned char *getFrame() {return(current);}
  stamp_t getFrameTime() {return(timestamp);}
  double getFrameTimeSec() {return(timestamp * 1.0E-9);}
  int getWidth() {return(width);}
  int getHeight() {return(height);}
};

#endif // __CAPTURE_H__
