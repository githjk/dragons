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

#ifndef __VISION_H__
#define __VISION_H__

#include "vtypes.h"
#include "../cmvision/cmvision.h"

typedef uyvy pixel;
typedef uchar cmap_t;
typedef CMVision::image<pixel> image;
typedef CMVision::color_class_state color_class_state;
typedef CMVision::run<cmap_t> run;
typedef CMVision::region region;

/*
const int bits_y = 3;
const int bits_u = 5;
const int bits_v = 5;
*/

const int bits_y = 4;
const int bits_u = 6;
const int bits_v = 6;

#define MAX_COLORS 9

#define MIN_EXP_REGION_SIZE 192
#define MIN_EXP_RUN_LENGTH   16

// Must match indicies in colors.txt!!
#define COLOR_ORANGE 1
#define COLOR_GREEN  2
#define COLOR_PINK   3
#define COLOR_PURPLE 4
#define COLOR_BLUE   5
#define COLOR_YELLOW 6
#define COLOR_WHITE  7
#define COLOR_BGREEN 8

rgb YuvToRgb(yuv p);

class LowVision{
  pixel *buf;
  cmap_t *cmap,*tmap;
  run *rmap;
  region *reg;

  color_class_state color[MAX_COLORS];

  int width,height;
  int max_width,max_height;
  int max_runs,max_regions;
  int num_colors,num_runs,num_regions;
  int field;

public:
  bool initialize(char *colorfile,char *tmapfile,int width,int height);
  bool close();

  bool processFrame(image &img,int nfield);
  bool saveThresholdImage(char *filename);
  bool saveColorizedImage(char *filename,rgb *reg_color);

  region *getRegions(int c)
    {return(color[c].list);}
  yuv getAverageColor(region *reg)
    {return(CMVision::AverageColor(buf,width,height,rmap,reg->run_start));}
  int getNumRegions(int c)
    {return(color[c].num);}
  int getRegionID(region *r)
    {return(r - reg);}
  region *findRegion(int x,int y);

  cmap_t getClassPixel(int x,int y)
    {return(cmap[y*width+x]);}
  pixel getImagePixel(int x,int y);
  int getField()
    {return(field);}

  void dump();
};

#endif /*__VISION_H__*/
