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

#include "vision.h"
#include "timer.h"

rgb YuvToRgb(yuv p)
{
  rgb r;
  int y,u,v;

  y = p.y;
  u = p.v*2 - 255;
  v = p.u*2 - 255;

  r.red   = bound(y + u                     ,0,255);
  r.green = bound((int)(y - 0.51*u - 0.19*v),0,255);
  r.blue  = bound(y + v                     ,0,255);

  return(r);
}

bool LowVision::initialize(char *colorfile,char *tmapfile,int width,int height)
{
  int num_y,num_u,num_v;
  int size;

  max_width  = width;
  max_height = height;

  // Load color information
  num_colors = CMVision::LoadColorInformation(color,MAX_COLORS,colorfile);
  if(num_colors > 0){
    printf("  Loaded %d colors.\n",num_colors);
  }else{
    printf("  ERROR: Could not load color information.\n");
  }

  // Set up threshold
  size = 1 << (bits_y + bits_u + bits_v);
  num_y = 1 << bits_y;
  num_u = 1 << bits_u;
  num_v = 1 << bits_v;

  tmap = new cmap_t[size];
  memset(tmap,0,size*sizeof(cmap_t));

  if(CMVision::LoadThresholdFile(tmap,num_y,num_u,num_v,tmapfile)){
    printf("  Loaded thresholds.\n");
  }else{
    printf("  ERROR: Could not load thresholds.\n");
  }
  CMVision::CheckTMapColors(tmap,num_y,num_u,num_v,num_colors,0);

  // Allocate map structures
  max_width  = width;
  max_height = height;
  size = width * height;
  max_runs = size / MIN_EXP_RUN_LENGTH;
  max_regions = size / MIN_EXP_REGION_SIZE;
  size = max_width * max_height;
  cmap = new cmap_t[size+1]; // extra needed for EncodeRuns terminator
  rmap = new run[max_runs];
  reg  = new region[max_regions];

  return(true);
}

bool LowVision::close()
{
  delete(tmap);
  delete(cmap);
  delete(rmap);
  delete(reg);

  tmap = NULL;
  cmap = NULL;
  rmap = NULL;
  reg  = NULL;

  max_width  = 0;
  max_height = 0;

  return(true);
}

unsigned checksum(char *buf,int len)
{
  unsigned u;
  int i;

  u = 0;
  for(i=0; i<len; i++){
    u = ((u << 5) | (u >> (32-5))) ^ buf[i];
  }

  return(u);
}

bool LowVision::processFrame(image &img,int nfield)
{
  int max_area;

  buf = img.buf;
  width  = img.width;
  height = img.height;
  field = nfield;

  // CMVision::ThresholdImage<cmap_t,image,bits_y,bits_u,bits_v>(cmap,img,tmap);
  CMVision::ThresholdImage2(cmap,img,tmap);
  num_runs = CMVision::EncodeRuns(rmap,cmap,img.width,img.height,max_runs);

  CMVision::ConnectComponents(rmap,num_runs);

  num_regions = CMVision::ExtractRegions(reg,max_regions,rmap,num_runs);

  /*
  printf("runs:%6d (%6d) regions:%6d (%6d)\n",
         num_runs,max_runs,
         num_regions,max_regions);
  */

  max_area = CMVision::SeparateRegions(color,num_colors,reg,num_regions);
  CMVision::SortRegions(color,num_colors,max_area);

  // CMVision::CreateRunIndex(yindex,rmap,num_runs);
  return(true);
}

int WritePPM(char *filename,rgb *img,int width,int height)
{
  FILE *out;
  int wrote;

  out = fopen(filename,"wb");
  if(!out) return(0);

  fprintf(out,"P6\n%d %d\n%d\n",width,height,255);
  wrote = fwrite(img,sizeof(rgb),width*height,out);
  fclose(out);

  return(wrote);
}

bool LowVision::saveThresholdImage(char *filename)
{
  rgb *buf;
  int wrote;

  buf = new rgb[width * height];
  if(!buf) return(false);

  IndexToRgb(buf,cmap,width,height,color,num_colors);
  wrote = WritePPM(filename,buf,width,height);
  delete(buf);

  return(wrote > 0);
}

bool LowVision::saveColorizedImage(char *filename,rgb *reg_color)
{
  rgb *buf;
  int wrote;

  buf = new rgb[width * height];
  if(!buf) return(false);

  RegionToRgbImage(buf,width,height,rmap,num_runs,reg_color);
  wrote = WritePPM(filename,buf,width,height);
  delete(buf);

  return(wrote > 0);
}

pixel LowVision::getImagePixel(int x,int y)
{
  return(buf[(y*width + x) / 2]);
}

region *LowVision::findRegion(int x,int y)
{
  int r;

  r = FindRun(rmap,0,num_runs-1,x,y);
  return((r >= 0)? &reg[rmap[r].parent] : NULL);
}

void LowVision::dump()
{
  printf("  image: %dx%d field=%d\n",width,height,field);
  printf("  runs: %d/%d (%1.2f%%)\n",
	 num_runs,max_runs,100.0*num_runs/max_runs);
  printf("  regions: %d/%d (%1.2f%%)\n",
	 num_regions,max_regions,100.0*num_regions/max_regions);
}

#if 0
int main()
{
  Vision vision;
  capture cap;
  image img;

  int width,height;
  int i,idx;

  width  = 640;
  height = 480;

  vision.initialize("colors.txt","thresh.tmap",width,height);
  cap.initialize(width,height);

  for(i=0; i<1000; i++){
    img.buf = (uyvy*)cap.captureFrame(idx);
    img.width  = width;
    img.height = height;
    vision.processFrame(img);
    printf("."); fflush(stdout);

    // Do stuff with output
    // vision.saveThresholdImage("/tmp/cmap.ppm");

    cap.releaseFrame((unsigned char*)img.buf,idx);
  }

  vision.saveThresholdImage("/tmp/cmap.ppm");

  vision.close();
  cap.close();
  printf("\n");

  return(0);
}
#endif
