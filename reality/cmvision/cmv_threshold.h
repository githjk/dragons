/*========================================================================
    cmv_threshold.h : Color threshold support for CMVision2
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

#ifndef __CMV_THRESHOLD_H__
#define __CMV_THRESHOLD_H__

#include "cmv_types.h"

namespace CMVision{

template <class cmap_t,class image,int bits_y,int bits_u,int bits_v>
void ThresholdImage(cmap_t *cmap,image &img,cmap_t *tmap)
{
  // yuyv *buf,p;
  uyvy *buf,p;
  int i,m,size;

  int rshift_y,rshift_u,rshift_v;
  int lshift_y,lshift_u,lshift_v;

  rshift_y = 8 - bits_y;
  rshift_u = 8 - bits_u;
  rshift_v = 8 - bits_v;

  lshift_y = bits_u + bits_v;
  lshift_u = bits_v;
  lshift_v = 0;

  size = img.width * img.height;
  buf  = img.buf;

  for(i=0; i<size; i++){
    p = buf[i / 2];
    m = ((p.u >> rshift_u) << lshift_u) +
        ((p.v >> rshift_v) << lshift_v);
    cmap[i + 0] = tmap[m + ((p.y1 >> rshift_y) << lshift_y)];
    cmap[i + 1] = tmap[m + ((p.y1 >> rshift_y) << lshift_y)];
  }
}

template <class cmap_t,class image>
void ThresholdImage2(cmap_t *cmap,image &img,cmap_t *tmap)
{
  int *buf,p;
  int i,m,size;

  size = img.width * img.height;
  buf  = (int*)img.buf;

  // YUV 4,6,6
  for(i=0; i<size; i+=2){
    p = buf[i / 2];
    m = ((p & 0x000000FC) <<  4) |
        ((p & 0x00FC0000) >> 18);
    cmap[i + 0] = tmap[((p & 0x0000F000) >> (12-12)) | m];
    cmap[i + 1] = tmap[((p & 0xF0000000) >> (28-12)) | m];
  }

  /*
  // YUV 3,5,5
  for(i=0; i<size; i+=2){
    p = buf[i / 2];
    m = ((p & 0x000000F8) <<  2) |
        ((p & 0x00F80000) >> 19);
    cmap[i + 0] = tmap[((p & 0x0000E000) >> (13-10)) | m];
    cmap[i + 1] = tmap[((p & 0xE0000000) >> (29-10)) | m];
  }
  */
}

template <class cmap_t,class image>
void ThresholdImageRGB16(cmap_t *cmap,image &img,cmap_t *tmap)
{
  unsigned short *buf;
  int i,size;

  size = img.width * img.height;
  buf  = (unsigned short*)img.buf;

  for(i=0; i<size; i++){
    cmap[i] = tmap[buf[i]];
  }
}

template <class cmap_t,class image,int bits_y,int bits_u,int bits_v>
void ThresholdImageYUVPlanar(cmap_t *cmap,image &img,cmap_t *tmap)
{
  uchar *buf_y,*buf_u,*buf_v;
  int i,size;
  uchar py,pu,pv;

  int rshift_y,rshift_u,rshift_v;
  int lshift_y,lshift_u,lshift_v;

  rshift_y = 8 - bits_y;
  rshift_u = 8 - bits_u;
  rshift_v = 8 - bits_v;

  lshift_y = bits_u + bits_v;
  lshift_u = bits_v;
  lshift_v = 0;

  size  = img.width * img.height;
  buf_y = img.buf_y;
  buf_u = img.buf_u;
  buf_v = img.buf_v;

  for(i=0; i<size; i++){
    py = buf_y[i] >> rshift_y;
    pu = buf_u[i] >> rshift_u;
    pv = buf_v[i] >> rshift_v;
    cmap[i] = tmap[(py << lshift_y) +
		   (pu << lshift_u) +
		   (pv << lshift_v)];
  }
}

template <class cmap_t>
void RgbToIndex(cmap_t *map,rgb *img,int width,int height,
		rgb *colors,int num)
{
  int i,j,size;

  size = width * height;

  j = 0;
  for(i=0; i<size; i++){
    if(img[i] != colors[j]){
      j = 0;
      while(j<num && img[i]!=colors[j]) j++;
    }
    map[i] = j;
  }
}

template <class cmap_t,class color_class_state_t>
void IndexToRgb(rgb *img,cmap_t *map,int width,int height,
		color_class_state_t *color,int num)
{
  int i,size;

  size = width * height;

  for(i=0; i<size; i++){
    img[i] = color[map[i]].color;
  }
}

template <class cmap_t>
void IndexToRgb(rgb *img,cmap_t *map,int width,int height,
		rgb *colors,int num)
{
  int i,size;

  size = width * height;

  for(i=0; i<size; i++){
    img[i] = colors[map[i]];
  }
}

template <class data>
data Get3D(data *arr,int num_i,int num_j,int num_k,int i,int j,int k)
{
  int l;
  l = i*num_j*num_k + j*num_k + k;
  return(arr[l]);
}

template <class data>
void Set3D(data *arr,int num_i,int num_j,int num_k,int i,int j,int k,data v)
{
  int l;
  l = i*num_j*num_k + j*num_k + k;
  arr[l] = v;
}

template <class tmap_t>
int RemapTMapColor(tmap_t *tmap,int num_y,int num_u,int num_v,int src_id,int dest_id)
{
  int i,n,size;

  size = num_y * num_u * num_v;
  n = 0;

  for(i=0; i<size; i++){
    if(tmap[i] == src_id){
      tmap[i] = dest_id;
      n++;
    }
  }

  return(n);
}

template <class tmap_t>
int CheckTMapColors(tmap_t *tmap,int num_y,int num_u,int num_v,int colors,int default_id)
{
  int i,n,size;

  size = num_y * num_u * num_v;
  n = 0;

  for(i=0; i<size; i++){
    if(tmap[i] >= colors){
      tmap[i] = default_id;
      n++;
    }
  }

  return(n);
}

template <class tmap_t>
bool LoadThresholdFile(tmap_t *tmap,int num_y,int num_u,int num_v,char *filename)
{
  FILE *in;
  char buf[256];
  int ny,nu,nv;
  int size,read;

  in = fopen(filename,"r");
  if(!in) return(false);

  // read magic
  if(!fgets(buf,256,in)) goto error;
  buf[4] = 0;
  if(strcmp(buf,"TMAP")) goto error;

  // read type (ignore for now)
  if(!fgets(buf,256,in)) goto error;

  // read size
  if(!fgets(buf,256,in)) goto error;
  ny = nu = nv = 0;
  sscanf(buf,"%d %d %d",&ny,&nu,&nv);
  if(num_y!=ny || num_u!=nu || num_v!=nv) goto error;

  size = num_y * num_u * num_v;
  read = fread(tmap,sizeof(tmap_t),size,in);

  fclose(in);
  return(read == size);
error:
  if(in) fclose(in);
  return(false);
}

template <class tmap_t>
bool SaveThresholdFile(tmap_t *tmap,int num_y,int num_u,int num_v,char *filename)
{
  FILE *out;
  int size,wrote;

  out = fopen(filename,"w");
  if(!out) return(false);

  fprintf(out,"TMAP\nYUV%d\n%d %d %d\n",
          sizeof(tmap_t),num_y,num_u,num_v);
  size = num_y * num_u * num_v;
  wrote = fwrite(tmap,sizeof(tmap_t),size,out);
  fclose(out);

  return(wrote == size);
}

} // namespace

#endif
