/*========================================================================
    Array.h : Template class for 3D arrays
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

#ifndef __ARRAY_H__
#define __ARRAY_H__

#define ARRAY3D_TEMPLATE template <class data,data def>
#define ARRAY3D_FUN(type) \
  ARRAY3D_TEMPLATE \
  type array3d<data,def>::

ARRAY3D_TEMPLATE
class array3d{
  data *arr;
  int sx,sy,sz;
public:
  array3d()  {arr=NULL; sx=sy=sz=0;}
  array3d(array3d<data,def> &src) {arr=NULL; copy(src);}
  ~array3d() {clear();}

  data *getData() {return(arr);}
  int getSizeX()  {return(sx);}
  int getSizeY()  {return(sy);}
  int getSizeZ()  {return(sz);}
  int getSize()   {return(sx*sy*sz);}

  bool init(int nx,int ny,int nz);

  data get(int x,int y,int z);
  void set(int x,int y,int z,data d);
  data getb(int x,int y,int z);
  bool setb(int x,int y,int z,data d);
  bool set(data d);

  bool copy(array3d<data,def> src);
  void clear();
};

ARRAY3D_FUN(bool) init(int nx,int ny,int nz)
{
  int size = nx*ny*nz;

  if(size == sx*sy*sz) return(true);

  delete(arr);
  arr = new data[size];
  if(arr){
    sx = nx;
    sy = ny;
    sz = nz;
    return(true);
  }else{
    sx = sy = sz = 0;
    return(false);
  }
}

ARRAY3D_FUN(data) get(int x,int y,int z)
{
  return(arr[(z*sy + y)*sx + x]);
}

ARRAY3D_FUN(void) set(int x,int y,int z,data d)
{
  arr[(z*sy + y)*sx + x] = d;
}

ARRAY3D_FUN(data) getb(int x,int y,int z)
{
  if(x>=0 && y>=0 && z>=0 && x<sx && y<sy && z<sz){
    return(arr[(z*sy + y)*sx + x]);
  }else{
    return(def);
  }
}

ARRAY3D_FUN(bool) setb(int x,int y,int z,data d)
{
  if(x>=0 && y>=0 && z>=0 && x<sx && y<sy && z<sz){
    arr[(z*sy + y)*sx + x] = d;
    return(true);
  }else{
    return(false);
  }
}

ARRAY3D_FUN(bool) set(data d)
{
  int size = sx*sy*sz;
  int i;

  for(i=0; i<size; i++) arr[i] = d;
  return (true);
}

ARRAY3D_FUN(bool) copy(array3d<data,def> src)
{
  int size = nx*ny*nz;
  int i;

  if(init(src.sx,src.sy,src.sz)){
    for(i=0; i<size; i++) arr[i] = src.arr[i];
    return(true);
  }else{
    return(false);
  }
}

ARRAY3D_FUN(void) clear()
{
  delete(arr);
  sx = sy = sz = 0;
}

//====================================================================//

#define ARRAY3D_DEF_TEMPLATE template <class data>
#define ARRAY3D_DEF_FUN(type) \
  ARRAY3D_DEF_TEMPLATE \
  type array3d_def<data>::

ARRAY3D_DEF_TEMPLATE
class array3d_def{
  data *arr;
  data def;
  int sx,sy,sz;
public:
  array3d_def()  {arr=NULL; sx=sy=sz=0;}
  array3d_def(array3d_def<data> &src) {arr=NULL; copy(src);}
  ~array3d_def() {clear();}

  data *getData() {return(arr);}
  int getSizeX()  {return(sx);}
  int getSizeY()  {return(sy);}
  int getSizeZ()  {return(sz);}
  int getSize()   {return(sx*sy*sz);}

  data getDefault() {return(def);}
  void setDefault(data ndef) {def=ndef;}

  bool init(int nx,int ny,int nz);

  data get(int x,int y,int z);
  void set(int x,int y,int z,data d);
  data getb(int x,int y,int z);
  bool setb(int x,int y,int z,data d);
  bool set(data d);

  bool copy(array3d_def<data> src);
  void clear();
};

ARRAY3D_DEF_FUN(bool) init(int nx,int ny,int nz)
{
  int size = nx*ny*nz;

  if(size == sx*sy*sz) return(true);

  delete(arr);
  arr = new data[size];
  if(arr){
    sx = nx;
    sy = ny;
    sz = nz;
    return(true);
  }else{
    sx = sy = sz = 0;
    return(false);
  }
}

ARRAY3D_DEF_FUN(data) get(int x,int y,int z)
{
  return(arr[(z*sy + y)*sx + x]);
}

ARRAY3D_DEF_FUN(void) set(int x,int y,int z,data d)
{
  arr[(z*sy + y)*sx + x] = d;
}

ARRAY3D_DEF_FUN(data) getb(int x,int y,int z)
{
  if(x>=0 && y>=0 && z>=0 && x<sx && y<sy && z<sz){
    return(arr[(z*sy + y)*sx + x]);
  }else{
    return(def);
  }
}

ARRAY3D_DEF_FUN(bool) setb(int x,int y,int z,data d)
{
  if(x>=0 && y>=0 && z>=0 && x<sx && y<sy && z<sz){
    arr[(z*sy + y)*sx + x] = d;
    return(true);
  }else{
    return(false);
  }
}

ARRAY3D_DEF_FUN(bool) set(data d)
{
  int size = sx*sy*sz;
  int i;

  for(i=0; i<size; i++) arr[i] = d;
  return (true);
}

ARRAY3D_DEF_FUN(bool) copy(array3d_def<data> src)
{
  int size = nx*ny*nz;
  int i;

  if(init(src.sx,src.sy,src.sz)){
    for(i=0; i<size; i++) arr[i] = src.arr[i];
    def = src.def;
    return(true);
  }else{
    return(false);
  }
}

ARRAY3D_DEF_FUN(void) clear()
{
  delete(arr);
  sx = sy = sz = 0;
}

#endif
