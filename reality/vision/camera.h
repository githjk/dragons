/*========================================================================
    Camera.h: Internal/external geometric camera calibration for CMVision2
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

#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "geometry.h"


class camera {
protected:
  vector3d loc;              // location of camera origin
  vector3d scale_x,scale_y; // image plane vectors in world space
  double a,b;               // radial distortion parameters

  int width,height;         // image dimensions
  double aspect;            // aspect ratio

  vector3d image;           // vector from origin to image plane

  int field,y_mult;
protected:
  void getState(double *param);
  void setState(double *param);
public:
  bool loadParam(char *filename);
  bool saveParam(char *filename);
  void print();

  void setField(int y_multiplier,int field_offset)
    {y_mult=y_multiplier; field=field_offset;}
  vector3d screenToRay(double sx,double sy);
  vector2d screenToWorld(double sx,double sy,double wz);
  vector2d worldToScreen(vector3d wp);

  void calibrate(vector2d *screen,vector3d *world,int num);
  void calibrate(char *filename);
};

class camera_calibrate : public camera {
protected:
  vector2d *screen;
  vector3d *world;
  int num_points;
public:
  double eval(double *state);

  friend class camera;
};

#endif /*__CAMERA_H__*/
