// evaluation.cc
// 
// Robot independent soccer evaluations.
//
// Created by:  Michael Bowling (mhb@cs.cmu.edu)
//
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

#include "soccer.h"
#include "evaluation.h"

Evaluation evaluation;

inline bool inside_bbox(vector2d bbox_min,vector2d bbox_max,
                        vector2d p,double radius)
{
  return((p.x+radius > bbox_min.x) && (p.y+radius > bbox_min.y) &&
         (p.x-radius < bbox_max.x) && (p.y-radius < bbox_max.y));
}

static double diffangle_pos(double a1, double a2) {
  double d = angle_mod(a1 - a2);
  if (d < 0.0) d += M_2PI;
  return d;
}

static int double_compare(const void *a, const void *b) {
  return (*((double *) a) > *((double *) b));
}

bool Evaluation::aim(World &world, double time, 
		     vector2d target, vector2d p2, vector2d p1,
		     int obs_flags,
		     vector2d pref_target_point, double pref_amount,
		     vector2d &target_point, double &target_tolerance)
{
  static struct _double_int { double d; int i; } a[MAX_TEAM_ROBOTS * 4];
  int n = 0, count = 0;
  double a_zero;
  double a_end;

  vector2d r1 = p1 - target;
  vector2d r2 = p2 - target;

  // Swap sides of endpoints if the target cone is oblique.
  if (diffangle_pos(r2.angle(), r1.angle()) > M_PI) {
    vector2d t = r1;
    r1 = r2;
    r2 = t;
  }

  a_zero = r1.angle();
  a_end = diffangle_pos(r2.angle(), a_zero);

  double pref_target_angle = 
    diffangle_pos((pref_target_point - target).angle(), a_zero);
  if (pref_target_angle - a_end > M_2PI - pref_target_angle)
    pref_target_angle -= M_2PI;

  a[n].d = 0.0; a[n++].i = 0; 
  a[n].d = a_end; a[n++].i = 0;

  for(int i=0; i<world.n_teammates; i++) {
    if (!(obs_flags & OBS_TEAMMATE(i))) continue;

    double width = (world.teammate_type(i) == ROBOT_TYPE_DIFF ? 
		    DIFFBOT_WIDTH_H : OMNIBOT_RADIUS);
    vector2d obs = world.teammate_position(i, time) - target;
    
    vector2d obs_perp = obs.rotate(M_PI_2).norm() * width;
    
    double a0 = diffangle_pos(obs.angle(), a_zero);
    double a1 = diffangle_pos((obs - obs_perp).angle(), a_zero);
    double a2 = diffangle_pos((obs + obs_perp).angle(), a_zero);
    
    double maxdist;
    
    if (a0 < a_end)
      maxdist = (a0 / a_end) * (r2.length() - r1.length()) + r1.length();
    else {
      if (a0 < (a_end + M_2PI) / 2.0) maxdist = r2.length();
      else maxdist = r1.length();
    }
    
    if (obs.length() - width > maxdist) continue;
    
    if (a1 < a_end) { a[n].d = a1; a[n++].i = 1; } 
    if (a2 < a_end) { a[n].d = a2; a[n++].i = -1; }
    if (a1 >= a_end && a2 < a_end) { count++; }
    if (a1 >= a_end && a2 >= a_end && a1 > a2) { count++; }
  }

  double width = ROBOT_DEF_WIDTH_H;

  for(int i=0; i<world.n_opponents; i++) {
    if (!(obs_flags & OBS_OPPONENT(i))) continue;

    vector2d obs = world.opponent_position(i, time) - target;
    vector2d obs_perp = obs.rotate(M_PI_2).norm() * width;
    
    double a0 = diffangle_pos(obs.angle(), a_zero);
    double a1 = diffangle_pos((obs - obs_perp).angle(), a_zero);
    double a2 = diffangle_pos((obs + obs_perp).angle(), a_zero);
    
    double maxdist;
    
    if (a0 < a_end)
      maxdist = (a0 / a_end) * (r2.length() - r1.length()) + r1.length();
    else {
      if (a0 < (a_end + M_2PI) / 2.0) maxdist = r2.length();
      else maxdist = r1.length();
    }
    
    if (obs.length() - width > maxdist) continue;
    
    if (a1 < a_end) { a[n].d = a1; a[n++].i = 1; } 
    if (a2 < a_end) { a[n].d = a2; a[n++].i = -1; }
    if (a1 >= a_end && a2 < a_end) { count++; }
    if (a1 >= a_end && a2 >= a_end && a1 > a2) { count++; }
  }

  // Sort the angle array.
  qsort(a, n, sizeof(a[0]), &double_compare);

  // Walk through the angle array finding the largest clear cone, and
  //  the closest clear cone to the preferred angle.
  double best_ang = 0.0, best_tol = 0.0;
  double closest_ang = M_2_PI, closest_tol = 0.0, closest_ang_diff = M_2_PI;
  bool found_one = false;
  
  for(int i=1; i<n; i++) {
    if (!count) {
      double tol = (a[i].d - a[i-1].d) / 2.0;
      double ang = (a[i].d + a[i-1].d) / 2.0;
      double ang_diff = max(0.0, fabs(anglemod(ang - pref_target_angle))-tol);

      if (!found_one || tol > best_tol) {
	best_tol = tol; best_ang = ang;
      }
      
      if (!found_one || ang_diff < closest_ang_diff) {
	closest_tol = tol; closest_ang = ang; closest_ang_diff = ang_diff;
      }

      found_one = true;
    }
    
    count += a[i].i;
  }

  // If there wasn't a clear angle we use the preferred angle and
  // return false.  Otherwise we check whether to use the closest
  // preferred or the largest depending on pref_amount and their
  // angular difference.

  double target_angle; 
  bool rv;

  if (found_one) {

    if (closest_tol + pref_amount > best_tol) {
      target_angle = closest_ang + a_zero;
      target_tolerance = closest_tol;
    } else {
      target_angle = best_ang + a_zero;
      target_tolerance = best_tol;
    }

    rv = true;
  } else {
    target_angle = pref_target_angle + a_zero;
    target_tolerance = 0.0;
    rv = false;
  }

  target_point = 
    intersection(p1, p2, target, target + vector2d(1, 0).rotate(target_angle));
  // target_point += (target_point - target).norm(GOAL_DEPTH);

  return rv;
}

bool Evaluation::defend_point(World &world, double time,
			      vector2d point, 
			      double distmin, double distmax, 
			      double dist_off_ball,
			      bool &intercept,
			      vector2d &target, vector2d &velocity)
{
  double radius = (world.ball_position(time) - point).length() - dist_off_ball;

  if (radius < distmin) return false;
  if (radius > distmax) radius = distmax;

  // We now can compute the static and intercept points and merge them.
  // Index: 0 = intercept, 1 = static(now), 2 = static(future)
  vector2d targets[3];
  double variance[3];
  int rv[3]; 

  rv[0] = intercept && defend_point_intercept(world, time, point, radius,
					      targets[0], variance[0]);
  intercept = rv[0];

  rv[1] = defend_point_static(world, time, point, radius,
			      targets[1], variance[1]);

  rv[2] = defend_point_static(world, time + FRAME_PERIOD, point, radius,
			      targets[2], variance[2]);

  if (rv[0] && rv[1]) {
    target = (targets[0] * variance[1] + targets[1] * variance[0]) /
      (variance[0] + variance[1]);
    target = point + (target - point).norm(radius);

    if (rv[2]) {
      velocity = (targets[2] - targets[1]) / FRAME_PERIOD *
	variance[0] / (variance[0] + variance[1]);
    } else velocity = vector2d(0, 0);

    return true;
  } else if (rv[0]) {
    target = targets[0];

    velocity = vector2d(0, 0);
    
    return true;
  } else if (rv[1]) {
    target = targets[1];

    if (rv[2]) velocity = (targets[2] - targets[1]) / FRAME_PERIOD;
    else velocity = vector2d(0, 0);

    return true;
  }


  return true;
}

bool Evaluation::defend_line(World &world, double time, 
			     vector2d g1, vector2d g2, 
			     double distmin, double distmax, 
			     double dist_off_ball,
			     bool &intercept,
			     int obs_flags, 
			     vector2d pref_point, double pref_amount,
			     vector2d &target, vector2d &velocity)
{
  vector2d ball = world.ball_position(time);
  vector2d g = (g1 + g2) / 2.0;

  vector2d gperp = (g2 - g1).perp().norm();
  if (gperp.dot(ball - g) < 0) gperp *= -1;

  // Special case of defending a single point. 
  if (g1 == g2) return defend_point(world, time, g1, 
				    distmin, distmax, dist_off_ball, intercept,
				    target, velocity);

  // First find the distance between min and max to play.
  //
  // The ratio is the cosine of the difference of the ball's angle to
  // the center point and a perpendicular to the line.
  // 
  double ang = anglemod((ball - g + gperp.norm(distmin)).angle() - 
			gperp.angle());
  double balldist = fabs(offset_to_line(g1, g2, ball));

  double dist = distmin + fabs(cos(ang)) * (distmax - distmin);

  if (dist > balldist - dist_off_ball) 
    dist = balldist - dist_off_ball;

  if (dist > distmax) dist = distmax;
  if (dist < distmin) {
    dist = distmin;
    intercept = false;
  }

  // We now can compute the static and intercept points and merge them.
  // Index: 0 = intercept, 1 = static(now), 2 = static(future)
  vector2d targets[3];
  double variance[3];
  int rv[3]; 

  rv[0] = intercept && defend_line_intercept(world, time, g1, g2, dist, 
					     targets[0], variance[0]);
  intercept = rv[0];

  if (!obs_flags) {
    rv[1] = defend_line_static(world, time, g1, g2, dist, 
			       targets[1], variance[1]);
    rv[2] = defend_line_static(world, time + FRAME_PERIOD, g1, g2, dist, 
			       targets[2], variance[2]);
  } else {
    vector2d p;
    double tol;

    rv[1] = rv[2] = false;

    if (aim(world, time, ball, g1, g2, 
	    obs_flags, pref_point, pref_amount, p, tol)) {
      vector2d ng1, ng2;
      
      ng1 = intersection(ball, ball + (p - ball).rotate(tol), g1, g2);
      ng2 = intersection(ball, ball + (p - ball).rotate(-tol), g1, g2);

      rv[1] = defend_line_static(world, time, ng1, ng2, dist, 
				 targets[1], variance[1]);
      rv[2] = defend_line_static(world, time + FRAME_PERIOD, ng1, ng2, dist, 
				 targets[2], variance[2]);
    }
  }


  if (rv[0] && rv[1]) {
    target = (targets[0] * variance[1] + targets[1] * variance[0]) /
      (variance[0] + variance[1]);

    if (rv[2]) {
      velocity = (targets[2] - targets[1]) / FRAME_PERIOD *
	variance[0] / (variance[0] + variance[1]);
    } else velocity = vector2d(0, 0);

    return true;
  } else if (rv[0]) {
    target = targets[0];

    velocity = vector2d(0, 0);

    return true; 
  } else if (rv[1]) {
    target = targets[1];

    if (rv[2]) velocity = (targets[2] - targets[1]) / FRAME_PERIOD;
    else velocity = vector2d(0, 0);

    return true;
  } else return false;
}

bool Evaluation::defend_on_line(World &world, double time,
				vector2d p1, vector2d p2,
				bool &intercept,
				vector2d &target, vector2d &velocity)
{
  vector2d ball = world.ball_position(time);
  vector2d ball_dt = world.ball_position(time + FRAME_PERIOD);

  vector2d targets[3];
  double variance[2];

  targets[1] = point_on_segment(p1, p2, ball);
  variance[1] = pow((p1 - p2).length(), 2.0) / 16;
  targets[2] = point_on_segment(p1, p2, ball_dt);

  if (intercept && defend_line_intercept(world, time, p1, p2, 0.0, 
					 targets[0], variance[0])) {
    target = (targets[0] * variance[1] + targets[1] * variance[0]) /
      (variance[0] + variance[1]);
    velocity = (targets[2] - targets[1]) / FRAME_PERIOD *
      variance[0] / (variance[0] + variance[1]);
  } else {
    target = targets[1];
    velocity = (targets[2] - targets[1]) / FRAME_PERIOD;
    intercept = false;
  }

  return true;
}

// defend_line_static()
//
// (g1, g2) defines a line segment to be defended.
//
// p1 is the point where the ball shot at g1 crosses the desired line.
// p2 is the point where the ball shot at g2 crosses the desired line.
//
// d1 is the distance from the ball to p1.
// d2 is the distance from the ball to p2.
//
// y is the distance between p1 and p2.
// x is the distance from p1 to the target point.
//

bool Evaluation::defend_line_static(World &world, double time,
				    vector2d g1, vector2d g2, double dist,
				    vector2d &target, double &variance)
{
  vector2d g = (g1 + g2) / 2.0;
  vector2d ball = world.ball_position(time);
  double balldist = fabs(offset_to_line(g1, g2, ball));
  double radius = 90.0; // should eventually be a parameter

  double c1,c2;
  double o1,o2;
  double d1,d2;
  double y;

  vector2d gperp = (g2 - g1).perp().norm();
  if (gperp.dot(ball - g) < 0) gperp *= -1;

  if (balldist < dist + BALL_RADIUS) {
    ball += gperp.norm(dist - balldist + BALL_RADIUS);
    balldist = dist + BALL_RADIUS;
  }

  double ratio = dist / balldist;

  vector2d p1 = ball * ratio + g1 * (1 - ratio);
  vector2d p2 = ball * ratio + g2 * (1 - ratio);

  // calculate inward offsets to take account of our radius
  y = Vector::distance(p1,p2);
  // c1 = fabs(cosine(ball-p1,p2-p1));
  // c2 = fabs(cosine(ball-p2,p1-p2));
  c1 = fabs(sin(vertex_angle(ball,p1,p2)));
  c2 = fabs(sin(vertex_angle(ball,p2,p1)));
  o1 = bound(radius / c1, 0.0, y/2);
  o2 = bound(radius / c2, 0.0, y/2);

  // correct the endpoint positions
  p1 += (p2 - p1).norm(o1);
  p2 += (p1 - p2).norm(o2);
  y = Vector::distance(p1,p2);

  /*
  printf("%f %f : %f %f <%f,%f> <%f,%f> y=%f\n",
         c1,c2,o1,o2,
         V2COMP(p1),V2COMP(p2),y);
  */

  // figure out where we want to be
  if(y > 0.0){
    d1 = (ball - p1).length();
    d2 = (ball - p2).length();

    double x = y * d2 / (d1 + d2);

    target = p1 * (x / y) + p2 * (1 - x / y);
  }else{
    target = p1;
  }
  variance = y * y / 16; // (y/4)^2

  return true;
}

/*
bool Evaluation::defend_line_static(World &world, double time,
				    vector2d g1, vector2d g2, double dist,
				    vector2d &target, double &variance)
{
  vector2d ball = world.ball_position(time);
  double balldist = fabs(offset_to_line(g1, g2, ball));

  if (balldist < dist) return false;

  double ratio = dist / balldist;

  vector2d p1 = ball * ratio + g1 * (1 - ratio);
  vector2d p2 = ball * ratio + g2 * (1 - ratio);

  double y = (p1 - p2).length();

  double d1 = (ball - p1).length();
  double d2 = (ball - p2).length();

  double x = y * d2 / (d1 + d2);

  target = p1 * (x / y) + p2 * (1 - x / y);
  variance = y * y / 16; // (y/4)^2
  
  return true;
}
*/

// defend_line_intercept()
//
// (g1, g2) defines a line segment to be defended.
//
// We lookahead through the ball's trajectory to find where, if it at
// all the ball crosses the goalie's line.  The covariance matrix is then
// used to set the variance for this position.
//

#define P_DefendLookahead 1.0
#define P_DefendLookstep  FRAME_PERIOD

bool Evaluation::defend_line_intercept(World &world, double time,
				       vector2d g1, vector2d g2, double dist,
				       vector2d &target, double &variance)
{
  static double lookahead = P_DefendLookahead;
  static double lookstep = P_DefendLookstep;
  static double radius = 90.0; // Should be a parameter

  vector2d gline = (g2 - g1);
  vector2d gline_1 = gline.norm();
  vector2d gperp = gline_1.rotate(M_PI_2);
  vector2d ball = world.ball_position(time);

  int side = (ball - g1).dot(gperp) >= 0.0 ? 1 : -1;
  
  if (world.ball_velocity(time).dot(gperp) * side >= 0.0)
    return false;

  vector2d orig_g1 = g1, orig_g2 = g2;
  
  //  g1 += gperp * (side * (dist + radius));
  //  g2 += gperp * (side * (dist + radius));

  g1 = intersection(ball, orig_g1, 
		    orig_g1 + gperp * (side * (dist + radius)),
		    orig_g2 + gperp * (side * (dist + radius)));
  g2 = intersection(ball, orig_g2, 
		    orig_g1 + gperp * (side * (dist + radius)),
		    orig_g2 + gperp * (side * (dist + radius)));

  // Lookahead from now to lookahead.
  for(double t = 0.0; t < lookahead; t += lookstep) {

    if (t!=0) 
      gui_debug_line(-1, GDBG_STRATEGY, 
		     world.ball_position(time + t - lookstep), 
		     world.ball_position(time + t));

    vector2d b = world.ball_position(time + t);
    vector2d v = world.ball_velocity(time + t); 

    if (v.dot(gperp) * side >= 0.0) return false;

    double d_to_line = fabs((b - g1).dot(gperp));
    double t_to_line = d_to_line / fabs(v.dot(gperp));

    if (t_to_line > lookstep) continue;

    b += v * t_to_line;

    double x = offset_along_line(g1, g2, b);

    Matrix c = world.ball_covariances(time + t);
    Matrix m = Matrix(4,1); 
    m.e(0,0) = gline_1.x;
    m.e(1,0) = gline_1.y;
    m.e(2,0) = m.e(3,0) = 0.0;
    variance = (transpose(m) * c * m).e(0,0);

    if (x < 0.0) {
      x = 0;
      variance = variance * exp(pow(x, 2.0) / variance);
    } else if (x > gline.length()) {
      x = gline.length();
      variance = variance * exp(pow(gline.length() - x, 2.0) / variance);
    }

    target = g1 + gline_1 * x;
    return true;
  }

  return false;
}

bool Evaluation::defend_point_static(World &world, double time,
				     vector2d point, double radius,
				     vector2d &target, double &variance)
{
  vector2d ball = world.ball_position(time);
  double ball_dist = (ball - point).length();

  target = point + (ball - point).norm(radius);

  double chordlength = (radius / ball_dist) * 
    sqrt(ball_dist * ball_dist - radius * radius);

  variance = chordlength * chordlength / 4.0;

  return true;
}

bool Evaluation::defend_point_intercept(World &world, double time,
					vector2d point, double radius,
					vector2d &target, double &variance)
{
  static double lookahead = P_DefendLookahead;
  static double lookstep = P_DefendLookstep;

  vector2d ball = world.ball_position(time);
  vector2d ball_vel = world.ball_velocity(time);

  if (ball_vel.length() == 0.0 ||
      ball_vel.dot(point - ball) < 0.0 || 
      (ball - point).length() < radius) return false; 

  // Lookahead from now to lookahead.
  double closest_dist = (ball - point).length();
  double closest_time = 0.0;

  for(double t = 0.0; t < lookahead; t += lookstep) {
    vector2d b = world.ball_position(time + t);

    if ((b - point).length() < closest_dist) {
      closest_dist = (b - point).length();
      closest_time = t;
      target = point + (b - point).norm(radius);

      if (closest_dist < radius) break;
    }
  }

  // Compute variance
  Matrix c = world.ball_covariances(time + closest_time);
      
  if (closest_dist > radius) {
    vector2d perp = (target - point).norm();
    Matrix m = Matrix(4,1); 
    m.e(0,0) = perp.x;
    m.e(1,0) = perp.y;
    m.e(2,0) = m.e(3,0) = 0.0;
    variance = (transpose(m) * c * m).e(0,0);

    variance = variance * exp(pow(closest_dist - radius, 2.0) / variance);
  } else {
    vector2d perp = (target - point).perp().norm();
    Matrix m = Matrix(4,1); 
    m.e(0,0) = perp.x;
    m.e(1,0) = perp.y;
    m.e(2,0) = m.e(3,0) = 0.0;
    variance = (transpose(m) * c * m).e(0,0);
  }
  
  if (isinf(variance)) return false;
  else return true;
}

// find the furthest point of a robot in a direction
vector2d Evaluation::farthest(World &world, double time, 
                              int obs_flags,
                              vector2d bbox_min,vector2d bbox_max,
                              vector2d dir)
{
  double x,max_x;
  vector2d obs,max_obs;
  double width;
  int i;

  max_x = 0;
  max_obs.set(0,0);

  for(i=0; i<world.n_teammates; i++){
    if(obs_flags & OBS_TEAMMATE(i)){
      width = (world.teammate_type(i) == ROBOT_TYPE_DIFF ? 
               DIFFBOT_WIDTH_H : OMNIBOT_RADIUS);
      obs = world.teammate_position(i, time);
      if(inside_bbox(bbox_min,bbox_max,obs,width)){
        x = obs.dot(dir) + width;

        if(x > max_x){
          max_x = x;
          max_obs = obs;
        }
      }
    }
  }

  width = OPPONENT_EFFECTIVE_RADIUS;

  for(i=0; i<world.n_opponents; i++){
    if(obs_flags & OBS_OPPONENT(i)){
      obs = world.opponent_position(i, time);

      if(inside_bbox(bbox_min,bbox_max,obs,width)){
        x = obs.dot(dir) + width;

        if(x > max_x){
          max_x = x;
          max_obs = obs;
        }
      }
    }
  }

  max_obs = dir*max_x;
  return(max_obs);
}

vector2d Evaluation::findOpenPosition(World &world, vector2d p, 
				      vector2d toward, int obs_flags,
				      double pradius)
{
  obs_flags = world.obsLine(p, toward, obs_flags);

  vector2d x = p;

  while(1) {
    if (!world.obsPosition(x, obs_flags, pradius)) break;
    
    if ((toward - x).length() < TEAMMATE_EFFECTIVE_RADIUS) {
      x = p; break; }

    x += (toward - p).norm(TEAMMATE_EFFECTIVE_RADIUS);
  }

  return x;
}

vector2d Evaluation::findOpenPositionAndYield(World &world, vector2d p,
					      vector2d toward, int obs_flags)
{
  p = findOpenPosition(world, p, toward, obs_flags,
		       TEAMMATE_EFFECTIVE_RADIUS);

  if (world.trole_active >= 0) 
    p = findOpenPosition(world, p, toward,
			 (obs_flags & OBS_TEAMMATE(world.trole_active)),
			 2 * TEAMMATE_EFFECTIVE_RADIUS);

  return p;
}
