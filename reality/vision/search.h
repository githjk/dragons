/*========================================================================
    Search.h: Stochaistic search method used by camera calibration class
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

#include <unistd.h>
#include <stdlib.h>
#include <math.h>

namespace Search{

const double epsilon = 1.0E-10;

template <class data>
void Copy(data *dest,data *src,int num)
{
  int i;

  for(i=0; i<num; i++) dest[i] = src[i];
}

template <class data>
data Sum(data *arr,int num)
{
  data sum;
  int i;

  sum = 0;
  for(i=0; i<num; i++) sum += arr[i];

  return(sum);
}

template <class data>
data Length(data *arr,int num)
{
  data sum;
  int i;

  sum = 0;
  for(i=0; i<num; i++) sum += arr[i]*arr[i];

  return(sqrt(sum));
}


template <class function,int dim>
double GradientSearch(function &f,double state[dim],int iterations,int samples)
{
  double deriv[dim],nstate[dim],mstate[dim];
  double err,merr,nerr;
  double l,t,t_root,a;
  int i,j,s;

  for(i=0; i<iterations; i++){
    err = f.eval(state);
    if(err < epsilon) return(err);

    a = 1.0 - (i + 1.0) / iterations;

    // Take numerical derivative
    Copy(mstate,state,dim);
    Copy(nstate,state,dim);
    for(j=0; j<dim; j++){
      mstate[j] = state[j] - epsilon;
      nstate[j] = state[j] + epsilon;

      merr = f.eval(mstate);
      nerr = f.eval(nstate);
      deriv[j] = (nerr - merr) / (2 * epsilon);
      // printf("%d:%f-%f ",j,nerr,err);

      mstate[j] = state[j];
      nstate[j] = state[j];
    }

    // Total error derivative for 1 unit of travel
    l = Length(deriv,dim);
    // Use this to estimate distance to zero error
    t_root = err / l;
    // Normalize the derivative into a gradient
    for(j=0; j<dim; j++) deriv[j] = -deriv[j] / l;

    /*
    printf("  state: ");
    for(j=0; j<dim; j++) printf("%g ",state[j]);
    printf("\n");
    */

    /*
    printf("  deriv: ");
    for(j=0; j<dim; j++) printf("%g ",deriv[j]);
    printf("\n");
    */

    // Randomized line search along the gradient
    Copy(mstate,state,dim);
    for(s=0; s<samples; s++){
      /*
      t = drand48() * ((s + 1.0) / samples);
      for(j=0; j<dim; j++){
        nstate[j] = state[j] + t*deriv[j] + (drand48()-0.5)*a*0.01;
      }
      */

      for(j=0; j<dim; j++){
        nstate[j] = state[j] + drand48()*deriv[j]*
          0.5 * ((s + 1.0) / samples);
      }

      nerr = f.eval(nstate);

      /*
      printf("nstate: ");
      for(j=0; j<dim; j++) printf("%g ",nstate[j]);
      printf("\n");
      printf("nerr = %g\n",nerr);
      */

      if(nerr < err){
        err = nerr;
        Copy(mstate,nstate,dim);
      }
    }

    // Select the best new estimate
    Copy(state,mstate,dim);
  }

  printf("error = %g\n",err);

  return(err);
}

template <class function,int dim>
double GradientSearch2(function &f,double state[dim],int iterations)
{
  double deriv[dim],nstate[dim],mstate[dim];
  double dir[dim],loc,step;
  double err,merr,nerr,serr;
  double l;
  // unsigned mask;
  int i,j;

  const bool print = false;

  serr = err = 0.0;

  for(i=0; i<iterations; i++){
    serr = err = f.eval(state);
    if(err < epsilon) return(err);

    /*
    do{
      mask = lrand48() & ((1<<dim) - 1);
    }while(mask == 0);
    */

    switch(lrand48()%4){
      case 0:
	// Take numerical derivative
	Copy(mstate,state,dim);
	Copy(nstate,state,dim);
	for(j=0; j<dim; j++){
	  mstate[j] = state[j] - epsilon;
	  nstate[j] = state[j] + epsilon;

	  merr = f.eval(mstate);
	  nerr = f.eval(nstate);
	  deriv[j] = (nerr - merr) / (2 * epsilon);

	  mstate[j] = state[j];
	  nstate[j] = state[j];
	}

	for(j=0; j<dim; j++) dir[j] = -err / deriv[j];

	if(print){
	  printf("Jump: ");
	  for(j=0; j<dim; j++) printf("%9.6f ",dir[j]);
	  printf("\n");
	}
	break;

      case 1:
	// Take numerical derivative
	Copy(mstate,state,dim);
	Copy(nstate,state,dim);
	for(j=0; j<dim; j++){
	  mstate[j] = state[j] - epsilon;
	  nstate[j] = state[j] + epsilon;

	  merr = f.eval(mstate);
	  nerr = f.eval(nstate);
	  deriv[j] = (nerr - merr) / (2 * epsilon);

	  mstate[j] = state[j];
	  nstate[j] = state[j];
	}

	// Normalize the derivative into a gradient
	l = Length(deriv,dim);
	for(j=0; j<dim; j++) dir[j] = -deriv[j] / l;

	if(print){
	  printf("Gradient: ");
	  for(j=0; j<dim; j++) printf("%9.6f ",dir[j]);
	  printf("\n");
	}
	break;

      case 2:
	for(j=0; j<dim; j++) dir[j] = 0.0;
	dir[lrand48()%dim] = (lrand48()%2)? 1 : -1;
	break;

      case 3:
	for(j=0; j<dim; j++) dir[j] = 2*drand48() - 1;
	break;
    }

    // Line search along that direction
    loc = 0;
    step = 100.0;

    while(step >= epsilon){
      for(j=0; j<dim; j++) nstate[j] = state[j] + dir[j]*(loc+step);
      nerr = f.eval(nstate);

      // printf("  loc=%f step=%f nerr=%f\n",loc,step,nerr);
      // if(isnan(nerr)) sleep(10);

      if(nerr < err){
        err = nerr;
	loc += step;
      }
      step *= 0.5;
    }

    if(err < serr){
      for(j=0; j<dim; j++) state[j] += dir[j]*loc;
      if(print) printf("good: loc=%f derr=%f\n",loc,err-serr);
    }else{
      if(print) printf("drop: loc=%f derr=%f\n",loc,err-serr);
    }
  }

  // printf("error = %g\n",err);

  return(err);
}

template <class function,int dim>
double BlindSearch(function &f,double state[dim],int iterations)
{
  int i,j,k;
  double d,v;
  double serr,err,nerr;
  double dir[dim],nstate[dim],loc,step;
  bool b;

  for(i=0; i<iterations; i++){
    serr = err = f.eval(state);
    if(err < epsilon) return(err);

    // pick a random direction
    for(j=0; j<dim; j++) dir[j] = 2*drand48() - 1;
    loc = step = epsilon;

    while(step >= epsilon){
      for(j=0; j<dim; j++) nstate[j] = state[j] + dir[j]*loc;
      nerr = f.eval(nstate);

      if(nerr < err){
        err = nerr;
	loc += step;
	step *= 2;
      }else{
	step *= 0.5;
      }
    }

    if(err < serr){
      for(j=0; j<dim; j++) state[j] += dir[j]*loc;
      // printf("good: loc=%f\n",loc);
    }else{
      // printf("drop: loc=%f\n",loc);
    }
  }

  printf("error = %g\n",err);

  return(err);
}

} // namespace Search
