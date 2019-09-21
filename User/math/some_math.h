#ifndef __SOME_MATH_H
#define __SOME_MATH_H




#define  LIMIT(x,min,max) ( (x)<(min) ? (min) : ( (x)>(max) ? (max):(x) ))
#define  MAX(x,y) ( (x)>(y) ? (x) :(y) )
#define  MIN(x,y) ( (x)<(y) ? (x): (x) )

#define  DEAD(x,y,z) ( (x)>(y)&&(x)<(z) ? (0): (x) )
#define  ABS(x) ( (x)>(0) ? (x): (-x) )

#define  INT_TO_FLOAT(x,y)\
         x=(int)y;
	
# endif 
