
#define TRUE    1
#define FALSE   0

#ifndef MAX
#define MAX(x,y) ( ((x)>(y)) ? (x) : (y) )
#endif

#ifndef	MIN
#define MIN(x,y) ( ((x)<(y)) ? (x) : (y) )
#endif

#define ABS(x)		( ((x)>0.0) ? (x) :-(x) )
#define ACOS(x)		( ((x)>1.0) ? (0) : ( ((x)<-1.0) ? (M_PI) : (acos(x)) ) )
#define ASIN(x)		( ((x)>1.0) ? (M_PI/2.0) : ( ((x)<-1.0) ? (-M_PI/2.0) : (asin(x)) ) )
#define SQR(x)		( (x)*(x) )
#define SHIFT(a,b,c,d) (a)=(b);(b)=(c);(c)=(d);
#define SIGN(a,b)	((b) >= 0.0 ? fabs(a) : -fabs(a))

#define	CBRT(a)		( ((a) >= 0) ? pow(a, 1.0/3.0) : -pow(-(a), 1.0/3.0))

