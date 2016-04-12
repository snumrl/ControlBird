#ifndef	_MATHCLASS_H_
#define	_MATHCLASS_H_

#include <math.h>
#include <assert.h>

#include <iostream>
using namespace std;
//#define	cout	cerr	// Visual stuio 2010 and maya have conflicts in using cout.

//typedef float	m_real;
typedef	double	m_real;

#ifndef	M_PI
#define	M_PI	3.14159265358979323846
#endif

extern m_real	EPS;

extern int		OpenMP_MIN_N;

#include "math_macro.h"

#include "position.h"
#include "vector3.h"
#include "unit_vector.h"
#include "matrix.h"

#include "quater.h"
#include "transf.h"
#include "transq.h"

#include "complex.h"

#include "interval.h"
#include "box.h"

#include "vectorN.h"
#include "matrixN.h"
#include "smatrixN.h"

#include "bvectorN.h"
#include "sbmatrixN.h"

#include "optimize.h"

#include "sgolay.h"

#include "cubic.h"

#include "omp_auto.h"

#include "point2.h"

#endif	// _MATHCLASS_H_
