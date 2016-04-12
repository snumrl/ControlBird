
#ifndef	_OMP_AUTO_H_
#define	_OMP_AUTO_H_

#include <omp.h>

#ifdef __INTEL_COMPILER
	#define	auto_guided auto
	#define auto_static auto
	#define auto_dynamic auto
#else
	#define	auto_guided guided
	#define auto_static static
	#define auto_dynamic dynamic, 8
#endif

#endif	// _OMP_AUTO_H_