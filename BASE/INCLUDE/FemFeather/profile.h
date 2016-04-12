#ifndef	_PROFILE_H_
#define	_PROFILE_H_

#define	_CPU_TIME_

#include <sys/types.h>
#ifdef _CPU_TIME_
	#include <time.h>
	#include <limits.h>
#else if
//	#include <stdafx.h>
//	#include <time.h>

	#include <windows.h>
	#include <mmsystem.h>
#endif

class	profile
{
public:
	profile();
	~profile();

	virtual void	begin();
	virtual void	end();

	double	used() const;

public:
#ifdef	_CPU_TIME_
	clock_t		starttime;
	clock_t		endtime;
#else if
	SYSTEMTIME	starttime;
	SYSTEMTIME	endtime;
#endif
};

#endif	// _PROFILE_H_
