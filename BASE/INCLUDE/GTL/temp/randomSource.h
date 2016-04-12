#ifndef	_RANDOM_SOURCE_H_
#define	_RANDOM_SOURCE_H_

#include <stdlib.h>

class	random_source
{
public:
	random_source()
	{
		//	srand( (unsigned)time( NULL ) );
	}
	~random_source()	{}

public:
	friend m_real	operator>>(random_source&, m_real& r)	{ return	rand() / (m_real)RAND_MAX; }

	m_real	ran1(int &idum)
	{
#ifdef	_OPENMP	
	error("ran1 does not support asynchronous calls!");
	return;
#endif
		const int IA=16807,IM=2147483647,IQ=127773,IR=2836,NTAB=32;
		const int NDIV=(1+(IM-1)/NTAB);
		const m_real EPS=3.0e-16,AM=1.0/IM,RNMX=(1.0-EPS);
		static int iy=0;
		static int iv[NTAB];
		int j,k;
		m_real	temp;

		if (idum <= 0 || !iy) {
			if (-idum < 1) idum=1;
			else idum = -idum;
			for (j=NTAB+7;j>=0;j--) {
				k=idum/IQ;
				idum=IA*(idum-k*IQ)-IR*k;
				if (idum < 0) idum += IM;
				if (j < NTAB) iv[j] = idum;
			}
			iy=iv[0];
		}
		k=idum/IQ;
		idum=IA*(idum-k*IQ)-IR*k;
		if (idum < 0) idum += IM;
		j=iy/NDIV;
		iy=iv[j];
		iv[j] = idum;
		if ((temp=AM*iy) > RNMX) return RNMX;
		else return temp;
	}

	m_real	ran2(int &idum)
	{
#ifdef	_OPENMP	
	error("ran2 does not support asynchronous calls!");
	return;
#endif
		const int IM1=2147483563,IM2=2147483399;
		const int IA1=40014,IA2=40692,IQ1=53668,IQ2=52774;
		const int IR1=12211,IR2=3791,NTAB=32,IMM1=IM1-1;
		const int NDIV=1+IMM1/NTAB;
		const m_real EPS=3.0e-16,RNMX=1.0-EPS,AM=1.0/m_real(IM1);
		int idum2=123456789,iy=0;
		static int iv[NTAB];
		int j,k;
		m_real	temp;

		if (idum <= 0) {
			idum=(idum==0 ? 1 : -idum);
			idum2=idum;
			for (j=NTAB+7;j>=0;j--) {
				k=idum/IQ1;
				idum=IA1*(idum-k*IQ1)-k*IR1;
				if (idum < 0) idum += IM1;
				if (j < NTAB) iv[j] = idum;
			}
			iy=iv[0];
		}
		k=idum/IQ1;
		idum=IA1*(idum-k*IQ1)-k*IR1;
		if (idum < 0) idum += IM1;
		k=idum2/IQ2;
		idum2=IA2*(idum2-k*IQ2)-k*IR2;
		if (idum2 < 0) idum2 += IM2;
		j=iy/NDIV;
		iy=iv[j]-idum2;
		iv[j] = idum;
		if (iy < 1) iy += IMM1;
		if ((temp=AM*iy) > RNMX) return RNMX;
		else return temp;
	}


	m_real	gasdev(int& idum)
	{
#ifdef	_OPENMP	
	error("gasdev does not support asynchronous calls!");
	return;
#endif
		static int	iset=0;
		static m_real	gset;
		m_real	fac,rsq,v1,v2;

		if (idum < 0)	iset = 0;
		if (iset == 0)
		{
			do	{
				v1=2.0*ran1(idum)-1.0;
				v2=2.0*ran1(idum)-1.0;
				rsq=v1*v1+v2*v2;
			}	while (rsq >= 1.0 || rsq == 0.0);
			fac=sqrt(-2.0*log(rsq)/rsq);
			gset=v1*fac;
			iset=1;

			return v2*fac;
		}
		else	{
			iset=0;
			return	gset;
		}
	}
};

#endif	// _RANDOM_SOURCE_H_