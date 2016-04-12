#ifndef	_GRID3D_H_
#define	_GRID3D_H_

#include <assert.h>
//
// two dimensional grid3d
//

template<class T>
class grid3d : public array< array< array<T> > >
{
public:
	grid3d() {}
	grid3d(int N, m_real X0, m_real Y0, m_real Z0, m_real X1, m_real Y1, m_real Z1);
	~grid3d()		{ clear(); }
	
	bool	valid(m_real x, m_real y, m_real z);
	T&		lookup(m_real x, m_real y, m_real z);
	T&		lookup(int ix, int iy, int iz);

	int		size()	{ return n; }
	void	clear();
	void	resize(int N, m_real X0, m_real Y0, m_real Z0, m_real X1, m_real Y1, m_real Z1);

//private:
public:
	void	index(m_real x, m_real y, m_real z, int&, int&, int&);

private:
	int	n;	// n x n x n grid3d

	m_real	x0, x1;
	m_real	y0, y1;
	m_real	z0, z1;
};

template<class T>
grid3d<T>::grid3d(int N, m_real X0, m_real Y0, m_real Z0, m_real X1, m_real Y1, m_real Z1)
{
	resize(N, X0, Y0, Z0, X1, Y1, Z1);
}

// Currenlty, it deletes all the memory. So it must be resized after clear to be properly used.
//
template<class T>
void
grid3d<T>::clear()
{
	array< array< array<T> > >::clear();

	x0 = y0 = z0 =  1.0 / EPS;
	x1 = y1 = z1 = -1.0 / EPS;
}

template<class T>
void
grid3d<T>::resize(int N, m_real X0, m_real Y0, m_real Z0, m_real X1, m_real Y1, m_real Z1)
{
	n = N;
	x0 = X0; y0 = Y0; z0 = Z0;
	x1 = X1; y1 = Y1; z1 = Z1;

	array< array< array<T> > >::setSize(n);
	for (int i = 0; i < n; i++)
	{
		array<array<array<T> > >::DATA()[i].setSize(n);
		for (int j = 0; j < n; j++)
			array<array<array<T> > >::DATA()[i].DATA()[j].setSize(n);
	}
}

template<class T>
void
grid3d<T>::index(m_real x, m_real y, m_real z, int& i, int& j, int& k)
{
	i = int(n * (x - x0) / (x1 - x0));
	j = int(n * (y - y0) / (y1 - y0));
	k = int(n * (z - z0) / (z1 - z0));
}

template<class T>
bool
grid3d<T>::valid(m_real x, m_real y, m_real z)
{
	int	ix, iy, iz;
	index(x, y, z, ix, iy, iz);

	if (ix < 0 || ix >= n)	return	false;
	if (iy < 0 || iy >= n)	return	false;
	if (iz < 0 || iz >= n)	return	false;

	return	true;
}

template<class T>
T&
grid3d<T>::lookup(m_real x, m_real y, m_real z)
{
	int	ix, iy, iz;
	index(x, y, z, ix, iy, iz);

	/*
	assert(0 <= ix && ix < n);
	assert(0 <= iy && iy < n);
	assert(0 <= iz && iz < n);
	*/

	return	array<array<array<T> > >::DATA()[ix].DATA()[iy].DATA()[iz];
}

template<class T>
T&
grid3d<T>::lookup(int ix, int iy, int iz)
{
	/*
	assert(0 <= ix && ix < n);
	assert(0 <= iy && iy < n);
	assert(0 <= iz && iz < n);
	*/

	return	array<array<array<T> > >::DATA()[ix].DATA()[iy].DATA()[iz];
}

#endif	// _GRID3D_H_