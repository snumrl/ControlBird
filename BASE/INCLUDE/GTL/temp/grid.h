#ifndef	_GRID_H_
#define	_GRID_H_

#include <assert.h>
//
// two dimensional grid
//

template<class T>
class grid : public array<array<T> >
{
public:
	grid() {}
	grid(int N, m_real X0, m_real Y0, m_real X1, m_real Y1);

	T&	lookup(m_real x, m_real y);

	int		size()	{ return n; }
	void	clear();
	void	resize(int N, m_real X0, m_real Y0, m_real X1, m_real Y1);

//private:
public:
	void	index(m_real x, m_real y, int&, int&);

private:
	int	n;	// n x n grid

	int	x0, x1;
	int	y0, y1;
};

template<class T>
grid<T>::grid(int N, m_real X0, m_real Y0, m_real X1, m_real Y1)
{
	resize(N, X0, Y0, X1, Y1);
}

template<class T>
void
grid<T>::clear()
{
	x0 = y0 = (int)(1.0 / EPS);
	x1 = y1 = (int)(-1.0 / EPS);
}

template<class T>
void
grid<T>::resize(int N, m_real X0, m_real Y0, m_real X1, m_real Y1)
{
	n = N;
	x0 = (int)X0; y0 = (int)Y0;
	x1 = (int)X1; y1 = (int)Y1;

	array<array<T> >::resize(n);
	for (int i = 0; i < n; i++)
		operator[](i).resize(n);
}

template<class T>
void
grid<T>::index(m_real x, m_real y, int& i, int& j)
{
	i = (int)(n * (x - x0) / (x1 - x0));
	j = (int)(n * (y - y0) / (y1 - y0));
}

template<class T>
T&
grid<T>::lookup(m_real x, m_real y)
{
	int	ix, iy;
	index(x, y, ix, iy);

	assert(0 <= ix && ix < n);
	assert(0 <= iy && iy < n);

	return	operator[](ix)[iy];
}

#endif	// _GRID_H_