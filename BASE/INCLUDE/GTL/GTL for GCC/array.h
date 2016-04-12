#ifndef	_ARRAY_H_
#define	_ARRAY_H_

#include <assert.h>

template<class T>
class	array
{
public:
	array()			{ n = 0; data = NULL; }
	array(int N)	{ n = N; data = new T[n]; }
	~array()		{ clear(); }

	void	clear();
	void	resize(int N);
	int		size() const			{ return n; }
	int		length() const			{ return n; }
	T		getValue(int i) const	{ assert( 0 <= i && i < n); return data[i]; }
	T&	operator[](int i) const		{ assert( 0 <= i && i < n); return data[i]; }
	array<T>&	operator=(array<T>&);

	// special care
	T*	DATA() const	{ return data; }

private:
	int	n;
	T*	data;
};

template<class T>
void
array<T>::clear()
{
	n = 0;
	if (data)	delete []	data;
	data = NULL;
}

template<class T>
void
array<T>::resize(int N)
{
	// nothing to do
	if (N < 0 || n == N)	return;

	// copy
	T*	new_data = new T[N];
	for (int i = 0; i < n && i < N; i++)
		new_data[i] = data[i];

	// replace
	n = N;
	if (data)	delete []	data;
	data = new_data;
}

template<class T>
array<T>&
array<T>::operator=(array<T>& a)
{
	if (n != a.n)
	{
		if (data)	delete []	data;

		n = a.n;
		data = new T[n];
	}

	for (int i = 0; i < n; i++)
		data[i] = a.data[i];

	return	*this;
}

#endif	// _ARRAY_H_