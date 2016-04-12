#ifndef	_ARRAY_H_
#define	_ARRAY_H_

#include <GTL/assert.h>

template<class T> class arrayD;

template<class T>
class	array
{
public:
	array()					{ n = 0; data = NULL; }
	array(array<T>&);
	array(int N)			{ n = N; data = new T[n]; }
	array(int N, T* _data)	{ n = N; data = _data; }
	~array()		{ clear(); }

	void	clear();
	void	setSize(int N)		{ resizeO(N, false); }
	void	resizeNcopy(int N)	{ resizeO(N, true); }

	void	resize(int N, T* newData);
	int		size() const			{ return n; }
	int		length() const			{ return n; }
	T		getValue(int i) const	{ myassert( 0 <= i && i < n); return data[i]; }
	T&	operator[](int i) const		{ myassert( 0 <= i && i < n); return data[i]; }
	array<T>&	operator=(array<T>&);
	array<T>&	operator=(arrayD<T>&);

	// special care
	T*	DATA() const	{ return data; }

private:
	// resizeO() is named because a direct call to this function is illegal.
	// resize() in previous version is the same with resizeNcopy() in the current version.
	//
	// Previous call resize() should be replaced by either setSize() or resizeNcopy()
	//
	void	resizeO(int N, bool copy = false);

private:
	int	n;
	T*	data;
};

template<class T>
array<T>::array(array<T>& a)
{
	n = a.n;
	data = new T[n];

	for (int i = 0; i < n; i++)
		data[i] = a.data[i];
}

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
array<T>::resizeO(int N, bool copy)
{
	// nothing to do
	if (N < 0 || n == N)	return;

	// Prepare new data
	T*	newData = new T[N];
	if (newData == NULL)	{ cerr << "Fail in arrayD<T>::resize(" << N << ")." << endl; exit(0); }	// Excpetion handling

	// copy
	if (copy)
	{
		int	smallN = min(n, N);
		for (int i = 0; i < smallN; i++)
			newData[i] = data[i];
	}

	// replace
	if (data)	delete []	data;

	n = N;
	data = newData;
}

template<class T>
void
array<T>::resize(int N, T* newData)
{
	// replace
	n = N;
	if (data)	delete []	data;
	data = newData;
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

template<class T>
array<T>&
array<T>::operator=(arrayD<T>& a)
{
	if (n != a.length())
	{
		if (data)	delete []	data;

		n = a.length();
		data = new T[n];
	}

	for (int i = 0; i < n; i++)
		data[i] = a[i];

	return	*this;
}


template<class T>
class	arrayD
{
public:
	arrayD()		{ m = n = 0; data = NULL; }
	arrayD(arrayD<T>&);
	arrayD(int N)	{ m = n = N; data = new T[m]; }
	~arrayD()		{ clear(); }

	void	clear();
	bool	resize(int N);
	void	resize(int N, T* newData);
	int		size() const			{ return n; }
	int		length() const			{ return n; }
	int		maxLength() const		{ return m; }
	void	append(T value)			{ resize(n+1); data[n-1] = value; }
	T		getValue(int i) const	{ assert( 0 <= i && i < n); return data[i]; }
	T&	operator[](int i) const		{ assert( 0 <= i && i < n); return data[i]; }
	arrayD<T>&	operator=(arrayD<T>&);
	arrayD<T>&	operator=(array<T>&);

	// special care
	T*	DATA() const	{ return data; }

private:
	int	m;	// maximum number of elements
	int	n;
	T*	data;
};

template<class T>
arrayD<T>::arrayD(arrayD<T>& a)
{
	m = n = a.n;
	data = new T[n];

	for (int i = 0; i < n; i++)
		data[i] = a.data[i];
}

template<class T>
void
arrayD<T>::clear()
{
	m = n = 0;
	if (data)	delete []	data;
	data = NULL;
}

template<class T>
bool
arrayD<T>::resize(int N)
{
	bool	containerChanged = false;

	if (N > m)
	{
		// Doubling
		m = max(2 * m, N);


		// Allocate new memory and then copy existing ones
		T*	newData = new T[m];
		if (newData == NULL)	{ cerr << "Fail in arrayD<T>::resize(" << m << ")." << endl; exit(0); }	// Excpetion handling
		for (int i = 0; i < n; i++)
			newData[i] = data[i];

		
		// Remove exsiting ones
		if (data)	delete []	data;
		data = newData;

		containerChanged = true;
	}

	// Set the current size
	n = N;

	return	containerChanged;
}

template<class T>
void
arrayD<T>::resize(int N, T* newData)
{
	// replace
	m = n = N;
	if (data)	delete []	data;
	data = newData;
}

template<class T>
arrayD<T>&
arrayD<T>::operator=(arrayD<T>& a)
{
	if (n != a.n)
	{
		if (data)	delete []	data;

		m = n = a.n;
		data = new T[n];
	}

	for (int i = 0; i < n; i++)
		data[i] = a.data[i];

	return	*this;
}

template<class T>
arrayD<T>&
arrayD<T>::operator=(array<T>& a)
{
	if (n != a.length())
	{
		if (data)	delete []	data;

		m = n = a.length();
		data = new T[n];
	}

	for (int i = 0; i < n; i++)
		data[i] = a[i];

	return	*this;
}

#endif	// _ARRAY_H_