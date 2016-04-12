
#ifndef BVECTOR_N_H
#define BVECTOR_N_H


class	sbmatrixN;
class	sbmatrixCSR;
class	sbmatrixCSRA;

class bvectorN
{
private:
    int	n, on;
    vector3*	v;

	// dot product
    friend m_real    operator%( bvectorN const&, bvectorN const& );

public:
     bvectorN();
     bvectorN(int);
     bvectorN(int, vector3*);
	 bvectorN(bvectorN const&);
    ~bvectorN();

	void	getValue(vector3*);
	vector3	getValue(int i) const { return v[i]; }
	void	setValue(vector3*);
	void	setValue(int i, vector3& d) { v[i] = d; }

    vector3&	operator[](int i) const { return v[i]; }
    int		size() const { return n; }
    int		getSize() const { return n; }
    void	setSize(int);

    m_real    len() const ;
    m_real    length() const ;

    bvectorN&  normalize();

	void	setZero();
    void	assign(bvectorN const&);
	void	negate();

    bvectorN&	operator=(bvectorN const&);
    bvectorN&	operator+=(bvectorN const&);
	bvectorN&	operator-=(bvectorN const&);
	bvectorN&	operator*=(m_real);
	bvectorN&	operator/=(m_real);

	void	add(bvectorN const&, bvectorN const&);
    void	sub(bvectorN const&, bvectorN const&);

	void	mult(m_real, bvectorN const&);
    void	mult(bvectorN const&, m_real);
   
	void	mult(sbmatrixN const&, bvectorN const&);
	void	mult(sbmatrixCSR const&, bvectorN const&);
	void	mult(sbmatrixCSRA const&, bvectorN const&);
	void	mult(bvectorN const&, sbmatrixN const&);

    void	div(bvectorN const&, m_real);

	// Preconditioned Conjugate Gradient
    template <typename T> int	solve(const T&, const bvectorN&, void (*filter)(bvectorN&, void*) = NULL, void* user_data = NULL, m_real Etol = 0.1, int MAX_ITER = 1000, bool MAX_ITER_REPORT = false);
	/*
	int	solve(const sbmatrixN&, const bvectorN&, void (*filter)(bvectorN&, void*) = NULL, void* user_data = NULL, m_real Etol = 0.1, int MAX_ITER = 1000, bool MAX_ITER_REPORT = false);
	int	solve(const sbmatrixCSR&, const bvectorN&, void (*filter)(bvectorN&, void*) = NULL, void* user_data = NULL, m_real Etol = 0.1, int MAX_ITER = 1000, bool MAX_ITER_REPORT = false);
	int	solve(const sbmatrixCSRA&, const bvectorN&, void (*filter)(bvectorN&, void*) = NULL, void* user_data = NULL, m_real Etol = 0.1, int MAX_ITER = 1000, bool MAX_ITER_REPORT = false);
	*/
};

template <typename T> 
int
bvectorN::solve(const T& A, const bvectorN& b, void (*filter)(bvectorN&, void* user_data), void* user_data, m_real Etol, int MAX_ITER, bool MAX_ITER_REPORT)
{
	assert(A.row() == A.column() && A.column() == size() && A.row() == b.size());

	bvectorN& x = *this;


	// Block preconditioner
	//
	matrix*	Pinv = new matrix[n];

	#pragma omp parallel for schedule(auto_guided)
	for (int i = 0; i < n; i++)
	{
		Pinv[i] = A.getValue(i, i);
		m_real	det = Pinv[i].det();

		if (det <= 0)	// diagonal preconditioning
		{
			cerr << "Illegal preconditioner at " << i << ": det = " << det << ", " << Pinv[i] << endl;
			Pinv[i][0][0] = 1.0 / Pinv[i][0][0];	Pinv[i][0][1] = Pinv[i][0][2] = 0;
			Pinv[i][1][1] = 1.0 / Pinv[i][1][1];	Pinv[i][1][0] = Pinv[i][1][2] = 0;
			Pinv[i][2][2] = 1.0 / Pinv[i][2][2];	Pinv[i][2][0] = Pinv[i][2][1] = 0;
		}
		else	Pinv[i] = Pinv[i].inverse();	// block diagonal preconditioning
	}

	// delta_0 = filter(b)^T Pinv filter(b)
	m_real		delta_0 = 0;
	bvectorN	bb = b;	if (filter)	filter(bb, user_data);
	#pragma omp parallel for reduction(+:delta_0) if (n > OpenMP_MIN_N)
	for (int i = 0; i < n; i++)
		delta_0 += bb[i] % (Pinv[i] * bb[i]);
	
	// r = filter(b - A * x)
	bvectorN	r(n);
	r.mult(A, x);	r.negate();	r += b;	if (filter)	filter(r, user_data);
	
	// c = filter(Pinv * r);
	bvectorN	c(n);
	#pragma omp parallel for schedule(auto_guided)
	for (int i = 0; i < n; i++)
		c[i] = Pinv[i] * r[i];
	if (filter)	filter(c, user_data);

	m_real	delta_new = r % c;

	//m_real	delta_0 = delta_new;
	//cerr << delta_0 << " vs " << delta_new << endl;
	
	// Stopping criterion
	m_real		e2d = (Etol * Etol) * delta_0;
	bvectorN	q(n);
	bvectorN	s(n);
	//cerr << delta_new << " > " << e2d << endl;
	int	i;
	for (i = 0; i < MAX_ITER && delta_new > e2d; i++)
	{
		// q = filter(Ac)
		q.mult(A, c);	if (filter)	filter(q, user_data);

		// alpha = delta_new / (c % q)
		m_real	alpha = delta_new / (c % q);

		#pragma omp parallel for schedule(auto_guided)		// OpenMP gives good performance in all n (2011 12 21).
		for (int j = 0; j < n; j++)
		{
			// x = x + alpha * c
			x[j] += alpha * c[j];
			
			// r = r - alpha * q
			r[j] -= alpha * q[j];
			
			// s = Pinv * r
			s[j] =  Pinv[j] * r[j];
		}
		
		if (i != 0 && i % 50 == 0)	// To prevent a false zero resdiual 
		{
			//cerr << "reset residual at i = " << i << endl;

			r.mult(A, x);	r.negate();	r += b;	if (filter)	filter(r, user_data);

			// s = Pinv * r
			#pragma omp parallel for schedule(auto_guided)
			for (int j = 0; j < n; j++)
				s[j] =  Pinv[j] * r[j];
		}
		
		// delta_old = delta_new
		m_real	delta_old = delta_new;

		// delta_new = r % s
		delta_new = r % s;
		
		// c = filter(s + (delta_new / delta_old) * c)
		m_real	delta_ratio = delta_new / delta_old;
		#pragma omp parallel for schedule(auto_guided)	if (n > OpenMP_MIN_N)
		for (int j = 0; j < n; j++)
			c[j] = s[j] + delta_ratio * c[j];
		if (filter)	filter(c, user_data);
	}
	if (MAX_ITER_REPORT && i == MAX_ITER)	cerr << "PCG exceeds the maximum number of iterations (" << MAX_ITER << ")!" << endl;

	// Block preconditioner
	delete []	Pinv;

	return	i;
}

#endif
