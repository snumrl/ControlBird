
#ifndef VECTOR_N_H
#define VECTOR_N_H

class matrixN;
class smatrixN;

class vectorN
{
  private:
    int     n,
			on;
    m_real *v;

    // stream
    friend ostream&  operator<<( ostream&, vectorN const& );
    friend istream&  operator>>( istream&, vectorN& );

    // dot product
    friend m_real    operator%( vectorN const&, vectorN const& );

  public:
     vectorN();
     vectorN( int );
     vectorN( int, m_real* );
	 vectorN(vectorN const&);
    ~vectorN();

	void	  getValue( m_real* );
	m_real	  getValue( int i ) const { assert(i>=0 && i<n); return v[i]; }
	void	  setValue( m_real* );
	void	  setValue( int i, m_real d ) { assert(i>=0 && i<n); v[i] = d; }

    m_real&   operator[](int i) const { assert(i>=0 && i<n); return v[i]; }
    int       size() const { return n; }
    int       getSize() const { return n; }
    void      setSize( int );

    m_real    len() const ;
    m_real    length() const ;

    vectorN&  normalize();

	vectorN&  setZero();
    vectorN&  assign( vectorN const& );
    vectorN&  operator=( vectorN const& );

    vectorN&  negate();

    vectorN&  add( vectorN const&, vectorN const& );
    vectorN&  operator+=( vectorN const& );

    vectorN&  sub( vectorN const&, vectorN const& );
    vectorN&  operator-=( vectorN const& );

    vectorN&  mult( vectorN const&, m_real );
    vectorN&  operator*=( m_real );

    vectorN&  mult( matrixN const&, vectorN const& );
    vectorN&  mult( vectorN const&, matrixN const& );
    vectorN&  mult( smatrixN const&, vectorN const& );
    vectorN&  mult( vectorN const&, smatrixN const& );

    vectorN&  div( vectorN const&, m_real );
    vectorN&  operator/=( m_real );

	// SOR (Successive Over Relaxation)
    vectorN&  solve( matrixN const&,  vectorN const&, int, m_real, m_real );

	// LU Decomposition
    vectorN&  solve( matrixN const&,  vectorN const& );

	// SVD (Singular Value Decomposition)
    vectorN&  solve( matrixN const&,  vectorN const&, m_real );

	// Preconditioned Conjugate Gradient
    int  solve( smatrixN const&, vectorN const&, m_real Etol = 0.1);

	// Interpolation
	vectorN&  interpolate( m_real, vectorN const&, vectorN const& );
};

#endif
