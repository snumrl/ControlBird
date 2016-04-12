
#ifndef MATRIX_N_H
#define MATRIX_N_H

class vectorN;

class matrixN
{
  private:
    int      n,
             m,
			on;
    vectorN *v;

  public:
    // constructors
	matrixN();
	matrixN( int, int );
	matrixN( int, int, vectorN* );
	matrixN(matrixN const&);
    ~matrixN();

    // inquiry functions
    void      getValue( m_real** ) const;
    m_real    getValue( int, int ) const;
    void      setValue( m_real** );
    void      setValue( int, int, m_real );

    vectorN&  operator[](int i) const { assert(i>=0 && n>i); return v[i]; };
    int       row()    const { return n; }
    int       column() const { return m; }
    void      setSize( int, int );
	void      setRow( int, const vectorN& );
	void	  setColumn( int, const vectorN& );

    matrixN&  transpose( matrixN const& );
    m_real    det() const;

	matrixN&  setZero();
    matrixN&  assign( matrixN const& );
    matrixN&  mult( matrixN const&, matrixN const& );
	matrixN&  sub( matrixN const&, matrixN const& );
	matrixN&  mult(const smatrixN&, const matrixN& );
	matrixN&  mult(const matrixN&, const smatrixN& );
	matrixN&  operator=( matrixN const& );
    matrixN&  operator*=( m_real );
    matrixN&  operator/=( m_real );
    matrixN&  operator+=( matrixN const& );
    matrixN&  operator-=( matrixN const& );

	matrixN&  mergeUpDown( matrixN const&, matrixN const& );
	matrixN&  mergeLeftRight( matrixN const&, matrixN const& );
	void      splitUpDown( matrixN&, matrixN& );
	void      splitLeftRight( matrixN&, matrixN& );
	void      splitUpDown( matrixN&, matrixN&, int );
	void      splitLeftRight( matrixN&, matrixN&, int );

	char      LUdecompose( int* );
	void	  LUsubstitute( int*, vectorN& );
	char      LUinverse( matrixN& );

	void	  SVdecompose( vectorN&, matrixN& );
	void	  SVsubstitute( vectorN const&, matrixN const&,
							vectorN const&, vectorN& );
	void      SVinverse( matrixN& );

	m_real	  norm();
	bool	  symmetric(m_real tol = EPS);
	void	  symmetricize();

	void	  eig_jacobi(matrixN& A, matrixN& B, vectorN& d);
	void	  eig_tri(const matrixN& A, vectorN& d);
	void	  eig_tri(matrixN& A, matrixN& B, vectorN& d);

	void	  tred2(vectorN& d, vectorN& e);
	void	  tqli(vectorN& d, vectorN& e);
	int		  jacobi(matrixN& A, vectorN& d);
	void	  eigsrt(vectorN& d);
	void	  whitening(vectorN& d);
	void	  restore();

    // stream
    friend ostream& operator<<( ostream&, matrixN const& );
    friend istream& operator>>( istream&, matrixN& );
};

#endif
