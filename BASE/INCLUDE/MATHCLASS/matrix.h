
#ifndef MATRIX_H
#define MATRIX_H

class transf;
class quater;

class matrix
{
  private:
    vector3 p[3];

	// negation
    friend matrix	operator-( matrix const& );

    // addition & subtraction
    friend matrix    operator+( matrix const&, matrix const& );
    friend matrix&   operator+=( matrix&, matrix const& );
    friend matrix    operator-( matrix const&, matrix const& );
    friend matrix&   operator-=( matrix&, matrix const& );

    // multiplication
    friend matrix    operator*( m_real, matrix const& );
	friend matrix    operator*( matrix const&, m_real );
	friend matrix    operator/( matrix const&, m_real );
    friend matrix    operator*( matrix const&, matrix const& );
    friend vector3    operator*( matrix const&, vector3 const& );
    friend vector3    operator*( vector3 const&, matrix const& );
    friend position  operator*( matrix const&, position const& );
    friend position  operator*( position const&, matrix const& );
    friend unit_vector  operator*( unit_vector const&, matrix const& );
	friend unit_vector& operator*=( unit_vector&, matrix const& );
	friend matrix&		operator*=( matrix&, m_real );
	friend matrix&		operator/=( matrix&, m_real );

    // transform with quaternions
    friend matrix    Quater2Matrix( quater const& );
    friend quater    Matrix2Quater( matrix const& );

    // functions
    friend matrix    inverse( matrix const& );
    friend matrix    interpolate( m_real, matrix const&, matrix const& );
	friend matrix    mexp( vector3 const& );
	friend vector3    mlog( matrix const& );

	friend vector3	columnVector(matrix& M, int i);
	friend vector3	columnVector(matrix& M);
	friend vector3	rowVector(matrix& M);
	friend matrix	get_row(matrix& M, int i, int j);
	friend matrix	get_row(matrix& M, int i);

    // stream
    friend ostream& operator<<( ostream&, matrix const& );
    friend istream& operator>>( istream&, matrix& );

  public:
    // constructors
    matrix() 
    {
        p[0][0]=1.0; p[0][1]=0.0; p[0][2]=0.0;
        p[1][0]=0.0; p[1][1]=1.0; p[1][2]=0.0;
        p[2][0]=0.0; p[2][1]=0.0; p[2][2]=1.0;
    };
    matrix( vector3 const& a, vector3 const& b, vector3 const& c )
    {
        p[0][0]=a.x(); p[0][1]=a.y(); p[0][2]=a.z();
        p[1][0]=b.x(); p[1][1]=b.y(); p[1][2]=b.z();
        p[2][0]=c.x(); p[2][1]=c.y(); p[2][2]=c.z();
    };
    matrix( m_real a00, m_real a01, m_real a02,
            m_real a10, m_real a11, m_real a12,
            m_real a20, m_real a21, m_real a22 )
    {
        p[0][0]=a00; p[0][1]=a01; p[0][2]=a02;
        p[1][0]=a10; p[1][1]=a11; p[1][2]=a12;
        p[2][0]=a20; p[2][1]=a21; p[2][2]=a22;
    };

	void   setZero()
	{
		p[0][0] = p[0][1] = p[0][2] =
		p[1][0] = p[1][1] = p[1][2] =
		p[2][0] = p[2][1] = p[2][2] = 0;
	}

    // inquiry functions
	inline m_real getValue( int row, int col ) const	{ return p[row].getValue( col );}
	inline vector3 getValue( int row ) const				{ return p[row];}

	inline vector3 setValue(int row, vector3 const& v)	{ return p[row]=v; }
	inline m_real setValue(int row, int col, m_real x)	{ return p[row][col]=x; }

	inline vector3&	operator[](int i) { return p[i]; };
	inline m_real	trace() const { return getValue(0, 0) + getValue(1, 1) + getValue(2, 2); }

    void   getValue( m_real d[3][3] )
			{ d[0][0]=p[0][0]; d[0][1]=p[0][1]; d[0][2]=p[0][2];
			  d[1][0]=p[1][0]; d[1][1]=p[1][1]; d[1][2]=p[1][2];
			  d[2][0]=p[2][0]; d[2][1]=p[2][1]; d[2][2]=p[2][2]; }

    void   setValue( m_real d[3][3] )
			{ p[0][0]=d[0][0]; p[0][1]=d[0][1]; p[0][2]=d[0][2];
			  p[1][0]=d[1][0]; p[1][1]=d[1][1]; p[1][2]=d[1][2];
			  p[2][0]=d[2][0]; p[2][1]=d[2][1]; p[2][2]=d[2][2]; }

	matrix setValue( m_real x00, m_real x01, m_real x02,
					 m_real x10, m_real x11, m_real x12,
					 m_real x20, m_real x21, m_real x22 )
			{ p[0][0]=x00, p[0][1]=x01, p[0][2]=x02,
			  p[1][0]=x10, p[1][1]=x11, p[1][2]=x12,
			  p[2][0]=x20, p[2][1]=x21, p[2][2]=x22;
			  return *this; }

    matrix transpose() const;
    matrix inverse() const;
    m_real det() const;

	m_real	norm();	// Frobenius norm of the matrix
	bool	symmetric(m_real tol = EPS);

	vector3	dual();
	void	SVD3(matrix& A, vector3& d);
	void	rotationBySVD3(matrix& R);

	inline matrix	ata3() const;
	void	eigenvalue(vector3& s) const;
	void	eig_iterative(vector3 E[3], m_real lambda[3]) const;
	int		SVD_direct(matrix& U, vector3& S, matrix& V) const;
	int		SVD_iterative(matrix& U, vector3& S, matrix& V) const;
	bool	SVdecompose(vector3& w, matrix& v);

	void	print();
};

extern matrix scaling( m_real );
extern matrix scaling( m_real, m_real, m_real );
extern matrix rotation( m_real, vector3 const& );
extern matrix reflection( vector3 const& );
extern vector3 Matrix2EulerAngle( const matrix& m );
extern matrix EulerAngle2Matrix( const vector3& m );

#endif
