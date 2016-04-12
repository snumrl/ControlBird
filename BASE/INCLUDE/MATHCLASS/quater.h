
#ifndef QUATER_H
#define QUATER_H

class vector3;
class matrixN;

class quater
{
  private:
    m_real p[4];

    // negation
    friend quater    operator-( quater const& );

    // addtion
    friend quater    operator+( quater const&, quater const& );

    // subtraction
    friend quater    operator-( quater const&, quater const& );

    // dot product
    friend m_real    operator%( quater const&, quater const& );

    // Multiplication
    friend quater    operator*( quater const&, quater const& );

    // scalar Multiplication
    friend quater    operator*( quater const&, m_real );
    friend quater    operator*( m_real, quater const& );

	// Vector multiplication
	friend quater	 operator*( vector3 const&, quater const&);

    // scalar Division
    friend quater    operator/( quater const&, m_real );

    // transform with matrix
    friend matrix    Quater2Matrix( quater const& );
	friend void		 Quater2Matrix(const quater& q, matrixN& M);
    friend quater    Matrix2Quater( matrix const& );
	friend vector3    Quater2EulerAngle( quater const&);
	friend quater    EulerAngle2Quater( vector3 const& r);

    // functions
    friend quater		exp( vector3 const& );
    friend quater		pow( vector3 const&, m_real );
    friend quater		inverse( quater const& );
    friend m_real		len( quater const& );
    friend vector3		ln ( quater const& );
    friend position		rotate( quater const&, position const& );
    friend vector3		rotate( quater const&, vector3 const& );
    friend unit_vector	rotate( quater const&, unit_vector const& );
    friend quater		slerp( quater const&, quater const&, m_real );
    friend quater		interpolate( m_real, quater const&, quater const& );
	friend quater		interpolate_a( m_real, quater const&, quater const& );
    friend m_real		distance( quater const&, quater const& );
    friend vector3		difference( quater const&, quater const& );

	// differential
	friend void			Dln( quater const&, matrixN& );
	friend void			Dexp( vector3 const&, matrixN& );

    // stream
    friend ostream& operator<<( ostream&, quater const& );
    friend istream& operator>>( istream&, quater& );

  public:
	//
    // constructors
	//
    quater() {};
    quater( m_real w, m_real x, m_real y, m_real z )
                          { p[0]=w; p[1]=x; p[2]=y; p[3]=z; }
    quater( m_real a[4] ) { p[0]=a[0]; p[1]=a[1]; p[2]=a[2]; p[3]=a[3]; }

	//
    // inquiry functions
	//
	m_real	real() const { return p[0]; }
	vector3	imaginaries() const { return vector3( p[1], p[2], p[3] ); }
    quater  inverse() const { return quater( p[0], -p[1], -p[2], -p[3] ); }
	quater  normalize() const;
	m_real  length() const;
    m_real& operator[] (int i) { return p[i]; }

    void getValue( m_real d[4] ) { d[0]=p[0]; d[1]=p[1]; d[2]=p[2]; d[2]=p[2]; }
    void setValue( m_real d[4] ) { p[0]=d[0]; p[1]=d[1]; p[2]=d[2]; p[2]=d[2]; }

    m_real w() const { return p[0]; }
    m_real x() const { return p[1]; }
    m_real y() const { return p[2]; }
    m_real z() const { return p[3]; }

	//
    // Set parameters
	//
    void set_w( m_real x ) { p[0]=x; }
    void set_x( m_real x ) { p[1]=x; }
    void set_y( m_real x ) { p[2]=x; }
    void set_z( m_real x ) { p[3]=x; }
};

#endif
