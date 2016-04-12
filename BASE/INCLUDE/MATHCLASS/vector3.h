
#ifndef VECTOR_H
#define VECTOR_H

class unit_vector;
class matrix;
class transf;

class vector3
{
  private:
    m_real p[3];

    // negation
    friend vector3      operator-( vector3 const& );
    friend unit_vector operator-( unit_vector const& );

    // addtion
    friend position& operator+=( position&, vector3 const& );
    friend vector3&   operator+=( vector3&, vector3 const& );
    friend vector3    operator+( vector3 const&, vector3 const& );
    friend position  operator+( position const&, vector3 const& );
    friend position  operator+( vector3 const&, position const& );

    // subtraction
    friend vector3    operator-( vector3 const&, vector3 const& );
    friend position  operator-( position const&, vector3 const& );
    friend position& operator-=( position&, vector3 const& );
    friend vector3&   operator-=( vector3&, vector3 const& );

    // dot product
    friend m_real    operator%( vector3 const&, vector3 const& );
    friend m_real    operator%( position const&, vector3 const& );
    friend m_real    operator%( vector3 const&, position const& );

    // cross product
    friend vector3    operator*( vector3 const&, vector3 const& );
    friend position  operator*( position const&, unit_vector const& );
    friend position  operator*( unit_vector const&, position const& );

    // scalar Multiplication
    friend vector3    operator*( vector3 const&, m_real );
    friend vector3    operator*( m_real, vector3 const& );
    friend vector3&   operator*=( vector3&, m_real );

    // scalar Division
    friend vector3    operator/( vector3 const&, m_real );
    friend m_real    operator/( vector3 const&, vector3 const& );
    friend vector3&   operator/=( vector3&, m_real );

    // matrix Multiplication
    friend vector3    operator*( vector3 const&, matrix const& );
    friend vector3    operator*( matrix const&, vector3 const& );

    friend vector3&   operator*=( vector3&, transf const& );
    friend vector3    operator*( vector3 const&, transf const& );

    // functions
    friend m_real       len( vector3 const& );
    friend unit_vector  normalize( vector3 const& );

	friend vector3       interpolate( m_real, vector3 const&, vector3 const& );

    friend m_real       angle( vector3 const&, vector3 const& );

    friend position     vector2position( vector3 const& );
    friend vector3       position2vector( position const& );
	friend void			vector2float3( vector3 const& , float[3] );
	friend void			vector2float4( vector3 const& , float[4] );
	friend vector3		float32vector(float*);

	friend matrix	covariance( vector3 const&, vector3 const& );
	friend matrix	selfCovariance( vector3 const& );

    // stream
    friend ostream& operator<<( ostream&, vector3 const& );
    friend istream& operator>>( istream&, vector3& );
    friend ostream& operator<<( ostream&, unit_vector const& );
    friend istream& operator>>( istream&, unit_vector& );

  public:
    // constructors
    vector3() { p[0]=0.0; p[1]=0.0; p[2]=0.0; }
    vector3( m_real x, m_real y, m_real z ) { p[0]=x; p[1]=y; p[2]=z; }
    vector3( m_real a[3] ) { p[0]=a[0]; p[1]=a[1]; p[2]=a[2]; }

	void   setZero()	{ p[0] = p[1] = p[2] = 0; }

    // inquiry functions
    m_real& operator[](int i) { return p[i]; }

    inline m_real x() const { return p[0]; };
    inline m_real y() const { return p[1]; };
    inline m_real z() const { return p[2]; };
    inline void getValue( m_real d[3] )  const { d[0]=p[0]; d[1]=p[1]; d[2]=p[2]; }
	inline m_real getValue( int n ) const { return p[n]; }

    void   setValue( m_real d[3] ) { p[0]=d[0]; p[1]=d[1]; p[2]=d[2]; }
	vector3 setValue( m_real x, m_real y, m_real z )	{ p[0]=x, p[1]=y, p[2]=z; return *this; }
	m_real setValue( int n, m_real x )	{ return p[n]=x; }

	m_real	length() const;
	m_real	sqLength() const;
	matrix	cross() const;

	// position to vector3
	vector3& operator=( position const &);

    // change functions
    void set_x( m_real x ) { p[0]=x; };
    void set_y( m_real x ) { p[1]=x; };
    void set_z( m_real x ) { p[2]=x; };
};

#endif
