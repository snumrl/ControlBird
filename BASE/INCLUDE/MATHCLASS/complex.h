
#ifndef COMPLEX_H
#define COMPLEX_H

class vector3;

class complex
{
  private:
    m_real p[2];

    // negation
    friend complex    operator-( complex const& );

    // addtion
    friend complex    operator+( complex const&, complex const& );

    // subtraction
    friend complex    operator-( complex const&, complex const& );

    // dot product
    friend m_real    operator%( complex const&, complex const& );

    // Multiplication
    friend complex    operator*( complex const&, complex const& );

    // scalar Multiplication
    friend complex    operator*( complex const&, m_real );
    friend complex    operator*( m_real, complex const& );

    // scalar Division
    friend complex    operator/( complex const&, m_real );

    // functions
    friend complex		c_exp( m_real );
	friend m_real		c_ln( complex const& );
    friend complex		inverse( complex const& );
    friend m_real		len( complex const& );
    friend complex		slerp( complex const&, complex const&, m_real );
    friend complex		interpolate( m_real, complex const&, complex const& );
    friend m_real		distance( complex const&, complex const& );
    friend m_real		difference( complex const&, complex const& );

    // stream
    friend ostream& operator<<( ostream&, complex const& );
    friend istream& operator>>( istream&, complex& );

  public:
	//
    // constructors
	//
    complex() {};
    complex( m_real x, m_real y)
                          { p[0]=x; p[1]=y; }
    complex( m_real a[2] ) { p[0]=a[0]; p[1]=a[1]; }

	//
    // inquiry functions
	//
	m_real	real() const { return p[0]; }
	m_real	imaginary() const { return p[1]; }
    complex  inverse() const { return complex( p[0], -p[1]); }
	complex  normalize() const;
	m_real  length() const;
    m_real& operator[] (int i) { return p[i]; }

    void getValue( m_real d[2] ) { d[0]=p[0]; d[1]=p[1]; }
    void setValue( m_real d[2] ) { p[0]=d[0]; p[1]=d[1]; }

    m_real x() const { return p[0]; }
    m_real y() const { return p[1]; }
    
	//
    // Set parameters
	//
    void set_x( m_real x ) { p[0]=x; }
    void set_y( m_real x ) { p[1]=x; }
};

#endif
