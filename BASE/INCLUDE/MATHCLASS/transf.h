
#ifndef TRANSF_H
#define TRANSF_H

class quater;

class transf
{
  private:
    matrix m;
    vector3 v;

    // multiplication
    friend transf&      operator*=( transf &, transf const& );
    friend transf       operator*( transf const&, transf const& );
    friend vector3&      operator*=( vector3&, transf const& );
    friend vector3       operator*( vector3 const&, transf const& );
    friend position&    operator*=( position&, transf const& );
    friend position     operator*( position const&, transf const& );
    friend unit_vector& operator*=( unit_vector&, transf const& );
    friend unit_vector  operator*( unit_vector const&, transf const& );

    // functions
    friend transf       inverse( transf const& );
    friend transf       interpolate( m_real, transf const&, transf const& );

	friend void			transf2float16(transf const&, float[16]);

	// stream
    friend ostream& operator<<( ostream&, transf const& );
    friend istream& operator>>( istream&, transf& );

  public:
    // constructors
    transf() {}
    transf( matrix const& a, vector3 const& b ) { m = a; v = b; }
    transf( quater const& a, vector3 const& b ) { m = Quater2Matrix(a); v = b; }

    // inquiry functions
    const matrix&	affine() const { return m; };
    const vector3&	translation() const { return v; };
    const matrix&	getAffine() const { return m; };
    const vector3&	getTranslation() const { return v; };
	const quater	getRotation() const { return Matrix2Quater(m); }

	void			setAffine( matrix const& a ) { m = a; }
	void			setTranslation( vector3 const& a ) { v = a; }
	void			setRotation(quater const& q) { m = Quater2Matrix(q); }

    transf			inverse() const;
};

// identity transform
extern transf identity_transf;

// generator
extern transf scale_transf( m_real );
extern transf scale_transf( m_real, m_real, m_real );
extern transf rotate_transf( m_real, vector3 const& );
extern transf reflect_transf( vector3 const& );
extern transf translate_transf( vector3 const& );
extern transf translate_transf( m_real, m_real, m_real );
extern transf coordinate_transf( position const&,
                  unit_vector const&, unit_vector const& );

#endif
