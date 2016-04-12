
#ifndef TRANSQ_H
#define TRANSQ_H

class transq
{
  public:
    quater rotation;
    vector3 translation;

  public:
    // multiplication
    friend transq&      operator*=( transq &, transq const& );
    friend transq       operator* ( transq const&, transq const& );
    friend vector3&      operator*=( vector3&, transq const& );
//    friend vector3       operator* ( vector3 const&, transq const& );
    friend vector3       operator* ( transq const&, vector3 const&);
    friend position&    operator*=( position&, transq const& );
//    friend position     operator* ( position const&, transq const& );
	friend position     operator* ( transq const&, position const& );
    friend unit_vector& operator*=( unit_vector&, transq const& );
    friend unit_vector  operator* ( unit_vector const&, transq const& );

    // functions
    friend transq       interpolate( m_real, transq const&, transq const& );
	friend transq		interpolate_a( m_real, transq const&, transq const& );

	// transform with transf
    friend transf	Transq2Transf( transq const& );
	friend transq	Transf2Transq( transf const& );

    // stream
    friend ostream& operator<<( ostream&, transq const& );
    friend istream& operator>>( istream&, transq& );

  public:
    // constructors
    transq() {};
    transq( quater const& a, vector3 const& b ) { rotation=a; translation=b; }

    transq			inverse() const;
};

// identity transq
extern transq identity_transq;

// generator
extern transq rotate_transq( m_real angle, vector3 const& axis );
extern transq translate_transq( vector3 const& axis );
extern transq translate_transq( m_real x, m_real y, m_real z );

#endif
