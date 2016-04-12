
#ifndef BOX_H
#define BOX_H

class box
{
  private:
    interval x, y, z;
    
    // addtion
    friend box& operator+=( box&, vector3 const& );
    friend box  operator+( box const&, vector3 const& );
    friend box  operator+( vector3 const&, box const& );

    // subtraction
    friend box& operator-=( box&, vector3 const& );
    friend box  operator-( box const&, vector3 const& );

    // multiplication by scalar
    friend box& operator*=( box&, m_real );
    friend box  operator*( box const&, m_real );
    friend box  operator*( m_real, box const& );

    // division by scalar
    friend box& operator/=( box&, m_real );
    friend box  operator/( box const&, m_real );

    // inclusion operation
    friend int  operator>>( box const&, position );
    friend int  operator<<( position, box const& );

    // inclusion operation
    friend int  operator>>( box const&, box const& );
    friend int  operator<<( box const&, box const& );

    // or operation
    friend box& operator|=( box&, box const& );
    friend box  operator|( box const&, box const& );

    // and operation
    friend box& operator&=( box&, box const& );
    friend box  operator&( box const&, box const& );

    // logical-and operation
    friend bool	operator&&( box const&, box const& );

    // expansion
    friend box  operator^( box const&, m_real );

    // stream
    friend ostream& operator<<( ostream&, box const& );
    friend istream& operator>>( istream&, box& );


  public:
    // constructors
    box() { };

    box( position const& a )
        : x( a.x() ), y( a.y() ), z( a.z() ) { };

    box( position const& a, position const& b )
        : x( a.x(), b.x() ),
          y( a.y(), b.y() ),
          z( a.z(), b.z() ) { };

    box( interval const& a, interval const& b, interval const& c )
        : x( a ), y( b ), z( c ) { };

	//
	void	set_low(const position& l)	{ x.set_start_pt(l.x()); y.set_start_pt(l.y()); z.set_start_pt(l.z()); }
	void	set_high(const position& h)	{ x.set_end_pt(h.x()); y.set_end_pt(h.y()); z.set_end_pt(h.z()); }

    // inquiry functions
	interval x_range() const { return x; }
	interval y_range() const { return y; }
	interval z_range() const { return z; }

    position low() const	{ return position( x.start_pt(), y.start_pt(), z.start_pt() ); }
    position high() const	{ return position( x.end_pt(),   y.end_pt(),   z.end_pt() ); }
	position center() const	{ return position( x.mid_pt(),   y.mid_pt(),   z.mid_pt()); }
	void center(m_real c[3]) const	{ c[0] = x.mid_pt(), c[1] = y.mid_pt(), c[2] = z.mid_pt(); }

    vector3 project( const vector3& ) const;

	// expand to contain the position p
	box&	expand(const position& p);
};

#endif
