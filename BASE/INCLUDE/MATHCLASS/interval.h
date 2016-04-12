
#ifndef INTERVAL_H
#define INTERVAL_H

class interval
{
  private:
    m_real start, end;
    
    // negation
    friend interval operator-( interval const& );

    // addtion
    friend interval& operator+=( interval&, m_real );
    friend interval  operator+( m_real, interval const& );
    friend interval  operator+( interval const&, m_real );

    // subtraction
    friend interval& operator-=( interval&, m_real );
    friend interval  operator-( interval const&, m_real );

    // multiplication by scalar
    friend interval& operator*=( interval&, m_real );
    friend interval  operator*( interval const&, m_real );
    friend interval  operator*( m_real, interval const& );

    // division by scalar
    friend interval& operator/=( interval&, m_real );
    friend interval  operator/( interval const&, m_real );

    // inclusion operation
    friend int   operator>>( interval const&, m_real );
    friend int   operator<<( m_real, interval const& );

    // inclusion operation
    friend int   operator>>( interval const&, interval const& );
    friend int   operator<<( interval const&, interval const& );

    // or operation
    friend interval& operator|=( interval&, interval const& );
    friend interval  operator|( interval const&, interval const& );

    // and operation
    friend interval& operator&=( interval&, interval const& );
    friend interval  operator&( interval const&, interval const& );

    // logical-and operation
    friend bool		operator&&( interval const&, interval const& );

    // expansion
    friend interval  operator^( interval const&, m_real );

    // axiliary functions
    friend void adjust_interval( interval& );

    // stream
    friend ostream& operator<<( ostream&, interval const& );
    friend istream& operator>>( istream&, interval& );

  public:
    // constructors
    interval( ) { start = 1.0 / EPS; end = -1.0 / EPS; }
    interval( m_real a ) { start = end = a; }
    interval( m_real a, m_real b ) { start = MIN(a,b);
                                     end   = MAX(a,b); }

	//
	void	set_start_pt(m_real s)	{ start = s; }
	void	set_end_pt(m_real e)	{ end = e; }

    // inquiry functions
    m_real start_pt() const { return start; }
    m_real end_pt() const { return end; }
    m_real mid_pt() const { return (m_real)((start+end)/2.0); }

    m_real interpolate( m_real t ) const { return (1-t)*start + t*end; }
    m_real len( ) const { return end - start; }

	m_real distance( m_real ) const;
    m_real project( m_real ) const;

	// expand to contain the coodirnate p
	interval&	expand(m_real p);
};

#endif
