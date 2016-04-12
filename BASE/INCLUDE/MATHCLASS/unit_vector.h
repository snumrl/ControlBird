
#ifndef UNIT_VECTOR_H
#define UNIT_VECTOR_H

class matrix;
class transf;

class unit_vector : public vector3
{
  private:
    // negation
    friend unit_vector    operator-( unit_vector const& );

    // cross product
    friend position  operator*( position const&, unit_vector const& );
    friend position  operator*( unit_vector const&, position const& );

    friend unit_vector& operator*=( unit_vector&, matrix const& );
    friend unit_vector  operator*( unit_vector const&, matrix const& );

    friend unit_vector& operator*=( unit_vector&, transf const& );
    friend unit_vector  operator*( unit_vector const&, transf const& );

    // functions
    friend int    equal_normal( unit_vector const&,
                                unit_vector const& );
    friend transf coordinate_transf( position const&,
                                     unit_vector const&,
                                     unit_vector const& );

  public:
    // constructors
    unit_vector() : vector3(0,0,0) { };
    unit_vector( m_real x, m_real y, m_real z ) : vector3( x, y, z ) { };
    unit_vector( m_real a[3] ) : vector3( a ) { };
};

extern unit_vector x_axis, y_axis, z_axis;

#endif
