
#ifndef POINT2_H__
#define POINT2_H__

#include <math.h>
#include "mathclass.h"

//class vectorN;

struct point2
{
	union{ struct{ m_real x, y; }; m_real p[2]; };
	point2( void ) : x(0),y(0) {};
	point2( m_real _x, m_real _y ) : x(_x),y(_y) {};
	point2( m_real theta ) : x(cosf(theta)),y(sinf(theta)) {};
	point2( vectorN v, int k=0) : x(v[k]), y(v[k+1]) {};
	point2( vectorN v, int k1, int k2 ) : x(v[k1]), y(v[k2]) {};

	m_real&	operator [] ( int i ){ return p[i]; };

	point2&	operator += ( const point2& p ){ x+=p.x; y+=p.y; return *this;};
	point2&	operator -= ( const point2& p ){ x-=p.x; y-=p.y; return *this;};
	point2&	operator *= ( m_real f ){ x*=f; y*=f; return *this;};
	point2&	operator /= ( m_real f ){ x/=f; y/=f; return *this;};

	point2	operator - ( void ) const { return point2(-x,-y); };

	point2	operator - ( const point2& p ) const { return point2(x-p.x,y-p.y); };
	point2	operator + ( const point2& p ) const { return point2(x+p.x,y+p.y); };
	m_real	operator * ( const point2& p ) const { return x*p.x+y*p.y; };
	m_real	operator % ( const point2& p ) const { return x*p.x+y*p.y; };
	point2	operator * ( m_real f ) const { return point2(x*f,y*f); };
	point2	operator / ( m_real f ) const { return point2(x/f,y/f); };
	friend	point2 operator *( m_real f, const point2& p ){ return p*f; };

	m_real	angle(void)	const { return atan2f(y,x); };
	m_real	angle(const point2&_to)	const { return _to.angle()-angle(); };
	friend	m_real angle( const point2&p ) { return atan2f(p.y,p.x); };
	friend	m_real angle( const point2&_from, const point2&_to ){ return _to.angle()-_from.angle();};

	point2&	rotate( m_real the ){ m_real xx=cosf(the)*x-sinf(the)*y, yy=sinf(the)*x+cosf(the)*y; x=xx, y=yy; return *this;};
	friend	point2	rotate( m_real the, const point2&p ){ return point2( cosf(the)*p.x-sinf(the)*p.y, sinf(the)*p.x+cosf(the)*p.y ); };

	m_real	len( void ) const { return sqrtf( x*x+y*y ); };
	m_real	len2(void ) const { return x*x+y*y; };
	friend	m_real	len( const point2&p ){ return p.len(); };

	point2	norm( void ) const { m_real l=len(); return point2(x/l,y/l); };
	friend	point2	norm( const point2&p ) { return p/p.len(); };
	point2&	normalize( void ) { m_real l=len(); x/=l; y/=l; return *this; };
	friend	point2	normalize( const point2&p ){ return p.norm(); };
	vectorN	getVectorN( void ){ vectorN v(2); v[0]=x; v[1]=y; return v; };
};
const quater QI(1,0,0,0);

#endif
