#pragma once

#include <time.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <ode/ode.h>
#include <string.h>

#include "GTL/array.h"
#include "MATHCLASS/mathclass.h"
#include "SISL\sisl.h"
#include "GL\glut.h"

using namespace std;

#define INFINITE_REAL	std::numeric_limits<m_real>::max()
#define INFINITE_INT	std::numeric_limits<int>::max()

#define	AirDensity		1.225f	

#define	EPSILON			1.0e-5

#define RadToDeg(x) ((double)(x/M_PI*180.f))	
#define DegToRad(x) ((double)(x/180.f*M_PI))
#define Rad2Deg(x) ((double)(x/M_PI*180.f))	
#define Deg2Rad(x) ((double)(x/180.f*M_PI))

#define RadToAng(x) ((double)(x/M_PI*180.f))
#define AngToRad(x) ((double)(x/180.f*M_PI))
#define RadToDeg(x) ((double)(x/M_PI*180.f))
#define DegToRad(x) ((double)(x/180.f*M_PI))

#define SAME_VALUE(a,b) (fabs((a)-(b)) <= EPSILON)
#define BETWEEN(x, x0, x1)	( ( (x)>=(x0) ) && ( (x)<=(x1) ) )
#define M_ROUND(x) (int)(x+0.5f)
#define M_RANDOM(s,e) (double)((double)(s)+((double)(e)-(double)(s))*(rand()/(double)RAND_MAX))
#define RANDOM(s,e) (double)((double)(s)+((double)(e)-(double)(s))*(rand()/(double)RAND_MAX))
#define RANDOM_INT(E) (int)(rand()%E)
#define SQUARE(x) (double)((x)*(x))

#define GAUSSIAN_FUNC(x,a,mu,sigma) ((double)a*(exp(-1.f*(x-mu)*(x-mu)/(double)sigma)))

#define VECTOR_PROJECTION(a,b)			((a%b)/b.length())
#define LERP_ARRAY(from, to, alpha)		(1.0f-alpha)*(from) + alpha*(to)

#define SAME_ARRAY3(v1,v2)				(SAME_VALUE((v1)[0],(v2)[0]) && SAME_VALUE((v1)[1],(v2)[1]) && SAME_VALUE((v1)[2],(v2)[2]))
#define SAME_ARRAY4(q1,q2)				(SAME_VALUE((q1)[0],(q2)[0]) && SAME_VALUE((q1)[1],(q2)[1]) && SAME_VALUE((q1)[2],(q2)[2]) && SAME_VALUE((q1)[3],(q2)[3]))
#define WRITE_ARRAY2(f,v)				f << (v)[0] << " " << (v)[1] << std::endl
#define WRITE_ARRAY3(f,v)				f << (v)[0] << " " << (v)[1] << " " << (v)[2] << " " << std::endl
#define WRITE_ARRAY4(f,q)				f << (q)[0] << " " << (q)[1] << " " << (q)[2] << " " << (q)[3] << " " << std::endl
#define WRITE_ARRAY2_NOENDL(f,v)		f << (v)[0] << " " << (v)[1] << " "
#define WRITE_ARRAY3_NOENDL(f,v)		f << (v)[0] << " " << (v)[1] << " " << (v)[2] << " "
#define WRITE_ARRAY4_NOENDL(f,q)		f << (q)[0] << " " << (q)[1] << " " << (q)[2] << " " << (q)[3] << " "
#define READ_ARRAY2(f,v)				f >> (v)[0]; f >> (v)[1];
#define READ_ARRAY3(f,v)				f >> (v)[0]; f >> (v)[1]; f >> (v)[2];
#define READ_ARRAY4(f,q)				f >> (q)[0]; f >> (q)[1]; f >> (q)[2]; f >> (q)[3];

#define GET_ANGULAR_VELOCITY_QUATER(n, b, time) ((n%b<0) ? 2*ln(-n*b.inverse())/time : 2*ln(n*b.inverse())/time)
#define DIFFERENCE2(n, b)	ln((n%b<0) ? (-n*b.inverse()) : (n*b.inverse()))

#define COUT_STRING(s)		cout << s << endl

#define CLAMP_FLOOR(x, v)				if(x < v) x=v
#define CLAMP_CEIL(x, v)				if(x > v) x=v

#define SET_GL_VERTEX(v)	glVertex3f(v[0],v[1],v[2])

#define GOLDEN_RATIO 1.618

#define GRAVITY		vector3(0, -9.81, 0)

#define SLERP(q1, q2, t)	((q1)%(q2) > 0) ? slerp((q1), (q2), (t)) : slerp(-(q1), (q2), (t))

#ifndef axis_X
#define	axis_X		0
#endif
#ifndef axis_Y
#define	axis_Y		1
#endif
#ifndef axis_Z
#define	axis_Z		2
#endif

using namespace std;

static bool LT( m_real a,  m_real b)
{
	return a < b;
}
static bool LT_PAIR( pair<int,m_real>& p1,  pair<int,m_real>& p2)
{
	return p1.second < p2.second;
}
static bool GT( m_real a,  m_real b)
{
	return a > b;
}
static bool GT_PAIR( pair<int,m_real>& p1,  pair<int,m_real>& p2)
{
	return p1.second > p2.second;
}

static quater quaterFromVector( const vector3& v1, const vector3& v2 )
{ 
	vector3 vn1 = normalize(v1);
	vector3 vn2 = normalize(v2);
	vector3 v = atan2f(len(vn1*vn2),vn1%vn2) * normalize(vn1*vn2) / 2;
	return exp( v );
}

static quater quaterFromAxisAngle( const vector3& a, m_real t ){ return exp( a*t/2.0f ); };

//static bool isBoxIn( vectorN& x,  vectorN& axis_len,  m_real tolerance=1.0,  int cutDim=INFINITE_INT)
//{
//	int N = x.size();
//	int N_r = N;
//	m_real tor_sum = 0.0;
//	m_real axis_len_sum = 0.0;
//	vector< pair<int,m_real> > axis_len_sorted;
//
//	for(int i=0; i<N; i++)
//	{
//		axis_len_sum += axis_len[i];
//		axis_len_sorted.push_back( make_pair(i,axis_len[i]) );
//	}
//
//	sort(axis_len_sorted.begin(),axis_len_sorted.end(),GT_PAIR);
//
//	for(int i=0; i<N; i++)
//	{
//		tor_sum += axis_len_sorted[i].second / axis_len_sum;
//		if(tor_sum >= tolerance || i>=cutDim)
//		{
//			N_r = i+1;
//			break;
//		}
//	}
//
//	bool isIn = true;
//
//	for(int i=0; i<N_r; i++)
//	{
//		int idx = axis_len_sorted[i].first;
//		if( abs(x[idx]) > abs(axis_len[idx]) + EPSILON )
//		{
//			isIn = false;
//			break;
//		}
//	}
//
//	return isIn;
//}
//static bool isElipseIn( vectorN& x,  vectorN& axis_len,  m_real tolerance=1.0,  int cutDim=INFINITE_INT)
//{
//	int N = x.size();
//	int N_r = N;
//	m_real tor_sum = 0.0;
//	m_real axis_len_sum = 0.0;
//	vector< pair<int,m_real> > axis_len_sorted;
//
//	for(int i=0; i<N; i++)
//	{
//		axis_len_sum += axis_len[i];
//		axis_len_sorted.push_back( make_pair(i,axis_len[i]) );
//	}
//
//	sort(axis_len_sorted.begin(),axis_len_sorted.end(),GT_PAIR);
//
//	for(int i=0; i<N; i++)
//	{
//		tor_sum += axis_len_sorted[i].second / axis_len_sum;
//		if(tor_sum >= tolerance || i>=cutDim)
//		{
//			N_r = i+1;
//			break;
//		}
//	}
//
//	vectorN x_r(N_r);
//
//	for(int i=0; i<N_r; i++)
//	{
//		int idx = axis_len_sorted[i].first;
//		x_r[i] = x[idx] / axis_len[idx];
//	}
//
//	return x_r.length() <= 1.0 + EPSILON;
//}
//static bool isCircleIn( vectorN& x,  vectorN& axis_len,  m_real tolerance=1.0)
//{
//	int N = x.size();
//	int N_r = N;
//	m_real tor_sum = 0.0;
//	m_real axis_len_sum = 0.0;
//	vector< pair<int,m_real> > axis_len_sorted;
//
//	for(int i=0; i<N; i++)
//	{
//		axis_len_sum += axis_len[i];
//		axis_len_sorted.push_back( make_pair(i,axis_len[i]) );
//	}
//
//	sort(axis_len_sorted.begin(),axis_len_sorted.end(),GT_PAIR);
//
//	/*for(int i=0; i<N; i++)
//	{
//		tor_sum += axis_len_sorted[i].second / axis_len_sum;
//		if(tor_sum >= tolerance || i>=2)
//		{
//			N_r = i+1;
//			break;
//		}
//	}*/
//
//	return x.length() <= axis_len_sorted[0].second;
//}

static void glSetMaterial(	float dr,      float dg,      float db,      float da=1.0f,
					float ar=-1, float ag=-1, float ab=-1, float aa=-1,
					float sr=0.0f, float sg=0.0f, float sb=0.0f, float sa=-1, float exponent=0.3f )
{
	float cc[4];
	cc[0]=dr, cc[1]=dg, cc[2]=db, cc[3]=da;
	glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, cc );

	cc[0]=ar<0?dr*0.2f:ar, cc[1]=ag<0?dg*0.2f:ag, cc[2]=ab<0?db*0.2f:ab, cc[3]=aa<0?da:aa;
	glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, cc );

	cc[0]=sr, cc[1]=sg, cc[2]=sb, cc[3]=sa<0?da:sa;
	glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, cc );

	glMaterialf ( GL_FRONT_AND_BACK, GL_SHININESS, exponent );
}

static void glSetMaterial( float d[4], float a[4]=NULL, float s[4]=NULL, float exponent=0.3f )
{
	float cc[4];
	glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, d );

	cc[0]=((a==NULL)?(d[0]*0.2f):a[0]);
	cc[1]=((a==NULL)?(d[1]*0.2f):a[1]);
	cc[2]=((a==NULL)?(d[2]*0.2f):a[2]);
	cc[3]=((a==NULL)?(d[3]):a[3]);
	glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, cc );

	cc[0]=((s==NULL)?0:s[0]);
	cc[1]=((s==NULL)?0:s[1]);
	cc[2]=((s==NULL)?0:s[2]);
	cc[3]=((s==NULL)?d[3]:s[3]);
	glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, cc );

	glMaterialf ( GL_FRONT_AND_BACK, GL_SHININESS, exponent );
}

struct 
SE3
{
	position pos;
	quater frame;

	SE3()
	{
	}

	SE3(quater q, position p)
	{
		pos = p;
		frame = q;
	}

	SE3& operator=(  SE3& _t )
	{	
		pos	= _t.pos;
		frame	= _t.frame;
		
		return *this;
	}

	friend SE3 operator*(  SE3& se3,  transf& T )
	{
		SE3 ret;

		transf T0 = transf(se3.frame, position2vector(se3.pos));

		T0 = T0 * T;

		ret.pos = vector2position( T0.getTranslation() );
		ret.frame = T0.getRotation();
	
		return ret;
	}
};
static void glTransform(vector3 trans, quater rot)
{
	vector3 axis;
	double angle;

	axis = ln(rot);
	angle = 2*axis.length(); 
	axis = normalize(axis);
	glTranslatef(trans[0], trans[1], trans[2]);
	glRotatef(RadToAng(angle), axis[0], axis[1], axis[2]);
}
static void glTransform(position p, quater ori)
{
	vector3 axis;
	double angle;

	axis = ln(ori);
	angle = 2*axis.length();
	axis = normalize(axis);
	glTranslatef(p[0], p[1], p[2]);
	glRotatef(RadToAng(angle), axis[0], axis[1], axis[2]);
}
static void glTransform(transf transform)
{
	vector3 trans = transform.getTranslation();
	quater rot	= transform.getRotation();

	vector3 axis;
	double angle;

	axis = ln(rot);
	angle = 2*axis.length();
	axis = normalize(axis);
	glTranslatef(trans[0], trans[1], trans[2]);
	glRotatef(RadToAng(angle), axis[0], axis[1], axis[2]);
}
static double
triangleArea(position p0, position p1, position p2)
{
	return 0.5f * ((p1-p0) * (p2-p0)).length();
}
static position
COM(position p0, position p1, position p2)
{
	position com;

	for(int i=0; i<3; i++)
		com[i] = 1.0/3.0f * p0[i] + 1.0/3.0f * p1[i] + 1.0/3.0f * p2[i];
	
	return com;
}
static string
getNext(ifstream& fin, string delim)
{	
	char ch[1024];

	while(true)
	{
		fin >> ch;

		if(strcmp(ch, delim.c_str())==0) fin.getline(ch,1024);
		else					break;
	}

	return string(ch);
}
static void
quatToOdeQuat(  quater& quat, dQuaternion odeQuat )
{
	odeQuat[0] = quat.w();
	odeQuat[1] = quat.x();
	odeQuat[2] = quat.y();
	odeQuat[3] = quat.z();	
}
static vector3
arrToVec( const dReal* arr )
{
	vector3 vec( arr[0], arr[1], arr[2] );
	return vec;
}
static quater
arrToQuat( const dReal* arr )
{
	quater q;
	q.set_w( arr[0] );
	q.set_x( arr[1] );
	q.set_y( arr[2] );
	q.set_z( arr[3] );

	return q;
}
static void
drawEllipse()
{
	float a	= 1.0f/2.0f;
	float b	= 1.0f/8.0f;
	float c	= 1.0f/8.0f;
	float z2	= 0.0f,	d2 = 0.0f; 
	float z1	= 0.0f,	d1 = 0.0f; 
	int step	= 2; 
	int slice	= 20; 
	float phi	= M_PI/(float)slice; 

	z1 = c; 
	d1 = 0;
	
	for(int i=step; i<=slice; i+=step) 
	{ 
		z2 = (float)(c*cos(i*phi)); 
		d2 = (float)sqrt(1-pow(z2/c,2)); 
		
		glBegin(GL_QUAD_STRIP); 
		for(int j=2*slice; j>=0; j-=step) 
		{ 
			glNormal3f((float)(cos(phi*j)*sin(phi*i)), (float)(sin(phi*j)*sin(phi*i)), (float)cos(phi*i)); 
			glVertex3f((float)(a*d2*cos(phi*j)), (float)(b*d2*sin(phi*j)), (float)z2); 
			
			glNormal3f((float)(cos(phi*j)*sin((i-step)*phi)), (float)(sin(phi*j)*sin((i-step)*phi)), (float)cos((i-step)*phi)); 
			glVertex3f((float)(a*d1*cos(phi*j)), (float)(b*d1*sin(phi*j)), (float)z1);
		} 
		glEnd();
		z1 = z2;
		d1 = d2;
	}
}
static void
simpleEllipse(vector3 start, vector3 end)
{
	vector3 v = (start + end)/2.0f;
	vector3 v1 = end - start;

	float a = v1.length();
	vector3 dir = v1;
	dir = normalize(dir);

	glPushMatrix();
	glTranslatef(v[0], v[1], v[2]);
	glPushMatrix();
	vector3 axis( 1.0f, 0.0f, 0.0f );
	vector3 normal = axis*dir;		
	normal = normalize(normal);
	glRotatef((float)acos((axis%dir)) * (float)(180.0f/M_PI), normal[0], normal[1], normal[2]);
	glScalef( a, 0.03f, 0.03f );
	drawEllipse();
	glPopMatrix();	
	glPopMatrix();
}
static void
drawLine_start_end(position start, position end)
{
	glBegin(GL_LINES);
	glVertex3f(start[0],start[1],start[2]);
	glVertex3f(end[0],end[1],end[2]);
	glEnd();
}
static void
drawLine_start_vector(position start, vector3 goVector)
{
	position end = start + goVector;

	glBegin(GL_LINES);
	glVertex3f(start[0],start[1],start[2]);
	glVertex3f(end[0],end[1],end[2]);
	glEnd();
}
static vector3
autoColor(int n)
{
	vector3 color;

	switch(n)
	{
		case 0:		color = vector3(0.98f,0.11f,0.21f);	break;
		case 1:		color = vector3(0.15f,0.94f,0.23f);	break;
		case 2:		color = vector3(0.95f,0.64,0.11f);	break;
		case 3:		color = vector3(0.11f,0.93f,0.98f);	break;
		case 4:		color = vector3(0.98f,0.07f,0.94f);	break;
		case 5:		color = vector3(52.0f/255,102.0f/255,1);	break;
		case 6:		color = vector3(1,102.0f/255,0);	break;
		case 7:		color = vector3(153.0f/255, 204.0f/255,0);	break;
		case 8:		color = vector3(1,153.0f/255,204.0f/255);	break;
		default:	color = vector3(1.7f,1.7f,1.7f);	break;
	}

	return color;
}
static void
drawTraiangle(vector3 v1, vector3 v2, vector3 v3)
{
	glBegin(GL_TRIANGLES);
	glVertex3f(v1[0],v1[1],v1[2]);
	glVertex3f(v2[0],v2[1],v2[2]);
	glVertex3f(v3[0],v3[1],v3[2]);
	glEnd();
}
static void
drawTraiangleAutoNormal(vector3 v1, vector3 v2, vector3 v3)
{
	vector3 n = ((v2-v1) * (v3-v1));
	n = normalize(n);

	glBegin(GL_TRIANGLES);
	glNormal3f(n[0], n[1], n[2]);
	glVertex3f(v1[0],v1[1],v1[2]);
	glVertex3f(v2[0],v2[1],v2[2]);
	glVertex3f(v3[0],v3[1],v3[2]);
	glEnd();
}
static void
drawQuad(vector3 v1, vector3 v2, vector3 v3, vector3 v4)
{
	glBegin(GL_QUADS);
	glVertex3f(v1[0],v1[1],v1[2]);
	glVertex3f(v2[0],v2[1],v2[2]);
	glVertex3f(v3[0],v3[1],v3[2]);
	glVertex3f(v4[0],v4[1],v4[2]);
	glEnd();
}
static void inline
draw_XYZ_AXIS(float axisWidth, float axisLength)
{
	glDisable(GL_LIGHTING);

	glLineWidth(axisWidth);

		glBegin(GL_LINES); glColor3f(1,0,0); glVertex3f(0, 0, 0); glVertex3f(axisLength, 0, 0); glEnd();
		glBegin(GL_LINES); glColor3f(0,1,0); glVertex3f(0, 0, 0); glVertex3f(0, axisLength, 0); glEnd();
		glBegin(GL_LINES); glColor3f(0,0,1); glVertex3f(0, 0, 0); glVertex3f(0, 0, axisLength); glEnd();

	glEnable(GL_LIGHTING);
}
static void 
draw_axis()
{
	glLineWidth(1.0);
	glColor3f(1,0,0); glBegin(GL_LINES); glVertex3f(0,0,0); glVertex3f(1,0,0); glEnd();
	glColor3f(0,1,0); glBegin(GL_LINES); glVertex3f(0,0,0); glVertex3f(0,1,0); glEnd();
	glColor3f(0,0,1); glBegin(GL_LINES); glVertex3f(0,0,0); glVertex3f(0,0,1); glEnd();
}
static void
drawArrow(GLUquadricObj* qObj,  m_real length,  m_real headScale)
{
	glPushMatrix();

	glPushMatrix();
	glTranslatef(0, 0, 2*length);
		gluCylinder(qObj, 0.01f, 0.01f, 0.5*length, 20, 20);
		glRotatef(180,0,1,0);
		gluDisk(qObj, 0, 0.01f, 20, 20);
	glPopMatrix();

	glTranslatef(0, 0, 2.5*length);

	glutSolidCone(headScale*0.03f, 3*headScale*0.1, 20, 30);

	glPopMatrix();
}
static void 
drawCylinder(
	GLUquadricObj* qObj, 
	position& pos, 
	m_real height, 
	m_real radTop, 
	m_real radBottom, 
	vector3 color, 
	m_real alpha, 
	int detail)
{
	m_real offset = 1.0f / (2*detail);

	glPushMatrix();

	glEnable(GL_BLEND);
	glEnable(GL_LIGHTING);

	glTranslatef(pos[0], pos[1], pos[2]);
	glRotatef(-90.0f, 1, 0, 0);

	glSetMaterial(color[0], color[1], color[2], alpha);

	// ¹Ù´Ú
	glBegin(GL_TRIANGLE_FAN);
	glVertex3f(0,0,0);
	for(int i=detail; i>=0; i--)
	{
		m_real angle = Deg2Rad(360*(i/(m_real)detail+offset));
		m_real x = radBottom * cos(angle);
		m_real y = radBottom * sin(angle);
		glVertex3f(x, y, 0);
	}
	glEnd();

	// ¸öÅë
	gluCylinder(qObj, radBottom, radTop, height, detail, 2);

	// ¶Ñ²±
	glBegin(GL_TRIANGLE_FAN);
	glVertex3f(0,0,height);
	for(int i=0; i<=detail; i++)
	{
		m_real angle = Deg2Rad(360*(i/(m_real)detail+offset));
		m_real x = radTop * cos(angle);
		m_real y = radTop * sin(angle);
		glVertex3f(x, y, height);
	}
	glEnd();

	glDisable(GL_BLEND);

	glPopMatrix();
}
static void
drawSphere( float x, float y, float z, float radius, int slices = 8, int stacks = 8 )
{
	glPushMatrix();
	glTranslatef( x, y, z );
	//sphere3D( radius );
	glutSolidSphere( radius, slices, stacks );
	glPopMatrix();
}

static void 
drawTree(GLUquadricObj* qObj, position& pos, m_real height, m_real tbRatio, m_real rad, vector3* rc)
{
	m_real topHeight = height * tbRatio;
	m_real bottomHeight = height * (1.0-tbRatio);

	glPushMatrix();

	drawCylinder(qObj, pos, bottomHeight, 0.3*rad, 0.3*rad, vector3(100.f/255, 20.f/255, 20.f/255)+rc[0], 1.0f, 10);

	glTranslatef(0,bottomHeight,0);

	drawCylinder(qObj, pos, topHeight, 0.2*rad, rad, vector3(46/255.f,82/255.f,15/255.f)+rc[1], 1.0f, 10);

	glPopMatrix();
}
static void inline
draw_LessThanSign(float width)
{
	float root3 = sqrtf(3.0);

	glDisable(GL_CULL_FACE);

	glBegin(GL_QUADS);
		glVertex2f(0,0);
		glVertex2f(1,1/root3);
		glVertex2f(1,1/root3-2*width/root3);
		glVertex2f(2*width,0);
	glEnd();

	glBegin(GL_QUADS);
		glVertex2f(0,0);
		glVertex2f(1,-1/root3);
		glVertex2f(1,-1/root3+2*width/root3);
		glVertex2f(2*width,0);
	glEnd();

	glEnable(GL_CULL_FACE);
}
//static vector3
//VectorFromAngles_Refer( float front, float top, bool isToLeft)
//{
//	vector3 res;
//
//	float scaleY, scaleZ, xx;
//
//	float cos_front2	= pow(cos(front), 2.0f);
//	float cos_top2		= pow(cos(top), 2.0f);
//	
//	scaleY = (1.0f - cos_front2)/cos_front2;
//	scaleZ = (1.0f - cos_top2)/cos_top2;
//
//	xx = 1.0f/(1.0f + scaleY + scaleZ);
//
//	res[0] = sqrt(xx);
//	res[1] = sqrt(xx * scaleY);
//	res[2] = sqrt(xx * scaleZ);
//
//	if(front<0)		res[1] *= -1.0f;
//	if(top>0)		res[2] *= -1.0f;
//	if(isToLeft)	
//	{
//		if(front > M_PI/2.0f || top > M_PI/2.0f)	res[0] *= -1.0f;	
//	}
//	else
//	{
//		if(front < M_PI/2.0f || top < M_PI/2.0f)	res[0] *= -1.0f;	
//	}
//	
//	return res;
//}
//static quater
//calcQuaterFromAngles_Refer( float front, float top, bool isToLeft)
//{
//	quater des, topNfront, twistQ;
//	vector3 before, after;
//
//	before = VectorFromAngles_Refer(0, 0, isToLeft);
//	after = VectorFromAngles_Refer(front, top, isToLeft);
//
//	topNfront.FromVectors(before, after);
//
//	des = topNfront;
//
//	return des;
//}


static void
cleanFloatStarVector(vector<float*>& v)
{
	for(int i=0; i<(int)v.size(); i++) delete[] v[i];

	v.clear();
}

static void
clean_copy_floatStar_STL_vector(int size, vector<float*>& v, const vector<float*>& _v)
{
	cleanFloatStarVector(v);

	for(int i=0; i<(int)_v.size(); i++)
	{
		float* pp = new float[size];
		
		for(int j=0; j<size; j++) pp[j] = _v[i][j];

		v.push_back(pp);
	}
}

inline static void
ASSERT(bool exp)
{
	if(!exp)
	{
		cerr << "Assertion fail!" << endl;
		cin.get(); cin.get();
	}
}


namespace util
{

template <typename FSTREAM>
inline bool fileOpenWrapper(FSTREAM& f,  char* name)
{
	if(name == NULL)
	{
		std::cout << "---------------------------------------------------------" << std::endl;
		std::cout << "File open error : null file name" << std::endl;
		std::cout << "---------------------------------------------------------" << std::endl;

		return false;
	}

	if(f.is_open())
	{
		f.close();
		f.clear();
	}
	f.open(name);

	if(f.fail())
	{
		std::cout << "---------------------------------------------------------" << std::endl;
		std::cout << "File open error : " << name << std::endl;
		std::cout << "---------------------------------------------------------" << std::endl;

		return false;
	}

	return true;
}

template <typename FSTREAM>
inline bool fileOpenWrapper(FSTREAM& f,  string name)
{
	if(f.is_open()) f.close();
	f.open(name);

	if(f.fail())
	{
		std::cout << "---------------------------------------------------------" << std::endl;
		std::cout << "File open error : " << name << std::endl;
		std::cout << "---------------------------------------------------------" << std::endl;

		return false;
	}

	return true;
}

template <typename FSTREAM>
inline void fileCloseWrapper(FSTREAM& f)
{
	if(f.is_open())
	{
		f.clear();
		f.close();
	}
}

class LOG
{
	std::ofstream flog;

	inline void initialize(char *str)
	{
		char buf[256];
		std::ofstream fout;
			
		sprintf(buf, "log/%s", str);
		fileOpenWrapper(fout, buf);
	}

	inline void pushLog(int id, char* str)
	{
		flog << str << std::endl;
	}
	inline void finalize()
	{
		flog.close();
	}
};

/*********************************************************************
*
*********************************************************************/
class SPLINE
{
	friend class SPLINE;

private:
	SISLCurve *pc;
	int dim;
	int order;
	double geomResolution;

public:

	SPLINE() { pc = NULL; dim = 3; order=4; geomResolution = 1.0e-6; };
	SPLINE(int dimension, int order, float geometric_resolution = 0.0001)
	{
		assert(order >= dim+1);

		pc = NULL;
		dim = dimension;
		this->order = order;
		geomResolution = geometric_resolution;
	};
	~SPLINE(){ destoryCurve(); };

	inline void destoryCurve()
	{
		if(pc) freeCurve(pc);
	}

	/**************************************************************************/
	/* description : generate a basic b-spline curve from given control points
	/* argument1 : control points of the curve (p1, p2, ... pn)
	/* return : true if success, false if fail
	/**************************************************************************/
	bool genCurve(const std::vector<double>& ctrpnts, bool endPointInterp=false);

	/**************************************************************************/
	/* description : generate a interpolated b-spline curve from given control points
	/* argument1 : control points of the curve (p1, p2, ... pn) or (p1 t2 p2 t3 p3 ... tn pn)
	/* argument2 : tangential is given?
	/* return : true if success, false if fail
	/**************************************************************************/
	bool genInterpCurve(const std::vector<double>& ctrpnts, bool isOrdinary=false);

	/**************************************************************************/
	/* description : generate a interpolated b-spline curve from given control points
	/* argument1 : control points of the curve (p1, p2, ... pn) or (p1 t2 p2 t3 p3 ... tn pn)
	/* argument2 : tangential is given?
	/* return : true if success, false if fail
	/**************************************************************************/
	bool genCatMullRommLikeCurve(const std::vector<double>& ctrpnts);

	/**************************************************************************/
	/* description : generate a blending b-spline curve from given two curves
	/* argument1 : a spline curve1 
	/* argument2 : parameter of the curve1
	/* argument3 : a spline curve2
	/* argument4 : parameter of the curve2
	/* return : true if success, false if fail
	/**************************************************************************/
	bool genBlendingCurve(SPLINE *curve1, float param1, SPLINE *curve2, float param2);

	/**************************************************************************/
	/* description : generate a fillet b-spline curve from given two curves
	/* argument1 : a spline curve1 
	/* argument2 : a spline curve2
	/* return : true if success, false if fail
	/**************************************************************************/
	bool genFilletCurve(SPLINE *curve1, SPLINE *curve2);

	/**************************************************************************/
	/* description : get a parameter range of the curve
	/* argument1 : two pointers to be given
	/* return : start & end parameters range of the curve
	/**************************************************************************/
	void getParameterRange(float& start, float& end);

	/**************************************************************************/
	/* description : get a parameter of a point on the curve
	/* argument1 : a point on the curve
	/* return : a parameter of given a point on the curve
	/**************************************************************************/
	float getParameter(const std::vector<double>& p);

	/**************************************************************************/
	/* description : get a point on the curve from given a parameter
	/* argument1 : a paramter on the curve (0~1)
	/* return : a point of given a paramter on the curve
	/**************************************************************************/
	std::vector<double> getPoint(double u);

	/**************************************************************************/
	/* description : get a closest point on the curve from given a point
	/* argument1 : a point in the space
	/* return : a closest point on the curve
	/**************************************************************************/
	std::vector<double> getClosestPoint(const std::vector<double>& p);

	/**************************************************************************/
	/* description : get tangents on the curve from given a point
	/* argument1 : a point on the curve
	/* argument2 : needed derivative
	/* return : tangents on the point
	/* (if der 0: point, 1: point and 1st derivative, 2: point and 1st derivative and 2st derivative)
	/**************************************************************************/
	std::vector<double> getTangent(const std::vector<double>& p, int der);

	/**************************************************************************/
	/* description : get frent frame on the curve from given a point
	/* argument1 : a point on the curve
	/* return : frent frame of the curve (tangent, normal, binomal)
	/**************************************************************************/
	std::vector<double> getFrentFrame(const std::vector<double>& p);

	/**************************************************************************/
	/* description : get the length of the curve
	/* return : length of the curve
	/**************************************************************************/
	float getCurveLength();

	/**************************************************************************/
	/* description : get the curvature from given a point on the curve
	/* argument1 : a point on the curve
	/* return : curvature on the point
	/**************************************************************************/
	float getCurvature(const std::vector<double>& p);
};

inline void error_message( char* ch)
{
	cerr << "------------------------------------------" << endl;
	cerr << "- Error ----------------------------------" << endl;
	cerr << "------------------------------------------" << endl;
	cerr << "- " << ch << endl;
	cerr << "------------------------------------------" << endl;

	cin.get(); cin.get(); exit(0);
}

inline void error_message( string ch)
{
	cerr << "------------------------------------------" << endl;
	cerr << "- Error ----------------------------------" << endl;
	cerr << "------------------------------------------" << endl;
	cerr << "- " << ch << endl;
	cerr << "------------------------------------------" << endl;

	cin.get(); cin.get(); exit(0);
}

inline void warinig_message( char* ch)
{
	cerr << "------------------------------------------" << endl;
	cerr << "- Warinig --------------------------------" << endl;
	cerr << "------------------------------------------" << endl;
	cerr << "- " << ch << endl;
	cerr << "------------------------------------------" << endl;
}

inline void warinig_message( string ch)
{
	cerr << "------------------------------------------" << endl;
	cerr << "- Warinig --------------------------------" << endl;
	cerr << "------------------------------------------" << endl;
	cerr << "- " << ch << endl;
	cerr << "------------------------------------------" << endl;
}

inline void assert_message(bool exp,  char* ch)
{
	if(!exp) error_message(ch);
}

inline void assert_message(bool exp,  string ch)
{
	if(!exp) error_message(ch);
}

inline string getNext(ifstream& fin)
{
	string token;
	char buf[1024];

	fin >> token;

	while(token == "//")
	{
		fin.getline(buf, 1023);
		fin >> token;
	}

	return token;
}

inline void ignoreComment(ifstream& fin)
{
	char buf[1024];
	char c;

	while(true)
	{
		if(fin.peek()=='/')
		{
			fin.get(c);
			if(fin.peek()=='/')
			{
				fin.getline(buf, 1023);
				continue;
			}
			else
			{
				fin.putback(c);
				break;
			}
		}
		else break;
	}
}

inline tm* getLocalTime()
{
	time_t current;

	time(&current);

	return localtime(&current);
}

inline string getLocalTimeString()
{
	string date;

	tm* clock = getLocalTime();

	ostringstream osstr;

	osstr << "(" << 1900+clock->tm_year << "-";
	osstr << 1+clock->tm_mon << "-" << clock->tm_mday << " ";
	osstr << clock->tm_hour << ";" << clock->tm_min << ";" << clock->tm_sec << ")";

	return osstr.str();
}

inline void tokenTime(char* time,  char* buf)
{
	 int size = strlen(buf);
	int s_idx, e_idx;

	for(int i=0; i<size-1; i++)
	{
		if( buf[i] == '(' ) s_idx = i;
		if( buf[i] == ')' ) e_idx = i;
	}

	for(int i=s_idx; i<=e_idx; i++)
		time[i-s_idx] = buf[i];
	time[e_idx-s_idx+1] = '\0';
}

inline void deleteClass(void* c_p)
{
	if(c_p!=NULL)
	{
		delete c_p;
		c_p = NULL;
	}
}

}
