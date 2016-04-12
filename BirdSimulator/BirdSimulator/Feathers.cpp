#include "Coeff.h"
#include "SystemState.h"
#include "Feathers.h"
#include "util.h"
#include "Controller.h"

extern Coeff*			coeff;
extern SystemState*		sysState;
extern Controller*		controller;

m_real	_interpolate(m_real t, m_real a, m_real b)
{
	return	(1.0 - t) * a + t * b;
}

m_real	liftCoefficient2(m_real theta_radian)
{
	const	m_real	cutoff = 85;

	const	m_real	xa = -90,	ya = -0.2;
	const	m_real	xb = -10,	yb = -0.4;
	const	m_real	xc =  30,	yc =  3.0;
	const	m_real	xd =  90,	yd =  2.3;

	m_real	theta = theta_radian * 180.0 / M_PI;

	if (fabs(theta) > cutoff)	return	0;

	if (theta < xb)	return	_interpolate((theta - xa) / (xb - xa), ya, yb);
	if (theta < xc)	return	_interpolate((theta - xb) / (xc - xb), yb, yc);
	else			return	_interpolate((theta - xc) / (xd - xc), yc, yd);
}

m_real	dragCoefficient2(m_real theta_radian)
{
	return	fabs(sin(theta_radian)) + 0.05;
}

float	// 새의 몸통이 lifting되는 정도. 그래프의 폭을 크게 하면, 더 위로 들려올려진다.
liftCoefficient(float radian)
{
	const	m_real	cutoff = 85;

	float angle = RadToAng(radian);

	if(angle > cutoff)	return 0;

	float re = 0.0f;

	// savedParam, optimization 날개짓
	point2 x0(-90,-0.1f);
	point2 x1(-15,-1.5f);
	point2 x2(-5,0.0f);
	point2 x3(18,5.0f);
	point2 x4(90,2.2f);

	/*point2 x0(-90,-0.2f);
	point2 x1(-15,-0.4f);
	point2 x2(-10,0.0f);
	point2 x3(30,3.5f);
	point2 x4(90,2.3f);*/

	if(angle < x1.x)
	{
		float t = (angle - x0.x)/(x1.x - x0.x);
		float p[4] = {x0.y, x0.y, x1.y, x2.y};
		re = (((-p[0]+3*p[1]-3*p[2]+p[3])*t*t*t+(2*p[0]-5*p[1]+4*p[2]-p[3])*t*t+(-p[0]+p[2])*t+2*p[1])/(float)2);
	}
	else if(angle >= x1.x && angle < x2.x)		
	{
		float t = (angle - x1.x)/(x2.x - x1.x);
		float p[4] = {x0.y, x1.y, x2.y, x3.y};
		re = (((-p[0]+3*p[1]-3*p[2]+p[3])*t*t*t+(2*p[0]-5*p[1]+4*p[2]-p[3])*t*t+(-p[0]+p[2])*t+2*p[1])/(float)2);
	}
	else if(angle >= x2.x && angle < x3.x)		
	{
		float t = (angle - x2.x)/(x3.x - x2.x);
		float p[4] = {x1.y, x2.y, x3.y, x4.y};
		re = (((-p[0]+3*p[1]-3*p[2]+p[3])*t*t*t+(2*p[0]-5*p[1]+4*p[2]-p[3])*t*t+(-p[0]+p[2])*t+2*p[1])/(float)2);
	}
	else if(angle >= x3.x)
	{
		float t = (angle - x3.x)/(x4.x - x3.x);
		float p[4] = {x2.y, x3.y, x4.y, x4.y};
		re = (((-p[0]+3*p[1]-3*p[2]+p[3])*t*t*t+(2*p[0]-5*p[1]+4*p[2]-p[3])*t*t+(-p[0]+p[2])*t+2*p[1])/(float)2);
	}

	return re;
}
float	// 새의 몸통을 drag함으로써, 속도에 영향을 미친다. 값이 커질수록 drag force가 커지면서, slow down 시킨다.
dragCoefficient(float radian)
{
	float angle = RadToAng(radian);
	
	float re = 0.0f;

	// savedParam, optimization 날개짓
	point2 y0(-90, 1.0f);
	point2 y1(0, 0.05f);
	point2 y2(90, 1.0f);		

	if(angle < y1.x)
	{
		re = (1.0f-cos(radian)) * (y0.y - y1.y) + y1.y;
	}
	else
	{
		re = (1.0f-cos(radian)) * (y2.y - y1.y) + y1.y;
	}

	return re;
}

void
EachPiece::updateBasicInform()
{
	// normal
	vector3	ua = pos[1] - pos[0];
	vector3 ub = pos[2] - pos[0];		
	
	normal = ua*ub;
	normal = normalize(normal);

	// center of piece
	position beforeCenterOfPiece = centerOfPiece;
	//centerOfPiece.setZero();
	centerOfPiece = COM(pos[0], pos[1], pos[2]);

	// velocity
	velocity = -(centerOfPiece - beforeCenterOfPiece) / sysState->TIME_STEP;

	// area
	area = triangleArea(pos[0], pos[1], pos[2]);
}
void
EachPiece::calcAeroForce(position bodyPos, float areaRatio)
{
	dragD = normalize(velocity);

	if(SAME_VALUE((dragD % normal), 1.0f))		liftD = vector3();
	else										liftD = normalize(dragD * normal) * dragD;
	
	aoa = M_PI/2.0f - acos((normal % velocity) / velocity.length() );

	if(velocity.length() == 0)	aoa = 0;

	float rho	= AirDensity;
	float vv	= velocity % velocity;

	dragForce	= 0.5f * rho * vv * areaRatio * area * dragCoefficient(aoa) * dragD;
	liftForce	= 0.5f * rho * vv * areaRatio * area * liftCoefficient(aoa) * liftD;
	//dragForce	= 0.5f * rho * vv * areaRatio * area * dragCoefficient2(aoa) * dragD;
	//liftForce	= 0.5f * rho * vv * areaRatio * area * liftCoefficient2(aoa) * liftD;
	netForce	= dragForce + liftForce;

	//cout << "(" << cDrag << "," << areaRatio << "," << area << "," << nn << "," << dragD.length() << ")" << endl;
	//cout << "(" << 	dragForce.length() << "," << liftForce.length() << ")" << endl;
	momentArmToBody	= centerOfPiece - bodyPos;
}
void
EachFeather::setInformation(float len, float lenR, float wid, float widR, float linkL, float linkR, float sAng, float zz)
{
	length			= len/100.0f;			// cm to m
	lengthRatio		= lenR;
	width			= wid/100.0f;			// cm to m
	widthRatio		= widR;
	linkLength		= linkL;
	linkRatio		= linkR;
	z_trans			= zz;

	twistAng		= 0.0f;
	initSpreadAng	= sAng;

	initialize();
}
void
EachFeather::initialize()
{
	if(isPrimary())
	{
		constructPrimaryFeather();	
	}
	else if(isSecondary())
	{
		constructSecondaryFeather();
		
	}
	else if(isTertiary())
	{
		constructTertiaryFeather();
		
	}
	else if(isTail())
	{
		constructTailFeather();
	}
	else
	{
		cout << "Given feather is not a any kinds of feather : It cannot be happen..." << endl;
		cin.get();
	}
}
void
EachFeather::constructFeather(bool left, const char* name, const vector3 scale)
{

//#define EXPORT_FEATHER

#ifdef EXPORT_FEATHER

	// each pieces vertical(long direction) length
	float oneLeng = length*(1.0f-lengthRatio)/(float)((numOfPiece-1)/2.0f);

	for(int i=0;i<numOfPiece;i++)
	{	
		// we think that a piece is located at start joint
		// pos(0,0,-length) is (tri's cusp point and  also link body's origin) and make each pieces along positive z axis 

		float x_0 = -width * (1.0f - widthRatio) * scale[0];
		float x_1 = 1.0f * (width * widthRatio) * scale[0];
		float z_0 = - oneLeng*(i/2+1) * scale[2];
		float z_1 = - oneLeng*(i/2+0) * scale[2];

		// with feather ppt
		//                   --
		//                   z_1
		//                   --
		//                   --
		//                   z_0
		//                   --
		// ||||||x_1|||||||x_0||

		// last tri piece
		if(i==numOfPiece-1)
		{ 
			init[i]->pos[0][axis_X]	= 0.0f;											init[i]->pos[0][axis_Z] = - length;
			init[i]->pos[1][axis_X] =  -width * (1.0f - widthRatio);				init[i]->pos[1][axis_Z] = - length + lengthRatio * length;
			init[i]->pos[2][axis_X] =  1.0f * (width * widthRatio);					init[i]->pos[2][axis_Z] = - length + lengthRatio * length;

			init[i]->area = width * ( length * lengthRatio) * 0.5f;
		}
		// interanl quad piece
		else
		{
			if(left)
			{
				switch(i%4)
				{
				case 0:
					init[i]->pos[0][axis_X] = x_0; init[i]->pos[0][axis_Z] = z_1;
					init[i]->pos[1][axis_X] = x_1; init[i]->pos[1][axis_Z] = z_1;
					init[i]->pos[2][axis_X] = x_0; init[i]->pos[2][axis_Z] = z_0;
					break;
				case 1:
					init[i]->pos[0][axis_X] = x_1; init[i]->pos[0][axis_Z] = z_0;
					init[i]->pos[1][axis_X] = x_0; init[i]->pos[1][axis_Z] = z_0;
					init[i]->pos[2][axis_X] = x_1; init[i]->pos[2][axis_Z] = z_1;
					break;
				case 2:
					init[i]->pos[0][axis_X] = x_1; init[i]->pos[0][axis_Z] = z_1;
					init[i]->pos[1][axis_X] = x_1; init[i]->pos[1][axis_Z] = z_0;
					init[i]->pos[2][axis_X] = x_0; init[i]->pos[2][axis_Z] = z_1;
					break;
				case 3:
					init[i]->pos[0][axis_X] = x_0; init[i]->pos[0][axis_Z] = z_0;
					init[i]->pos[1][axis_X] = x_0; init[i]->pos[1][axis_Z] = z_1;
					init[i]->pos[2][axis_X] = x_1; init[i]->pos[2][axis_Z] = z_0;
					break;
				}
			}
			else
			{
				switch(i%4)
				{
				case 0:
					init[i]->pos[0][axis_X] = x_1; init[i]->pos[0][axis_Z] = z_1;
					init[i]->pos[1][axis_X] = x_1; init[i]->pos[1][axis_Z] = z_0;
					init[i]->pos[2][axis_X] = x_0; init[i]->pos[2][axis_Z] = z_1;
					break;
				case 1:
					init[i]->pos[0][axis_X] = x_0; init[i]->pos[0][axis_Z] = z_0;
					init[i]->pos[1][axis_X] = x_0; init[i]->pos[1][axis_Z] = z_1;
					init[i]->pos[2][axis_X] = x_1; init[i]->pos[2][axis_Z] = z_0;
					break;
				case 2:
					init[i]->pos[0][axis_X] = x_0; init[i]->pos[0][axis_Z] = z_1;
					init[i]->pos[1][axis_X] = x_1; init[i]->pos[1][axis_Z] = z_1;
					init[i]->pos[2][axis_X] = x_0; init[i]->pos[2][axis_Z] = z_0;
					break;
				case 3:
					init[i]->pos[0][axis_X] = x_1; init[i]->pos[0][axis_Z] = z_0;
					init[i]->pos[1][axis_X] = x_0; init[i]->pos[1][axis_Z] = z_0;
					init[i]->pos[2][axis_X] = x_1; init[i]->pos[2][axis_Z] = z_1;
					break;
				}
			}

			init[i]->area = width * oneLeng * 0.5f;
		}
	
		// after making each pieces, move it to link body's edge(skeleton) by link ratio, z_trans(body's second axis length/2)

		//for(int j=0;j<init[i]->nn;j++)
		//{
		//	//init[i]->pos[j][axis_Z] *= 2.0f;
		//	init[i]->pos[j][axis_X] += (-linkLength/2.0f + linkLength*linkRatio);
		//	init[i]->pos[j][axis_Z] += z_trans;
		//}
		
		// calculate normal, center of piece, relative velocity ...

		init[i]->updateBasicInform();
		
		// assign this to previous and current feather state
		//pprev[i]->assignState( init[i] );
		//prev[i]->assignState( init[i] );
		//current[i]->assignState( init[i] );
	}

	endTriArea = width * ( length * lengthRatio) * 0.5f;
	area = width * oneLeng * (numOfPiece-1) * 0.5f + width * ( length * lengthRatio) * 0.5f;

	/* export mesh */

	ofstream fout;

	cout << "-------------------------------------------------" << endl;
	cout << "----Export Feather Mesh : " << name << endl;
	cout << "-------------------------------------------------" << endl;

	if(util::fileOpenWrapper(fout, name))
	{
		// nodes
		int idxElement = 0;
		fout << "Nodes " << (int)init.size()+2 << endl;
		for(int i=0; i<numOfPiece; i++)
		{
			position* p = init[i]->pos;
			
			// end tri
			if( i==numOfPiece-1 )
			{
				WRITE_ARRAY3(fout, p[2]);
				WRITE_ARRAY3(fout, p[1]);
				WRITE_ARRAY3(fout, p[0]);
			}
			else
			{
				if(left)
				{
					switch(i%4)
					{
					case 0:
						WRITE_ARRAY3(fout, p[1]);
						WRITE_ARRAY3(fout, p[0]);
						break;
					case 2:
						WRITE_ARRAY3(fout, p[0]);
						WRITE_ARRAY3(fout, p[2]);
						break;
					}
				}
				else
				{
					switch(i%4)
					{
					case 0:
						WRITE_ARRAY3(fout, p[0]);
						WRITE_ARRAY3(fout, p[2]);
						break;
					case 2:
						WRITE_ARRAY3(fout, p[1]);
						WRITE_ARRAY3(fout, p[0]);
						break;
					}
				}
			}
		}
		// index
		fout << "Elements " << numOfPiece << endl;
		for(int i=0; i<numOfPiece; i++)
		{
			int a0 = (i/2)*2 + 0;
			int a1 = (i/2)*2 + 1;
			int a2 = (i/2)*2 + 2;
			int a3 = (i/2)*2 + 3;

			// end tri
			if( i==numOfPiece-1 )
			{
				fout << a2 << " " << a1 << " " << a0 << endl;
			}
			else
			{
				if(left)
				{
					switch(i%4)
					{
					case 0:	fout << a1 << " " << a0 << " " << a3 << endl; break;
					case 1:	fout << a2 << " " << a3 << " " << a0 << endl; break;
					case 2:	fout << a0 << " " << a2 << " " << a1 << endl; break;
					case 3:	fout << a3 << " " << a1 << " " << a2 << endl; break;
					}
				}
				else
				{
					switch(i%4)
					{
					case 0:	fout << a0 << " " << a2 << " " << a1 << endl; break;
					case 1:	fout << a3 << " " << a1 << " " << a2 << endl; break;
					case 2:	fout << a1 << " " << a0 << " " << a3 << endl; break;
					case 3:	fout << a2 << " " << a3 << " " << a0 << endl; break;
					}
				}
			}
		}
		// edges
		fout << "Edges " << 2*(int)init.size()+1 << endl;
		for(int i=0; i<numOfPiece; i++)
		{
			int a_2 = (i/2)*2 + -2;
			int a_1 = (i/2)*2 + -1;
			int a0	= (i/2)*2 + 0;
			int a1	= (i/2)*2 + 1;
			int a2	= (i/2)*2 + 2;
			int a3	= (i/2)*2 + 3;
			int empty = -1;
		
			CLAMP_FLOOR(a_2, -1);
			CLAMP_FLOOR(a_1, -1);

			if(i==numOfPiece-1)
			{
				if(left)
				{
					switch(i%4)
					{
					case 0:
						fout << a0 << " " << a1 << " " << a2 << " " << a_1 << endl;
						fout << a0 << " " << a2 << " " << a1 << " " << empty << endl;
						fout << a1 << " " << a2 << " " << a0 << " " << empty << endl;
						break;
					case 2:
						fout << a0 << " " << a1 << " " << a2 << " " << a_2 << endl;
						fout << a0 << " " << a2 << " " << a1 << " " << empty << endl;
						fout << a1 << " " << a2 << " " << a0 << " " << empty << endl;
						break;
					}
				}
				else
				{
					switch(i%4)
					{
					case 0:
						fout << a0 << " " << a1 << " " << a2 << " " << a_2 << endl;
						fout << a0 << " " << a2 << " " << a1 << " " << empty << endl;
						fout << a1 << " " << a2 << " " << a0 << " " << empty << endl;
						break;
					case 2:
						fout << a0 << " " << a1 << " " << a2 << " " << a_1 << endl;
						fout << a0 << " " << a2 << " " << a1 << " " << empty << endl;
						fout << a1 << " " << a2 << " " << a0 << " " << empty << endl;
						break;
					}
				}
			}
			else
			{
				if(left)
				{
					switch(i%4)
					{
					case 0:
						fout << a0 << " " << a1 << " " << a3 << " " << a_1 << endl;
						fout << a0 << " " << a3 << " " << a1 << " " << a2 << endl;
						fout << a0 << " " << a2 << " " << a3 << " " << empty << endl;
						fout << a1 << " " << a3 << " " << a0 << " " << empty << endl;
						break;
					case 2:
						fout << a0 << " " << a1 << " " << a2 << " " << a_2 << endl;
						fout << a1 << " " << a2 << " " << a0 << " " << a3 << endl;
						fout << a0 << " " << a2 << " " << a1 << " " << empty << endl;
						fout << a1 << " " << a3 << " " << a2 << " " << empty << endl;
						break;
					}
				}
				else
				{
					switch(i%4)
					{
					case 0:
						fout << a0 << " " << a1 << " " << a2 << " " << a_2 << endl;
						fout << a1 << " " << a2 << " " << a0 << " " << a3 << endl;
						fout << a0 << " " << a2 << " " << a1 << " " << empty << endl;
						fout << a1 << " " << a3 << " " << a2 << " " << empty << endl;
						break;
					case 2:
						fout << a0 << " " << a1 << " " << a3 << " " << a_1 << endl;
						fout << a0 << " " << a3 << " " << a1 << " " << a2 << endl;
						fout << a0 << " " << a2 << " " << a3 << " " << empty << endl;
						fout << a1 << " " << a3 << " " << a0 << " " << empty << endl;
						break;
					}
				}
				
			}
		}

		util::fileCloseWrapper(fout);
	}
#else
	/* load mesh */

	loadFeatherMesh(name, scale);
	area = 0.0f;
	areaRatio = 1.0f;

	for(int i=0; i<mesh.getNumElements(); i++)
	{
		int idx;

		// load nodes
		for(int j=0; j<3; j++)
		{
			idx = (int)mesh.element[i][j];
			init[i]->pos[j] = mesh.node[idx];
		}

		// compute area
		init[i]->area = triangleArea(init[i]->pos[0], init[i]->pos[1], init[i]->pos[2]);
		area += init[i]->area;

		// calculate normal, center of piece, relative velocity ...
		init[i]->updateBasicInform();

		// assign this to previous and current feather state
		*(pprev[i]) = *(init[i]);
		*(prev[i]) = *(init[i]);
		*(current[i]) = *(init[i]);
	}
#endif

	linkToFeatherTransf.setRotation(quater(1,0,0,0));
	linkToFeatherTransf.setTranslation( vector3( (-linkLength/2.0f + linkLength*linkRatio), 0, z_trans ) );

	//cout << "sol : " << ((1-lengthRatio) * length * width) + (lengthRatio * length * width * 0.5) << endl;
	//cout << "area : " << area << endl;
}
void
EachFeather::constructPrimaryFeather()
{
	char name[256];
	sprintf(name, "%s%d.mesh", isLeft() ? meshNamePrimary_L : meshNamePrimary_R, orderInGroup);

#ifdef PEACOCK
	constructFeather(isLeft(), name, vector3(1,1,1.5));
#else
	constructFeather(isLeft(), name, vector3(1,1,1));
#endif
}
void
EachFeather::constructSecondaryFeather()
{
	char name[256];
	sprintf(name, "%s%d.mesh", isLeft() ? meshNameSecondary_L : meshNameSecondary_R, orderInGroup);

#ifdef PEACOCK
	constructFeather(isLeft(), name, vector3(1,1,1.5));
#else
	constructFeather(isLeft(), name, vector3(1,1,1));
#endif
}
void
EachFeather::constructTertiaryFeather()
{
	char name[256];
	sprintf(name, "%s%d.mesh", isLeft() ? meshNameTertiary_L : meshNameTertiary_R, orderInGroup);

#ifdef PEACOCK
	constructFeather(isLeft(), name, vector3(1,1,1.5));
#else
	constructFeather(isLeft(), name, vector3(1,1,1));
#endif
}
void
EachFeather::constructTailFeather()
{
	char name[256];
	sprintf(name, "%s%d.mesh", ((orderInGroup/(float)numSibling)<0.5f) ? meshNameTail_L : meshNameTail_R, orderInGroup);

#ifdef PEACOCK
	constructFeather(((orderInGroup/(float)numSibling)<0.5f), name, vector3(1,1,10));
#else
	constructFeather(((orderInGroup/(float)numSibling)<0.5f), name, vector3(1,1,1));
#endif
	
}
void
EachFeather::loadFeatherMesh(const char* name, vector3 scale)
{
	ifstream fin;

	//cout << "-------------------------------------------------" << endl;
	//cout << "----Load Feather Mesh : " << name << endl;
	//cout << "-------------------------------------------------" << endl;

	if(util::fileOpenWrapper(fin, name))
	{
		string tokenName;
		position p;

		// Nodes
		int num_nodes;
		fin >> tokenName >> num_nodes;
		//cout << "tokenName : " << tokenName << " ( " << num_nodes << " ) " << endl;

		for(int i=0; i<num_nodes; i++)
		{
			READ_ARRAY3(fin, p);
			p[0] = scale[0] * p[0];
			p[1] = scale[1] * p[1];
			p[2] = scale[2] * p[2];
			mesh.node.push_back(p);
		}

		// Elements
		int num_elements;
		fin >> tokenName >> num_elements;
		//cout << "tokenName : " << tokenName << " ( " << num_elements << " ) " << endl;

		for(int i=0; i<num_elements; i++)
		{
			READ_ARRAY3(fin, p);
			mesh.element.push_back(p);
		}

		// Edges
		int num_edges;
		fin >> tokenName >> num_edges;
		//cout << "tokenName : " << tokenName << " ( " << num_edges << " ) " << endl;

		for(int i=0; i<num_edges; i++)
		{
			Mesh::Edge edge;
			READ_ARRAY4(fin, edge.e);
			mesh.edge.push_back(edge);
		}

		// Node to element info
		mesh.computeNodeID2element();
		mesh.computeTotalArea();

		util::fileCloseWrapper(fin);
	}

	//cout << "-------------------------------------------------" << endl;
}
void
EachFeather::loadPrimaryFeatherMesh()
{
	char name[256];
	sprintf(name, "%s%d.mesh", isLeft() ? meshNamePrimary_L : meshNamePrimary_R, orderInGroup);

	loadFeatherMesh(name);
}
void
EachFeather::loadSecondaryFeatherMesh()
{
	char name[256];
	sprintf(name, "%s%d.mesh", isLeft() ? meshNameSecondary_L : meshNameSecondary_R, orderInGroup);

	loadFeatherMesh(name);
}
void
EachFeather::loadTertiaryFeatherMesh()
{
	char name[256];
	sprintf(name, "%s%d.mesh", isLeft() ? meshNameTertiary_L : meshNameTertiary_R, orderInGroup);

	loadFeatherMesh(name);
}
void
EachFeather::loadTailFeatherMesh()
{
	char name[256];
	sprintf(name, "%s%d.mesh", ((orderInGroup/(float)numSibling)<0.5f) ? meshNameTail_L : meshNameTail_R, orderInGroup);

	loadFeatherMesh(name);
}
void
EachFeather::updateLocalState()
{
	updatePosition();
	updateRotation();
}
void
EachFeather::updatePosition()
{
	for(int i=0;i<numOfPiece;i++)	
	{
		*(pprev[i])		= *(prev[i]);
		*(prev[i])		= *(current[i]);
		*(current[i])	= *(init[i]);
	}
}
void
EachFeather::updateRotation()
{
	float transZ;
	quater rotAngle;

	//// move current feather(==init) to link body's origin
	//for(int i=0;i<numOfPiece;i++)
	//for(int j=0;j<init[i]->nn;j++)	
	//{
	//	current[i]->pos[j][axis_X] -= (-linkLength/2.0f + linkLength*linkRatio);
	//	current[i]->pos[j][axis_Z] -= z_trans;
	//}

	//WRITE_ARRAY3(cout, current[0]->pos[0]);

	if(isTail())	// spread -> bend
	{
		// spread (y axis)
		quater spreadAngle = EulerAngle2Quater(vector3(0,initSpreadAng+spreadAng,0));
		quater bendAngle = EulerAngle2Quater(vector3(-bendAng[0],0,0));

		/*for(int i=0;i<numOfPiece;i++)
		{
			for(int j=0;j<current[i]->nn;j++)
			{
				current[i]->pos[j]			= rotate(spreadAngle, current[i]->pos[j]);
			}
		}*/

		//// bend (x axis)
		//for(int k=numOfPiece-1;k>=0;k--)
		//{
		//	rotAngle = EulerAngle2Quater(vector3(-bendAng[k],0,0));

		//	if(current[k]->nn == 4)			transZ = (current[k]->pos[0][axis_Z]*(1.0f-widthRatio) + current[k]->pos[3][axis_Z]*widthRatio);
		//	else if(current[k]->nn == 3)	transZ = (current[k]->pos[1][axis_Z]*(1.0f-widthRatio) + current[k]->pos[2][axis_Z]*widthRatio);
 

		//	for(int i=k;i<numOfPiece;i++)
		//	{
		//		for(int j=0;j<current[i]->nn;j++)
		//		{
		//			current[i]->pos[j][axis_Z]	-= transZ;
		//			current[i]->pos[j]			= rotate(rotAngle, current[i]->pos[j]);
		//			current[i]->pos[j][axis_Z]	+= transZ;
		//		}
		//	}
		//}

		linkToFeatherTransf.setRotation(bendAngle*spreadAngle);
	}
	else	// bend -> twist -> spread
	{
		//// bend (x axis)
		//for(int k=numOfPiece-1;k>=0;k--)
		//{
		//	rotAngle = EulerAngle2Quater(vector3(-bendAng[k],0,0));

		//	if(current[k]->nn == 4)			transZ = (current[k]->pos[0][axis_Z]*(1.0f-widthRatio) + current[k]->pos[3][axis_Z]*widthRatio);
		//	else if(current[k]->nn == 3)	transZ = (current[k]->pos[1][axis_Z]*(1.0f-widthRatio) + current[k]->pos[2][axis_Z]*widthRatio);
 
		//	// cumulate bend angle
		//	for(int i=k;i<numOfPiece;i++)
		//	{
		//		for(int j=0;j<current[i]->nn;j++)
		//		{
		//			current[i]->pos[j][axis_Z]	-= transZ;
		//			current[i]->pos[j]			= rotate(rotAngle, current[i]->pos[j]);
		//			current[i]->pos[j][axis_Z]	+= transZ;
		//		}
		//	}
		//}

		//// twist (z axis)
		//rotAngle = EulerAngle2Quater( vector3( 0, 0, twistAng ) );
		////if(isLeft())	rotAngle = EulerAngle2Quater( vector3( 0, 0, twistAng ) );
		////else			rotAngle = EulerAngle2Quater( vector3( 0, 0, -twistAng ) );

		//for(int i=0;i<numOfPiece;i++)
		//{
		//	for(int j=0;j<current[i]->nn;j++)
		//	{
		//		current[i]->pos[j]			= rotate(rotAngle, current[i]->pos[j]);
		//	}
		//}
		
		// spread (y axis)
		quater spreadAngle	= EulerAngle2Quater(vector3(0,initSpreadAng+spreadAng,0));
		quater bendAngle = EulerAngle2Quater(vector3(-bendAng[0],0,0));

		/*for(int i=0;i<numOfPiece;i++)
		{
			for(int j=0;j<current[i]->nn;j++)
			{
				current[i]->pos[j] = rotate(spreadAngle, current[i]->pos[j]);
			}
		}*/

		linkToFeatherTransf.setRotation(spreadAngle*bendAngle);
	}

	// move current feather(attached to link body's origin) to initial position
	/*for(int i=0;i<numOfPiece;i++)
	for(int j=0;j<init[i]->nn;j++)	
	{
		current[i]->pos[j][axis_X] += (-linkLength/2.0f + linkLength*linkRatio);
		current[i]->pos[j][axis_Z] += z_trans;
	}*/
	linkToFeatherTransf.setTranslation( vector3( (-linkLength/2.0f + linkLength*linkRatio), 0, z_trans ) );
}
void
EachFeather::updateOrientation(bool isLeft)
{
	// calc orientation of each piece
	for(int i=0;i<current.size();i++)
	{
		current[i]->ori = QI;

		if(i%2 == 1)	continue;

		int a, b;	

		if(isLeft)	
		{
			if(i%4 == 0)		{	a = 1;	b = 0;	}
			else				{	a = 0;	b = 2;	}
		}
		else
		{
			if(i%4 == 0)		{	a = 0;	b = 2;	}
			else				{	a = 1;	b = 0;	}
		}

		vector3 trans, ori1[3], ori2[3];

		float ratioA = abs(init[i]->pos[a][axis_X]);
		float ratioB = abs(init[i]->pos[b][axis_X]);

		vector3 it = (position2vector(init[i]->pos[b]) * ratioA + position2vector(init[i]->pos[a]) * ratioB)/(ratioA + ratioB);
		vector3 ct = (position2vector(current[i]->pos[b]) * ratioA + position2vector(current[i]->pos[a]) * ratioB)/(ratioA + ratioB);

		trans = ct - it;

		/*cout << "currA : "; WRITE_ARRAY3(cout, current[i]->pos[a]);
		cout << "currB : "; WRITE_ARRAY3(cout, current[i]->pos[b]);
		cout << "trans : "; WRITE_ARRAY3(cout, trans);*/

		ori1[0] = init[i]->pos[1] - init[i]->pos[0];	
		ori1[1] = init[i]->pos[2] - init[i]->pos[0];	
		ori1[2] = ori1[0] * ori1[1];						

		ori2[0] = current[i]->pos[1] - current[i]->pos[0];	
		ori2[1] = current[i]->pos[2] - current[i]->pos[0];	
		ori2[2] = ori2[0] * ori2[1];	

		for(int j=0;j<3;j++)
		{
			ori1[j] = normalize(ori1[j]);
			ori2[j] = normalize(ori2[j]);
		}

		matrix m_init(ori1[0], ori1[1], ori1[2]);
		matrix m_curr(ori2[0], ori2[1], ori2[2]);

		current[i]->ori = Matrix2Quater( m_init.inverse() * m_curr);

		//cout << "curr : ";	WRITE_ARRAY3(cout, current[i]->pos[0]);
		//cout << "res : ";	WRITE_ARRAY3(cout, init[i]->ori.Rotate(position2vector(init[i]->pos[0]) - it ) + ct);

	}
}

void
EachFeather::updateGlobalState(transf linkTransf, vector3 linkLinVel, vector3 linkAngVel, float areaRatio)
{
	if(cnt==0)
	{
		initTransf	= linkToFeatherTransf * linkTransf;
		curTransf	= linkToFeatherTransf * linkTransf;
	}

	curTransf = linkToFeatherTransf * linkTransf;

	vector3 preLinVel = linVel;
	vector3 preAngVel = angVel;

	// linear velocity
	linVel = linkLinVel + linkAngVel * (position(0,0,0)*curTransf - position(0,0,0) * linkTransf);
	// angular velocity
	angVel = linkAngVel;

	// linear acc
	linAcc = (linVel - preLinVel) / sysState->TIME_STEP;
	// angular acc
	angAcc = (angVel - preAngVel) / sysState->TIME_STEP;

	// here, current positon is set
	updateFemMesh(areaRatio);
}
void
EachFeather::updateBasicInform()
{
	for(int i=0;i<numOfPiece;i++)
	{
		current[i]->updateBasicInform();
	}
}
void
EachFeather::updateBendAngle(double K_s, double K_d)
{
	vector3		axis, torque, normForce, tempV, momentArm, momentArmS, momentArmE;
	float		preAngle, force;

	int num = (int)(numOfPiece*0.6f);

	for(int i=numOfPiece-1;i>=num;i--)
	{
		torque = vector3(0.0f, 0.0f, 0.0f);

		/*if(current[i]->nn == 4)			axis = current[i]->pos[0] - current[i]->pos[3];
		else if(current[i]->nn == 3)	axis = current[i]->pos[2] - current[i]->pos[1];

		if(current[i]->nn == 4)			momentArmS = interpolate(0.5f, current[i]->pos[0], current[i]->pos[3]);
		else if(current[i]->nn == 3)	momentArmS = interpolate(0.5f, current[i]->pos[2], current[i]->pos[1]);*/

		axis = normalize(axis);

		for(int j=i;j<numOfPiece;j++)
		{
			/*if(current[i]->nn == 4)			tempV = current[i]->centerOfPiece - current[i]->pos[0];
			else if(current[i]->nn == 3)	tempV = current[i]->centerOfPiece - current[i]->pos[2];*/
		
			//momentArm = tempV - VECTOR_PROJECTION(tempV, axis) * axis;
			momentArmE = current[i]->centerOfPiece;
			momentArm = momentArmE - momentArmS;

			normForce = VECTOR_PROJECTION((current[j]->netForce),(current[j]->normal)) * current[j]->normal;
			torque += momentArm * normForce;
		}

		force		= torque.length();
		if(torque%axis < 0)	force *= -1.0f;

		// static intgration
		//bendAng[i]		= ( (K_d/sysState->TIME_STEP) * bendAng - force )/(K_s + K_d/sysState->TIME_STEP);

		// euler integration
		//bendAng[i] = bendAng[i] + sysState->TIME_STEP * (-K_s * bendAng[i] - force) / K_d;

		// runge-kutta
		float bendAng_t	 = bendAng[i];
		float bendAng_tt = bendAng_t + sysState->TIME_STEP * (-K_s * bendAng_t - force) / K_d;
		float dA_dt = ( (-K_s * bendAng_t - force) / K_d + (-K_s * bendAng_tt - force) / K_d ) / 2.0f;

		bendAng[i] = bendAng[i] + sysState->TIME_STEP * dA_dt;

		//bendAng[i] = AngToRad(-30.0f);
		//cout << bendAng[i] << endl;
	}
}
void
EachFeather::updateTwistAngle(double K_s, double K_d)
{
	vector3		axis, realAxis, torque, normForce, tempV, vA, vB, momentArm, momentArmS, momentArmE;
	
	torque = vector3(0.0f, 0.0f, 0.0f);
	
	realAxis = interpolate(widthRatio, current.front()->pos[1], current.front()->pos[2]) -
		interpolate(widthRatio, current.front()->pos[0], current.front()->pos[3]);
	realAxis = normalize(realAxis);

	for(int i=0;i<numOfPiece;i++)
	{
		/*if(current[i]->nn == 4)
		{
			vA =  interpolate(widthRatio, current[i]->pos[1], current[i]->pos[2]);
			vA =  interpolate(widthRatio, current[i]->pos[0], current[i]->pos[3]);
		}
		else if(current[i]->nn == 3)
		{
			vA =  current[i]->pos[0];
			vB =  interpolate(widthRatio, current[i]->pos[2], current[i]->pos[1]);
		}*/

		axis = vA - vB;	axis = normalize(axis);
		momentArmS = interpolate(0.5f, vA, vB);
		momentArmE = current[i]->centerOfPiece;
		
		/*if(current[i]->nn == 4)
		{
			tempV = current[i]->pos[1] - current[i]->pos[2];
		}
		else if(current[i]->nn == 3)
		{
			tempV = current[i]->pos[2] - current[i]->pos[1];
		}*/

		tempV = normalize(tempV);
		
		position p1 = current[i]->centerOfPiece;
		position p2 = vector2position(vA);
		position p3 = vector2position(vB);
		//momentArm = positiondistance(p1, p2, p3) * tempV;
		momentArm = momentArmE - momentArmS;

		normForce = VECTOR_PROJECTION((current[i]->netForce),(current[i]->normal)) * current[i]->normal;
		//torque += VECTOR_PROJECTION(torque,realAxis) * realAxis;
		torque += momentArm * normForce;
	}

	float force		= torque.length();
	if(torque%axis < 0)	force		*= -1.0f;
	
	//if(torque%axis < 0)		force		*= -1.0f;
	//if(!isLeft())			preAngle	*= -1.0f;
	// static integration
	//twistAng = ( (K_d/sysState->TIME_STEP) * twistAng - force )/(K_s + K_d/sysState->TIME_STEP);

	// euler integration
	//twistAng = twistAng + sysState->TIME_STEP * (-K_s * twistAng - force) / K_d;

	// runge-kutta
	float twistAng_t = twistAng;
	float twistAng_tt = twistAng_t + sysState->TIME_STEP * (-K_s * twistAng_t - force) / K_d;
	float dA_dt = ( (-K_s * twistAng_t - force) / K_d + (-K_s * twistAng_tt - force) / K_d ) / 2.0f;

	twistAng = twistAng + sysState->TIME_STEP * dA_dt;
	
	if(isLeft())
	{
		if(twistAng < 0)					twistAng = 0.0f;
		else if(twistAng > M_PI/2.0f)		twistAng = M_PI/2.0f;
	}
	else
	{
		if(twistAng > 0)					twistAng = 0.0f;
		else if(twistAng < -M_PI/2.0f)		twistAng = -M_PI/2.0f;
	}
}
void
EachFeather::calcAeroForce(position bodyPos, float areaRatio)
{
	totalForce = vector3(0,0,0);
	totalTorque = vector3(0,0,0);

	position pinnedPos = getPinnedPositionLocal() * curTransf;
	vector3 twistAxis = getTwistAxis() * curTransf;
	vector3 r;

	totalTwistTorque = 0.0f;

	for(int i=0;i<numOfPiece;i++)
	{
		//current[i]->calcAeroForce( bodyPos, areaRatio );

		current[i]->dragForce = femMesh->dragForce_aero[i];
		current[i]->liftForce = femMesh->liftForce_aero[i];
		current[i]->netForce = femMesh->netForce_aero[i];
		current[i]->momentArmToBody = femMesh->COM(i) - bodyPos;
		
		totalForce	+= current[i]->netForce;
		totalTorque	+= current[i]->momentArmToBody * current[i]->netForce;

		// twist torque
		r = current[i]->centerOfPiece - pinnedPos;
		totalTwistTorque += (r*current[i]->netForce) % twistAxis;
	}

	//cout << current[0]->netForce << endl;
}
bool
EachFeather::isLeft()
{
	return	(groupID == Feathers::PrimL) || 
			(groupID == Feathers::SeconL) ||
			(groupID == Feathers::TertL);
}
bool
EachFeather::isPrimary()
{
	return	(groupID == Feathers::PrimL) || (groupID == Feathers::PrimR);
}
bool
EachFeather::isSecondary()
{
	return	(groupID == Feathers::SeconL) || (groupID == Feathers::SeconR);
}
bool
EachFeather::isTertiary()
{
	return	(groupID == Feathers::TertL) || (groupID == Feathers::TertR);
}
bool
EachFeather::isTail()
{
	return	(groupID == Feathers::Tail);
}
// this should be called after loading mesh
void
EachFeather::constructFemMesh(bool left)
{
	int idxElement;

	// Scale factor
	femMesh->scaleFactor = 1.0f;	// meter

	// Twist position
	femMesh->setPinnedPosition(getPinnedPositionLocal() * curTransf);
	// Twist axis
	femMesh->setTwistAxis(getTwistAxis() * curTransf);
	if(!isPrimary())	femMesh->twistEnalbed = false;
	else				femMesh->twistEnalbed = true;
	// Nodes
	femMesh->setNumNodes(getNumNodes());
	for(int i=0; i<getNumNodes(); i++)
	{
		femMesh->setMaterialCoord(i, mesh.node.at(i) * initTransf);
	}
	// Elements: triangle elements in this case
	femMesh->setNumElements(getNumElements());
	for(int i=0; i<getNumElements(); i++)
	{
		int vtx0 = mesh.element[i][0];
		int vtx1 = mesh.element[i][1];
		int vtx2 = mesh.element[i][2];

		femMesh->element(i).setNodeId(vtx0, vtx1, vtx2);
	}
	// Edges
	femMesh->setNumEdges(getNumEdges());
	for(int i=0; i<getNumEdges(); i++)
	{
		int vtx0 = mesh.edge[i].e[0];
		int vtx1 = mesh.edge[i].e[1];
		int vtx2 = mesh.edge[i].e[2];
		int vtx3 = mesh.edge[i].e[3];

		femMesh->edges[i].setNodeId(vtx0, vtx1, vtx2, vtx3);
	}

	// Material binding
	//femMesh->prepareMaterialBinding();

	// Static constrained nodes
	femMesh->beginStaticConstSetup();
		femMesh->setStaticPositionConst(0);				// At least 3 vertices should be constrained.
		femMesh->setStaticPositionConst(1);				// In this demo, we constrain 3 vertices,
		//femMesh->setStaticPositionConst(left ? 3 : 2);	// nearest to the twist joint.
		femMesh->setStaticPositionConst(2);	// nearest to the twist joint.
		femMesh->setStaticPositionConst(3);	// nearest to the twist joint.
	femMesh->endStaticConstSetup();

	// FPS
	femMesh->fps = sysState->FPS * sysState->STEPS;
	femMesh->startFrame = 0;

	// femMesh->diagnosis = true;

	// Material properties
	//

	if(isPrimary())
	{
		femMesh->rho				= coeff->femMeshCoeff->density_primary;
		femMesh->height				= coeff->femMeshCoeff->height_primary;
		femMesh->lambda				= coeff->femMeshCoeff->stretch_stiff_primary;
		femMesh->num_modes			= coeff->femMeshCoeff->num_mode_primary;
		femMesh->mu2				= coeff->femMeshCoeff->bend_stiff_near_primary;
		femMesh->mu1				= coeff->femMeshCoeff->bend_stiff_far_primary;
		femMesh->damping			= coeff->femMeshCoeff->bend_damping_primary;
		femMesh->twistStiffness		= coeff->femMeshCoeff->twist_stiff_primary;
		femMesh->twistDampingFactor	= coeff->femMeshCoeff->twist_damping_primary;
	}
	else if(isTail())
	{
		femMesh->rho				= coeff->femMeshCoeff->density_tail;
		femMesh->height				= coeff->femMeshCoeff->height_tail;
		femMesh->lambda				= coeff->femMeshCoeff->stretch_stiff_tail;
		femMesh->num_modes			= coeff->femMeshCoeff->num_mode_tail;
		femMesh->mu2				= coeff->femMeshCoeff->bend_stiff_near_tail;
		femMesh->mu1				= coeff->femMeshCoeff->bend_stiff_far_tail;
		femMesh->damping			= coeff->femMeshCoeff->bend_damping_tail;
		femMesh->twistStiffness		= coeff->femMeshCoeff->twist_stiff_tail;
		femMesh->twistDampingFactor	= coeff->femMeshCoeff->twist_damping_tail;
	}
	else
	{
		femMesh->rho				= coeff->femMeshCoeff->density_secondary;
		femMesh->height				= coeff->femMeshCoeff->height_secondary;
		femMesh->lambda				= coeff->femMeshCoeff->stretch_stiff_secondary;
		femMesh->num_modes			= coeff->femMeshCoeff->num_mode_secondary;
		femMesh->mu2				= coeff->femMeshCoeff->bend_stiff_near_secondary;
		femMesh->mu1				= coeff->femMeshCoeff->bend_stiff_far_secondary;
		femMesh->damping			= coeff->femMeshCoeff->bend_damping_secondary;
		femMesh->twistStiffness		= coeff->femMeshCoeff->twist_stiff_secondary;
		femMesh->twistDampingFactor	= coeff->femMeshCoeff->twist_damping_secondary;
	}

	// Material binding
	femMesh->prepareMaterialBinding();

	//femMesh->rho = 200.0;		// density
	//femMesh->height = 0.002;	// 2mm
	//femMesh->mu1 = 0.001;		// bending stiffness
	//femMesh->mu2 = 0.05;		// bending stiffness
	//femMesh->lambda = 2500.0;	// stretching stiffness
	//femMesh->num_modes = 4;		// 4 is enough for bending motion

	//femMesh->damping = 0.01;	// damping for thin shell

	//femMesh->twistStiffness = 0.001;//6.0e-5;
	//femMesh->twistDampingFactor = 0.13;	// damping for twist joint

	//if(isPrimary())
	//{
	//	femMesh->mu1 = 0.000001;		// bending stiffness
	//	femMesh->mu2 = 0.00005;		// bending stiffness
	//	femMesh->damping = 0.0005;	// damping for thin shell

	//	femMesh->twistStiffness = 6.0e-5;
	//	femMesh->twistDampingFactor = 0.03;	// damping for twist joint
	//}

//	femMesh->integrationScheme = FEM_MA_SHAPE;	// To show mode shape....
//	femMesh->twistEnalbed = false;;
//	femMesh->damping = 0.5;
}
void
EachFeather::constructFemMesh_primary()
{
	constructFemMesh(isLeft());
}
void	
EachFeather::constructFemMesh_secondary()
{
	constructFemMesh(isLeft());
}
void
EachFeather::constructFemMesh_tertiary()
{
	constructFemMesh(isLeft());
}
void
EachFeather::constructFemMesh_tail()
{
	constructFemMesh(((orderInGroup/(float)numSibling)<0.5f));
}
// this should be called after transformation setting
void
EachFeather::updateFemMesh(float areaRatio)
{
	if(!coeff->femMeshCoeff->loaded) return;

	/////////////////// / / / / / / / / / / / / 
	// Simulation loop
	//

	/*if(cnt % 1000 == 0)
		cout << "frame : " << cnt << endl;*/
	//cout << "here0" << endl;

	if(!initialized)
	{
		if(isPrimary())			constructFemMesh_primary();
		else if(isSecondary())	constructFemMesh_secondary();
		else if(isTertiary())	constructFemMesh_tertiary();
		else if(isTail())		constructFemMesh_tail();
	}

	//cout << "here1" << endl;

	/*for(int i=0; i<(int)getNumElements(); i++)
	{
		for(int j=0; j<3; j++)
		{
			femMesh->addExternalForce(mesh.element[i][j], current[i]->netForce / 3.0f);
		}
	}

	femMesh->setExternalTorqueTwist(totalTwistTorque);*/

	// Relative transform with respect to the inital. i.e T0.inverse() * Tcurrent
	transf	T = getInitToCurrentTransf();
	// Linear and angular velocity
	vector3	lv = getLinearVel(), av = getAngularVel();	
	// Linear and angular acceleration
	vector3	la = getLinearAcc(), aa = getAngularAcc();	

	// set area ratio
	femMesh->areaRatio = areaRatio;

	if(!initialized)
	{
		//cout << "feather init" << endl;
		femMesh->simulateMain(0, T, lv, av, la, aa);
		initialized = true;
	}

	femMesh->simulateMain(++cnt, T, lv, av, la, aa);

	//cout << "here2" << endl;

	if (femMesh->wasFailed())
	{
		cerr << "Simulation failed!" << endl;
		cin.get();
	}

	for(int i=0; i<(int)getNumNodes(); i++)
	for(int j=0; j<(int)mesh.nodeID2element[i].size(); j++)
	{
		int eleID = mesh.nodeID2element[i][j][0];
		int vIdx = mesh.nodeID2element[i][j][1];

		// the position of the i-th node in the global coordinates
		current[eleID]->pos[vIdx] = femMesh->p[i];
	}

	//cout << "here3" << endl;
}
void
EachFeather::saveFemMesh(int sID)
{
	femMesh->getState(femMeshState[sID]);

	fem_dragForce_save[sID].clear();
	fem_liftForce_save[sID].clear();
	fem_netForce_save[sID].clear();

	for(int i=0; i<getNumElements(); i++)
	{
		fem_dragForce_save[sID].push_back(femMesh->dragForce_aero[i]);
		fem_liftForce_save[sID].push_back(femMesh->liftForce_aero[i]);
		fem_netForce_save[sID].push_back(femMesh->netForce_aero[i]);
	}
}
void
EachFeather::restoreFemMesh(int sID)
{
	femMesh->setState(femMeshState[sID]);

	for(int i=0; i<getNumElements(); i++)
	{
		femMesh->dragForce_aero[i] = fem_dragForce_save[sID][i];
		femMesh->liftForce_aero[i] = fem_liftForce_save[sID][i];
		femMesh->netForce_aero[i] = fem_netForce_save[sID][i];
	}
}
position
EachFeather::getPinnedPositionLocal()
{
	return mesh.getPinnedPosition();
}
position
EachFeather::getPinnedPositionGlobal()
{
	return mesh.getPinnedPosition() * curTransf;
}
vector3
EachFeather::getTwistAxis()
{
	if(isLeft())	return mesh.getTwistAxis();
	else			return -mesh.getTwistAxis();
}
int
EachFeather::getNumNodes()
{
	return (int)mesh.node.size();
}
int
EachFeather::getNumElements()
{
	return (int)mesh.element.size();
}
int
EachFeather::getNumEdges()
{
	//return 2 * (int)init.size() + 1;
	return (int)mesh.edge.size();
}
transf	
EachFeather::getInitTransf() 
{
	return initTransf;
}
transf
EachFeather::getInitToCurrentTransf() 
{ 
	return initTransf.inverse() * curTransf;
}
vector3	
EachFeather::getLinearVel()
{
	return linVel;
}
vector3	
EachFeather::getLinearAcc()
{
	return linAcc;
}
vector3	
EachFeather::getAngularVel()
{
	return angVel;
}
vector3	
EachFeather::getAngularAcc()
{
	return angAcc;
}
void
EachFeather::drawEachFeather(bool depthTest)
{
	float kd[4] = {0.25,		0.20725,	0.20725,	0.75};
	float ka[4] = {1.0,			0.829,		0.829,		0.75};
	float ks[4] = {0.296648,	0.296648,	0.296648,	0.75};
	float exponent = 11.264;

	//glSetMaterial(kd,ka,ks,exponent);
	//glSetMaterial(0.85,0.85,0.85, 0.85);
	
	if(!depthTest) glDisable(GL_DEPTH_TEST);
	//glEnable(GL_BLEND);
	glDisable(GL_CULL_FACE);
	glDisable(GL_LIGHTING);
	glEnable(GL_LIGHTING);

	vector3 color1, color2;

	switch(groupID)
	{
	case Feathers::PrimL:	case Feathers::PrimR:
		color1 = vector3(217/255.0,112/255.0,50/255.0);
		color2 = vector3(215/255.0,146/255.0,51/255.0);
		break;
	case Feathers::SeconL:	case Feathers::SeconR:
		color1 = vector3(20/255.0,35/255.0,90/255.0);
		color2 = vector3(10/255.0,10/255.0,7/255.0);
		break;
	case Feathers::TertL:	case Feathers::TertR:
		break;
	case Feathers::Tail:
		color1 = vector3(36/255.0,103/255.0,50/255.0);
		color2 = vector3(180/255.0,160/255.0,0/255.0);
		break;
	}

	for(int i=0;i<numOfPiece;i++)
	{
		m_real alpha = (i)/(float)(numOfPiece-1);
		vector3 c = (1-alpha)*color1 + alpha*color2;
		
		//glSetMaterial(kd,ka,ks,exponent);
		//glColor4f(c[0],c[1],c[2],0.6);
		//glColor3f(i/(float)numOfPiece,0,0);

		glColor4f(1,1,1,0.6);

		current[i]->drawSurface();
		current[i]->drawSurfaceBoundary();
	}

	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	//glDisable(GL_BLEND);
	if(!depthTest) glEnable(GL_DEPTH_TEST);
}
void
EachFeather::drawEachInform()
{
	glDisable(GL_LIGHTING);

	glColor3f(1,0,0);	for(int i=0;i<numOfPiece;i++)	current[i]->drawDragForce();
	glColor3f(0,1,0);	for(int i=0;i<numOfPiece;i++)	current[i]->drawLiftForce();
	glColor3f(0,0,1);	for(int i=0;i<numOfPiece;i++)	current[i]->drawNetForce();

	//glColor3f(0,1,1);	for(int i=0;i<numOfPiece;i++)	current[i]->drawNormal();
	//glColor3f(1,1,0);	for(int i=0;i<numOfPiece;i++)	current[i]->drawVelocity();

	glEnable(GL_LIGHTING);
}
void	
Feathers::loadFeathersInformation()
{
	ifstream fin;
	if(util::fileOpenWrapper(fin, INFORM_DIR+string("Feather_inform.txt")))
	{
		for(int i=0;i<MaxFeatherName;i++)
		{
			GroupFeathers* gF	= new GroupFeathers();

			gF->numOfeachFeather	= atoi(getNext(fin, "//").c_str());

			featherGroup.push_back(gF);
		}

		for(int i=0;i<MaxFeatherName;i++)
		{
			featherGroup[i]->numOfPiece = atoi(getNext(fin, "//").c_str());

			switch(i)
			{
			//case PrimL: case PrimR:
			default:
				featherGroup[i]->numOfPiece = 2*(featherGroup[i]->numOfPiece-1) + 1;
				for(int j=0;j<featherGroup[i]->numOfeachFeather;j++)
				{
					EachFeather* eF	= 
						new EachFeather(i, j, featherGroup[i]->numOfeachFeather ,featherGroup[i]->numOfPiece);
					featherGroup[i]->each.push_back(eF);
				}
				break;
			/*default:
				for(int j=0;j<featherGroup[i]->numOfeachFeather;j++)
				{
					EachFeather* eF	= new EachFeather(i, featherGroup[i]->numOfPiece);
					featherGroup[i]->each.push_back(eF);
				}
				break;*/
			}
		}

		float temp;

		featherGroup[PrimL]->length			= featherGroup[PrimR]->length		= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[SeconL]->length		= featherGroup[SeconR]->length		= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[TertL]->length			= featherGroup[TertR]->length		= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[Tail]->length												= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );

		featherGroup[PrimL]->lengthRatio	= featherGroup[PrimR]->lengthRatio	= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[SeconL]->lengthRatio	= featherGroup[SeconR]->lengthRatio	= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[TertL]->lengthRatio	= featherGroup[TertR]->lengthRatio	= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[Tail]->lengthRatio											= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );

		featherGroup[PrimL]->width			= featherGroup[PrimR]->width		= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[SeconL]->width			= featherGroup[SeconR]->width		= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[TertL]->width			= featherGroup[TertR]->width		= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[Tail]->width												= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );

		featherGroup[PrimL]->widthRatio		= featherGroup[PrimR]->widthRatio	= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[SeconL]->widthRatio	= featherGroup[SeconR]->widthRatio	= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[TertL]->widthRatio		= featherGroup[TertR]->widthRatio	= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[Tail]->widthRatio											= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );

		featherGroup[PrimL]->spreadAngle	= featherGroup[PrimR]->spreadAngle	= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[SeconL]->spreadAngle	= featherGroup[SeconR]->spreadAngle	= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[TertL]->spreadAngle	= featherGroup[TertR]->spreadAngle	= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );
		featherGroup[Tail]->spreadAngle											= make_pair( (float)atof(getNext(fin, "//").c_str()), (float)atof(getNext(fin, "//").c_str()) );

		// initialze feather_save
		for(int i=0; i<max_save; i++)
		{
			for(int j=0; j<(int)MaxFeatherName; j++)
			{
				GroupFeathers *gF = new GroupFeathers();

				for(int k=0; k<featherGroup[j]->numOfeachFeather; k++)
				{
					EachFeather* eF	= 
						new EachFeather(j, k, featherGroup[j]->numOfeachFeather ,featherGroup[j]->numOfPiece);

					gF->each.push_back(eF);
				}
				featherGroupSave[i].push_back(gF);
			}
		}

		util::fileCloseWrapper(fin);
	}
#ifdef OLD
#else
	featherAngle.readData();
#endif
}
void
Feathers::setFeathers()
{
	for(int i=0;i<MaxFeatherName;i++)
	{
		if(i%2 == 1)	featherGroup[i]->swapInform();

		// 0 : start joint attach, 1 : end joint attach
		float linkRatio;
		// body side axis length/2 where feathers is attached
		float z_trans = thisBody->length[convertID_feather2body( i )][axis_Z]/2.0f;
		// body main axis length where feathers is attached
		float linkLength	= thisBody->length[ convertID_feather2body( i ) ][axis_X];

		vector3 linkTrans	= thisBody->getLinkTranslation((Body::LinkName) convertID_feather2body( i ));
		quater linkRot		= thisBody->getLinkOrientation((Body::LinkName) convertID_feather2body( i ));

		// a bird's head directs z axis and arms are coincide to x axis and a bird's up direction is y axis
		// we look a bird from positive z axis
		// left and right are difined when we look a bird on top and when north direction is bird's head direction
		switch(i)
		{
			case PrimL:		linkRatio = 0.0f;	break;
			case PrimR:		linkRatio = 1.0f;	break;
			case SeconL:	linkRatio = -1.0f;	break;
			case SeconR:	linkRatio = -1.0f;	break;
			case TertL:		linkRatio = -1.0f;	break;
			case TertR:		linkRatio = -1.0f;	break;
			case Tail:		linkRatio = 0.5f;	z_trans *= -1.0f;	break;
		}
		
		int num = featherGroup[i]->numOfeachFeather;
		for(int j=0;j<num;j++)		
		{
			float inputRatio;

			// if link ratio is negative then feathers are attached toward start and end joint
			if(linkRatio < 0.0f)	inputRatio  = (float)j / (num-1);
			else					inputRatio	= linkRatio;

			float sAng = 0.0f;

			featherGroup[i]->setEachFeather(j, linkLength, inputRatio, z_trans);
			featherGroup[i]->area += featherGroup[i]->each[j]->area;

			//cout << featherGroup[i]->area << endl;
		}
	}
}
void
Feathers::applyFeatherAngle()
{
	OneFeatherAngle ofa;

	for(int i=0; i<3; i++)
	{
		ofa.spread_from[i]	= coeff->featherAngleCoeff->feather_spread_from[i];
		ofa.spread_to[i]	= coeff->featherAngleCoeff->feather_spread_to[i];
		ofa.bend_from[i]	= coeff->featherAngleCoeff->feather_bend_from[i];
		ofa.bend_to[i]		= coeff->featherAngleCoeff->feather_bend_to[i];
	}

	updateSpreadBend_manual(ofa);

	// local update, global update
	#pragma omp parallel for
	for(int i=0;i<MaxFeatherName;i++)
	{		
		transf linkTransf	= thisBody->getLinkTransformation((Body::LinkName) convertID_feather2body( i ));
		vector3 linkLinVel	= thisBody->getLinkLinearVelocity((Body::LinkName) convertID_feather2body( i ));
		vector3 linkAngVel	= thisBody->getLinkAngularVelocity((Body::LinkName) convertID_feather2body( i ));

		featherGroup[i]->updateState(linkTransf, linkLinVel, linkAngVel);
	}
}
void
Feathers::spreadFromTo(int idx, float from, float to)
{
	int num = featherGroup[idx]->numOfeachFeather;

	for(int i=0;i<num;i++)
	{
		float ang = from + (to - from) * ( i / (float)(num-1) );
		featherGroup[idx]->each[i]->spreadAng = ang;
	}
}
void
Feathers::bendFromTo(int idx, float from, float to)
{
	int num = featherGroup[idx]->numOfeachFeather;

	for(int i=0;i<num;i++)
	{
		float ang = from + (to - from) * ( i / (float)(num-1) );
		featherGroup[idx]->each[i]->bendAng[0]= ang;
	}
}
void
Feathers::updateSpreadBend()
{
	//cin.get(); cin.get();

	float bend1, bend2, twist, temp;

	float default_angle = AngToRad(10);
	float default_angle2 = AngToRad(15);

	static float wristBendL_init, wristBendR_init;
	static float elbowBendL_init, elbowBendR_init;
	static float elbowTwistL_init, elbowTwistR_init;

	float wristBendL, wristBendR;
	float elbowBendL, elbowBendR;
	float elbowTwistL, elbowTwistR;

	if(isFirstSimulation)
	{
		thisBody->getJointAngle( (Body::JointName)Body::WristL, &wristBendL_init, &temp );
		thisBody->getJointAngle( (Body::JointName)Body::ElbowL, &elbowBendL_init, &elbowTwistL_init );

		thisBody->getJointAngle( (Body::JointName)Body::WristR, &wristBendR_init, &temp );
		thisBody->getJointAngle( (Body::JointName)Body::ElbowR, &elbowBendR_init, &elbowTwistR_init );
	}

	// Left feathers
	{
		// wrist bend -이면 펴진것
		// elbow bend +이면 펴진것
		// elbow twist -이면 올라간것

		thisBody->getJointAngle( (Body::JointName)Body::WristL, &wristBendL, &temp );
		thisBody->getJointAngle( (Body::JointName)Body::ElbowL, &elbowBendL, &elbowTwistL );

		//cout << "L----------------" << endl;
		//cout << "wrist bend" << wristBendL_init << " / " << wristBendL << endl;
		//cout << "elbow bend" << elbowBendL_init << " / " << elbowBendL << endl;
		//cout << "elbow twist" << elbowTwistL_init << " / " << elbowTwistL << endl;

		spreadFromTo( PrimL , 0.1*GOLDEN_RATIO, thisBody->initS.wristBend[0] - (wristBendL-wristBendL_init)/2.0f);
		spreadFromTo( SeconL, -0.4+(wristBendL-wristBendL_init)/3.0f, 0.3-(wristBendL-wristBendL_init)/2.0f);
		//spreadFromTo( SeconL, 0, 0 );
		bendFromTo( PrimL, default_angle, default_angle);
		bendFromTo( SeconL, default_angle2, default_angle2);
		//bendFromTo( PrimL, default_angle + (bend2 + AngToRad(50.0f))/5.0f, default_angle + (bend2 + AngToRad(50.0f))/5.0f);
		//bendFromTo( SeconL, default_angle - twist/10.0f, default_angle + (bend2 + AngToRad(50.0f))/5.0f);
	}

	// right feathers
	{
		// wrist bend +이면 펴진것
		// elbow bend -이면 펴진것
		// elbow twist -이면 올라간것

		thisBody->getJointAngle( (Body::JointName)Body::WristR, &wristBendR, &temp );
		thisBody->getJointAngle( (Body::JointName)Body::ElbowR, &elbowBendR, &elbowTwistR );

		//cout << "R----------------" << endl;
		//cout << "wrist bend" << wristBendR_init << " / " << wristBendR << endl;
		//cout << "elbow bend" << elbowBendR_init << " / " << elbowBendR << endl;
		//cout << "elbow twist" << elbowTwistR_init << " / " << elbowTwistR << endl;

		spreadFromTo( PrimR , thisBody->initS.wristBend[1] - (wristBendR-wristBendR_init)/2.0f, -0.1*GOLDEN_RATIO);
		spreadFromTo( SeconR, -0.3-(wristBendR-wristBendR_init)/2.0f, 0.4+(wristBendR-wristBendR_init)/3.0f);

		bendFromTo( PrimR, default_angle, default_angle);
		bendFromTo( SeconR, default_angle2, default_angle2);
		//spreadFromTo( PrimR, thisBody->initS.wristBend[1] + (wristBendR-wristBendR_init)/2.0f, 0 );
		//spreadFromTo( SeconR, -0.2, 0.2);
		//bendFromTo( PrimR, default_angle - (bend2 - AngToRad(50.0f))/5.0f, default_angle - (bend2 - AngToRad(50.0f))/5.0f);
		//bendFromTo( SeconR, default_angle - (bend2 - AngToRad(50.0f))/5.0f, default_angle - twist/10.0f );
	}

	// tail feathers
	float getTail[2];
	controller->getDesiredTailState(getTail);
	bendFromTo( Tail, getTail[0], getTail[0]);
	spreadFromTo( Tail, -getTail[1]/2.0f, getTail[1]/2.0f);
}

// before modify
void
Feathers::updateSpreadBend2()
{
	float bend1, bend2, twist, temp;

	float default_angle = AngToRad(5);

	static float wristBendL_init, wristBendR_init;
	static float elbowBendL_init, elbowBendR_init;
	static float elbowTwistL_init, elbowTwistR_init;

	float wristBendL, wristBendR;
	float elbowBendL, elbowBendR;
	float elbowTwistL, elbowTwistR;

	if(isFirstSimulation)
	{
		thisBody->getJointAngle( (Body::JointName)Body::WristL, &wristBendL_init, &temp );
		thisBody->getJointAngle( (Body::JointName)Body::ElbowL, &elbowBendL_init, &elbowTwistL_init );

		thisBody->getJointAngle( (Body::JointName)Body::WristR, &wristBendR_init, &temp );
		thisBody->getJointAngle( (Body::JointName)Body::ElbowR, &elbowBendR_init, &elbowTwistR_init );
	}

	// Left feathers
	{
		// wrist bend -이면 펴진것
		// elbow bend +이면 펴진것
		// elbow twist -이면 올라간것

		thisBody->getJointAngle( (Body::JointName)Body::WristL, &wristBendL, &temp );
		thisBody->getJointAngle( (Body::JointName)Body::ElbowL, &elbowBendL, &elbowTwistL );

		//cout << "L----------------" << endl;
		//cout << "wrist bend" << wristBendL_init << " / " << wristBendL << endl;
		//cout << "elbow bend" << elbowBendL_init << " / " << elbowBendL << endl;
		//cout << "elbow twist" << elbowTwistL_init << " / " << elbowTwistL << endl;

		spreadFromTo( PrimL , 0.1*GOLDEN_RATIO, thisBody->initS.wristBend[0] - (wristBendL-wristBendL_init)/2.0f);
		spreadFromTo( SeconL, -0.4+(wristBendL-wristBendL_init)/3.0f, 0.3-(wristBendL-wristBendL_init)/2.0f);
		//spreadFromTo( SeconL, 0, 0 );
		bendFromTo( PrimL, default_angle, default_angle);
		bendFromTo( SeconL, default_angle, default_angle);
		//bendFromTo( PrimL, default_angle + (bend2 + AngToRad(50.0f))/5.0f, default_angle + (bend2 + AngToRad(50.0f))/5.0f);
		//bendFromTo( SeconL, default_angle - twist/10.0f, default_angle + (bend2 + AngToRad(50.0f))/5.0f);
	}

	// right feathers
	{
		// wrist bend +이면 펴진것
		// elbow bend -이면 펴진것
		// elbow twist -이면 올라간것

		thisBody->getJointAngle( (Body::JointName)Body::WristR, &wristBendR, &temp );
		thisBody->getJointAngle( (Body::JointName)Body::ElbowR, &elbowBendR, &elbowTwistR );

		//cout << "R----------------" << endl;
		//cout << "wrist bend" << wristBendR_init << " / " << wristBendR << endl;
		//cout << "elbow bend" << elbowBendR_init << " / " << elbowBendR << endl;
		//cout << "elbow twist" << elbowTwistR_init << " / " << elbowTwistR << endl;

		spreadFromTo( PrimR , thisBody->initS.wristBend[1] - (wristBendR-wristBendR_init)/2.0f, -0.1*GOLDEN_RATIO);
		spreadFromTo( SeconR, -0.3-(wristBendR-wristBendR_init)/2.0f, 0.4+(wristBendR-wristBendR_init)/3.0f);

		bendFromTo( PrimR, default_angle, default_angle);
		bendFromTo( SeconR, default_angle, default_angle);
		//spreadFromTo( PrimR, thisBody->initS.wristBend[1] + (wristBendR-wristBendR_init)/2.0f, 0 );
		//spreadFromTo( SeconR, -0.2, 0.2);
		//bendFromTo( PrimR, default_angle - (bend2 - AngToRad(50.0f))/5.0f, default_angle - (bend2 - AngToRad(50.0f))/5.0f);
		//bendFromTo( SeconR, default_angle - (bend2 - AngToRad(50.0f))/5.0f, default_angle - twist/10.0f );
	}

	// tail feathers
	float* getTail;	getTail = new float[2];
	controller->getDesiredTailState(getTail);
	bendFromTo( Tail, getTail[0], getTail[0]);
	spreadFromTo( Tail, -getTail[1]/2.0f, getTail[1]/2.0f);
}
void
Feathers::updateSpreadBend3()
{

	float bend1, bend2, twist, temp;

	float default_angle = M_PI/12.0f;

	// Left feathers
	{
		thisBody->getJointAngle( (Body::JointName)Body::WristL, &bend1, &temp );	bend1 *= -1.0f;
		thisBody->getJointAngle( (Body::JointName)Body::ElbowL, &bend2, &twist );	twist *= -1.0f;
		if(bend1 > 0)	bend1 *= 1.0f/3.0f;	// secondary쪽으로 접힐때는 1/3정도는 영향을 받아서 접히도록
		else			bend1 *= 1.0f/3.0f;

		bend1	+= thisBody->initS.wristBend[0];
		bend2	+= thisBody->initS.elbowBend[0];
		twist	+= thisBody->initS.elbowTwist[0];

		spreadFromTo( PrimL , 0, bend1);
		spreadFromTo( SeconL, bend2/2.5f, 0);
		//bendFromTo( PrimL, default_angle + (bend2 + AngToRad(50.0f))/5.0f, default_angle + (bend2 + AngToRad(50.0f))/5.0f);
		//bendFromTo( SeconL, default_angle - twist/10.0f, default_angle + (bend2 + AngToRad(50.0f))/5.0f);
	}

	// right feathers
	{
		thisBody->getJointAngle( (Body::JointName)Body::WristR, &bend1, &temp );	bend1 *= -1.0f;
		thisBody->getJointAngle( (Body::JointName)Body::ElbowR, &bend2, &twist );	twist *= -1.0f;
		if(bend1 < 0)	bend1 *= 1.0f/3.0f;	// secondary쪽으로 접힐때는 1/3정도는 영향을 받아서 접히도록
		else			bend1 *= 1.0f/3.0f;
		
		bend1	+= thisBody->initS.wristBend[1];
		bend2	+= thisBody->initS.elbowBend[1];
		twist	+= thisBody->initS.elbowTwist[1];

		spreadFromTo( PrimR, bend1, 0 );
		spreadFromTo( SeconR, 0, bend2/2.5f);
		//bendFromTo( PrimR, default_angle - (bend2 - AngToRad(50.0f))/5.0f, default_angle - (bend2 - AngToRad(50.0f))/5.0f);
		//bendFromTo( SeconR, default_angle - (bend2 - AngToRad(50.0f))/5.0f, default_angle - twist/10.0f );
	}

	// tail feathers
	float* getTail;	getTail = new float[2];
	controller->getDesiredTailState(getTail);
	bendFromTo( Tail, getTail[0], getTail[0]);
	spreadFromTo( Tail, -getTail[1]/2.0f, getTail[1]/2.0f);
}
void
Feathers::updateSpreadBend_manual(const OneFeatherAngle& ofa)
{
	spreadFromTo( PrimL,	DegToRad(ofa.spread_from[0]),	DegToRad(ofa.spread_to[0]) );
	spreadFromTo( PrimR,	DegToRad(-ofa.spread_to[0]),	DegToRad(-ofa.spread_from[0]) );
	spreadFromTo( SeconL,	DegToRad(ofa.spread_from[1]),	DegToRad(ofa.spread_to[1]) );
	spreadFromTo( SeconR,	DegToRad(-ofa.spread_to[1]),	DegToRad(-ofa.spread_from[1]) );

	bendFromTo( PrimL,	DegToRad(ofa.bend_from[0]), DegToRad(ofa.bend_to[0]) );
	bendFromTo( PrimR,	DegToRad(ofa.bend_to[0]),	DegToRad(ofa.bend_from[0]) );
	bendFromTo( SeconL, DegToRad(ofa.bend_from[1]), DegToRad(ofa.bend_to[1]) );
	bendFromTo( SeconR, DegToRad(ofa.bend_to[1]),	DegToRad(ofa.bend_from[1]) );

	// tail feathers
	float getTail[2];
	controller->getDesiredTailState(getTail);
	bendFromTo( Tail, getTail[0], getTail[0]);
	spreadFromTo( Tail, -getTail[1]/2, getTail[1]/2);
}
void
Feathers::updateAreaRatio()
{

	for(int i=0; i<MaxFeatherName; i++)
	{
		float tempArea = 0.0f;
		position p0, p1, p2, p3;
		int idx;

		int numPiece;
		int numEach = featherGroup[i]->numOfeachFeather;

		switch(i)
		{
		case PrimL: case SeconL:

			p0 = featherGroup[i]->each[numEach-1]->getPinnedPositionGlobal();

			for(int j=numEach-1; j>0; j--)
			{
				int numPieceCur = featherGroup[i]->each[j]->numOfPiece;
				int numPieceNext = featherGroup[i]->each[j-1]->numOfPiece;

				p1 = featherGroup[i]->each[j]->current[numPieceCur-2]->pos[2];
				p2 = featherGroup[i]->each[j-1]->current[numPieceNext-2]->pos[2];

				tempArea += triangleArea( p0, p1, p2 );
			}

			// 마지막 깃털은 따로
			numPiece = featherGroup[i]->each[0]->numOfPiece;
			p1 = featherGroup[i]->each[0]->current[numPiece-2]->pos[2];
			p2 = featherGroup[i]->each[0]->current[numPiece-2]->pos[0];
			p3 = featherGroup[i]->each[0]->current[0]->pos[0];
			tempArea += triangleArea(p0, p1, p2);
			tempArea += triangleArea(p1, p2, p3);

			break;

		case PrimR: case SeconR: case Tail:
			
			p0 = featherGroup[i]->each[0]->getPinnedPositionGlobal();

			for(int j=0; j<numEach-1; j++)
			{
				int numPieceCur = featherGroup[i]->each[j]->numOfPiece;
				int numPieceNext = featherGroup[i]->each[j+1]->numOfPiece;

				p1 = featherGroup[i]->each[j]->current[numPieceCur-2]->pos[1];
				p2 = featherGroup[i]->each[j+1]->current[numPieceNext-2]->pos[1];

				tempArea += triangleArea( p0, p1, p2 );
			}

			// 마지막 깃털은 따로
			numPiece = featherGroup[i]->each[numEach-1]->numOfPiece;
			p1 = featherGroup[i]->each[numEach-1]->current[numPiece-2]->pos[1];
			p2 = featherGroup[i]->each[numEach-1]->current[numPiece-2]->pos[0];
			p3 = featherGroup[i]->each[numEach-1]->current[0]->pos[0];
			tempArea += triangleArea(p0, p1, p2);
			tempArea += triangleArea(p1, p2, p3);

			break;
		}

		// 끝 조각들 더해주기

		for(int j=0; j<numEach; j++)
		{
			numPiece = featherGroup[i]->each[j]->numOfPiece;

			p0 = featherGroup[i]->each[j]->current[numPiece-1]->pos[0];
			p1 = featherGroup[i]->each[j]->current[numPiece-1]->pos[1];
			p2 = featherGroup[i]->each[j]->current[numPiece-1]->pos[2];

			tempArea += triangleArea( p0, p1, p2 );
		}

		featherGroup[i]->areaRatio = tempArea / featherGroup[i]->area;

		if(false)
		{
			cout << "group: " << i << endl;
			cout << "area Ratio: " << featherGroup[i]->areaRatio << "     ";
			cout << "tempArea: " << tempArea << " initArea: " << featherGroup[i]->area << endl;
			cout << endl;
		}
	}
}
void
Feathers::updateBendAngle()	
{
	featherGroup[PrimL]->updateBendAngle(coeff->linkCoeff->spring[0], coeff->linkCoeff->spring[1]);
	featherGroup[PrimR]->updateBendAngle(coeff->linkCoeff->spring[0], coeff->linkCoeff->spring[1]);
}
void				
Feathers::updateTwistAngle()
{
	featherGroup[PrimL]->updateTwistAngle(coeff->linkCoeff->spring[2], coeff->linkCoeff->spring[3]);
	featherGroup[PrimR]->updateTwistAngle(coeff->linkCoeff->spring[2], coeff->linkCoeff->spring[3]);
}
void
Feathers::updateBasicInform()
{
	for(int i=0;i<MaxFeatherName;i++)
	{
		featherGroup[i]->updateBasicInform();
	}
}
void
Feathers::updateOrientation()
{
	for(int i=0;i<MaxFeatherName;i++)
	for(int j=0;j<featherGroup[i]->numOfeachFeather;j++)
	{	
		bool isLeft = true;

		if(i%2 == 1)	isLeft = false;
		if(i==Tail && j >= featherGroup[i]->numOfeachFeather/2)		isLeft = false;

		featherGroup[i]->each[j]->updateOrientation(isLeft);
	}
}
void
Feathers::updateState()
{
	// update bend and twist btw each pieces
	if(!isFirstSimulation)	
	{
		//updateBendAngle();
		//updateTwistAngle();
	}

	//cout << "us0" << endl;

	// update spread bend btw link body and feather
	//updateSpreadBend();
	if(featherAngle.loaded())	updateSpreadBend_manual( featherAngle.getAngle( controller->wbRatio, false ) );
	else						updateSpreadBend();

	//cout << "us1" << endl;
	updateAreaRatio();		// area ratio

	// local update, global update
	#pragma omp parallel for
	for(int i=0;i<MaxFeatherName;i++)
	{		
		transf linkTransf	= thisBody->getLinkTransformation((Body::LinkName) convertID_feather2body( i ));
		vector3 linkLinVel	= thisBody->getLinkLinearVelocity((Body::LinkName) convertID_feather2body( i ));
		vector3 linkAngVel	= thisBody->getLinkAngularVelocity((Body::LinkName) convertID_feather2body( i ));

		featherGroup[i]->updateState(linkTransf, linkLinVel, linkAngVel);
	}

	updateOrientation();
	updateBasicInform();	// normal, center of piece, velocity, transform

	if(isFirstSimulation)	
	{
		#pragma omp parallel for
		for(int i=0;i<MaxFeatherName;i++)
		for(int j=0;j<featherGroup[i]->numOfeachFeather;j++)
		for(int k=0;k<featherGroup[i]->each[j]->numOfPiece;k++)
		{
			featherGroup[i]->each[j]->current[k]->velocity.setZero();
		}

		isFirstSimulation = false;
	}
}
void
Feathers::calcAeroForce()
{
	for(int i=0;i<MaxFeatherName;i++)
	{
		position pos = thisBody->getLinkPosition((Body::LinkName) convertID_feather2body( i ));

		featherGroup[i]->calcAeroForce(pos);
	}
}
void
Feathers::applyAeroForce()
{
#ifndef ROOT_FIX
	for(int i=0;i<MaxFeatherName;i++)
	{
		for(int j=0;j<featherGroup[i]->numOfeachFeather;j++)
		{
			thisBody->setTorque( (Body::LinkName) convertID_feather2body( i ), featherGroup[i]->each[j]->totalTorque );
			thisBody->setForce( (Body::LinkName) convertID_feather2body( i ), featherGroup[i]->each[j]->totalForce );
		}
	}
#endif
}
void
Feathers::saveState(int sID)
{
	#pragma omp parallel for
	for(int i=0; i<getNumGroup(); i++)
	{
		*(featherGroupSave[sID][i]) = *(featherGroup[i]);

		for (int j=0; j<featherGroup[i]->getNumFeather(); j++)
		{
			featherGroup[i]->each[j]->saveFemMesh(sID);
		}
	}
}
void
Feathers::restoreState(int sID)
{
	#pragma omp parallel for
	for(int i=0; i<getNumGroup(); i++)
	{
		*(featherGroup[i]) = *(featherGroupSave[sID][i]);

		for (int j=0; j<featherGroup[i]->getNumFeather(); j++)
		{
			featherGroup[i]->each[j]->restoreFemMesh(sID);
		}
	}
}
int
Feathers::getNumGroup()
{
	return (int)featherGroup.size();
}
RenderState_feather
Feathers::getRenderState()
{
	RenderState_feather rs_f;

	for(int i=0; i<getNumGroup(); i++)
	{
		rs_f.groups.push_back( featherGroup[i] );
	}

	return rs_f;
}
void
Feathers::drawFeathers(bool depthTest)
{
	for(int i=0;i<MaxFeatherName;i++)
	{
		featherGroup[i]->drawFeathers(depthTest);
	}
}