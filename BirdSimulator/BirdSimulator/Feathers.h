#pragma once

#include "definitions.h"
#include "Body.h"
#include "GL/glut.h"
#include <FemFeather/FeatherConstants.h>
#include <FemFeather/FeatherShellMesh.h>

#include "SystemState.h"

const static char meshNamePrimary_L[128]	= "Informations/model/feather_primary_L";
const static char meshNamePrimary_R[128]	= "Informations/model/feather_primary_R";
const static char meshNameSecondary_L[128]	= "Informations/model/feather_secondary_L";
const static char meshNameSecondary_R[128]	= "Informations/model/feather_secondary_R";
const static char meshNameTertiary_L[128]	= "Informations/model/feather_tertiary_L";
const static char meshNameTertiary_R[128]	= "Informations/model/feather_tertiary_R";
const static char meshNameTail_L[128]		= "Informations/model/feather_tail_L";
const static char meshNameTail_R[128]		= "Informations/model/feather_tail_R";

struct RenderState_feather;

// 새가 날개를 접는 모습 control
struct 
OneFeatherAngle
{
	double		keyFrame;

	double		spread_from[3];
	double		spread_to[3];
	double		bend_from[3];
	double		bend_to[3];

	OneFeatherAngle& operator=( const OneFeatherAngle& _ofa )
	{
		keyFrame = _ofa.keyFrame;
		
		for(int i=0; i<3; i++)
		{
			spread_from[i]	= _ofa.spread_from[i];
			spread_to[i]	= _ofa.spread_to[i];
			bend_from[i]	= _ofa.bend_from[i];
			bend_to[i]		= _ofa.bend_to[i];
		}

		return *this;
	}

	friend ostream& operator<<( ostream& os, const OneFeatherAngle& ofs )
	{
		for(int i=0; i<3; i++)
		{
			os << ofs.spread_from[i] <<	"      " << ofs.spread_to[i] << endl;
			os << ofs.bend_from[i]	 << "      " << ofs.bend_to[i] << endl;
		}
		os << endl;

		return os;
	}
};
class 
FeatherAngle
{
public:
	util::SPLINE* curve_spread_from[3];
	util::SPLINE* curve_spread_to[3];
	util::SPLINE* curve_bend_from[3];
	util::SPLINE* curve_bend_to[3];

	vector< point2 > value_spread_from[3];
	vector< point2 > value_spread_to[3];
	vector< point2 > value_bend_from[3];
	vector< point2 > value_bend_to[3];
	
	//
	float stopFrame;
	//
	vector<OneFeatherAngle> oneFeatherAngles;

	FeatherAngle() { stopFrame = 1.0f; }

	bool loaded() { return !oneFeatherAngles.empty(); }

	int numKeyFrame() { return (int)oneFeatherAngles.size(); }

	int binarySearch(float keyFrame, const vector< point2 >& v, int s, int e)
	{
		int m = (s+e)/2;


		if( (e-s == 1) && (keyFrame >= v[s].x) && (keyFrame < v[e].x) ) return s;
		if( (e-s == 1) && (keyFrame > v[s].x) && (keyFrame <= v[e].x) ) return e;

		if(keyFrame == v[m].x) return m;
		else if( keyFrame < v[m].x )	return binarySearch(keyFrame, v, s, m);
		else							return binarySearch(keyFrame, v, m, e);
	}
	int binarySearch(float keyFrame, const vector< OneFeatherAngle >& v, int s, int e)
	{
		int m = (s+e)/2;


		if( (e-s == 1) && (keyFrame >= v[s].keyFrame) && (keyFrame < v[e].keyFrame) ) return s;
		if( (e-s == 1) && (keyFrame > v[s].keyFrame) && (keyFrame <= v[e].keyFrame) ) return e;

		if(keyFrame == v[m].keyFrame) return m;
		else if( keyFrame < v[m].keyFrame )	return binarySearch(keyFrame, v, s, m);
		else								return binarySearch(keyFrame, v, m, e);
	}

	void readData()
	{
		cout << "---------------------------------------------------" << endl;

		oneFeatherAngles.clear();

		ifstream fin("Informations/featherAngle/featherAngle.txt");

		if(fin.fail())
		{
			cout << " Cannot open feather angle file !!! " << endl;
			cout << "---------------------------------------------------" << endl;
			return;
		}

		int numberOfKeyFrame;

		fin >> numberOfKeyFrame;

		for(int i=0; i<numberOfKeyFrame; i++)
		{
			OneFeatherAngle ofa;

			fin >> ofa.keyFrame;

			fin >> ofa.spread_from[0];	fin >> ofa.spread_to[0];
			fin >> ofa.bend_from[0];	fin >> ofa.bend_to[0];

			fin >> ofa.spread_from[1];	fin >> ofa.spread_to[1];
			fin >> ofa.bend_from[1];	fin >> ofa.bend_to[1];
			
			fin >> ofa.spread_from[2];	fin >> ofa.spread_to[2];
			fin >> ofa.bend_from[2];	fin >> ofa.bend_to[2];

			oneFeatherAngles.push_back(ofa);
		}

		cout << "complete import <feather angle> : " << oneFeatherAngles.size() << endl;
		constructSpline();
		cout << "---------------------------------------------------" << endl;
	}

	void constructSpline()
	{
		if(!loaded()) return;

		vector<double> ctrPnts_spread_from[3];
		vector<double> ctrPnts_spread_to[3];
		vector<double> ctrPnts_bend_from[3];
		vector<double> ctrPnts_bend_to[3];

		for(int j=0; j<3; j++)
		{
			curve_spread_from[j]	= new util::SPLINE(2, 3);
			curve_spread_to[j]		= new util::SPLINE(2, 3);
			curve_bend_from[j]		= new util::SPLINE(2, 3);
			curve_bend_to[j]		= new util::SPLINE(2, 3);
		}

		for(int i=0; i<numKeyFrame(); i++)
		{
			for(int j=0; j<3; j++)
			{
				ctrPnts_spread_from[j].push_back( oneFeatherAngles[i].keyFrame );
				ctrPnts_spread_from[j].push_back( oneFeatherAngles[i].spread_from[j] );

				ctrPnts_spread_to[j].push_back( oneFeatherAngles[i].keyFrame );
				ctrPnts_spread_to[j].push_back( oneFeatherAngles[i].spread_to[j] );

				ctrPnts_bend_from[j].push_back( oneFeatherAngles[i].keyFrame );
				ctrPnts_bend_from[j].push_back( oneFeatherAngles[i].bend_from[j] );

				ctrPnts_bend_to[j].push_back( oneFeatherAngles[i].keyFrame );
				ctrPnts_bend_to[j].push_back( oneFeatherAngles[i].bend_to[j] );
			}
		}

		for(int j=0; j<3; j++)
		{
			float s, e;

			//curve_spread_from[j]->genInterpCurve( ctrPnts_spread_from[j], true );
			//curve_spread_to[j]->genInterpCurve( ctrPnts_spread_to[j], true );
			//curve_bend_from[j]->genInterpCurve( ctrPnts_bend_from[j], true );
			//curve_bend_to[j]->genInterpCurve( ctrPnts_bend_to[j], true );

			//curve_spread_from[j]->genCatMullRommLikeCurve( ctrPnts_spread_from[j] );
			//curve_spread_to[j]->genCatMullRommLikeCurve( ctrPnts_spread_to[j] );
			//curve_bend_from[j]->genCatMullRommLikeCurve( ctrPnts_bend_from[j] );
			//curve_bend_to[j]->genCatMullRommLikeCurve( ctrPnts_bend_to[j] );

			curve_spread_from[j]->genCurve( ctrPnts_spread_from[j] , true );
			curve_spread_to[j]->genCurve( ctrPnts_spread_to[j] , true );
			curve_bend_from[j]->genCurve( ctrPnts_bend_from[j] , true );
			curve_bend_to[j]->genCurve( ctrPnts_bend_to[j] , true );

			int detail = 2400;
			vector<double> out;

			value_spread_from[j].clear(); 
			value_spread_to[j].clear(); 
			value_bend_from[j].clear(); 
			value_bend_to[j].clear(); 

			for(int k=0; k<detail; k++)
			{
				out = curve_spread_from[j]->getPoint((double)k/(detail-1));
				value_spread_from[j].push_back( point2(out[0], out[1]) );

				out.clear();

				out = curve_spread_to[j]->getPoint((double)k/(detail-1));
				value_spread_to[j].push_back( point2(out[0], out[1]) );

				out.clear();

				out = curve_bend_from[j]->getPoint((double)k/(detail-1));
				value_bend_from[j].push_back( point2(out[0], out[1]) );

				out.clear();

				out = curve_bend_to[j]->getPoint((double)k/(detail-1));
				value_bend_to[j].push_back( point2(out[0], out[1]) );
			}

			curve_spread_from[j]->getParameterRange(s, e);
			//cout << "spread_from < " << j << " > : " << s << "    " << e << endl;
			curve_spread_to[j]->getParameterRange(s, e);
			//cout << "spread_to < " << j << " > : " << s << "    " << e << endl;
			curve_bend_from[j]->getParameterRange(s, e);
			//cout << "bend_from < " << j << " > : " << s << "    " << e << endl;
			curve_bend_to[j]->getParameterRange(s, e);
			//cout << "bend_to < " << j << " > : " << s << "    " << e << endl;

			//cin.get();
		}

		cout << "complete spline curve : " << value_spread_from[0].size() <<endl;
	}

	// t : current simulation ratio 0~1(one wingbeat cycle)
	OneFeatherAngle getAngle(const double t, const bool linear=true)
	{
		assert( (t>=0) && (t<=1) );

		OneFeatherAngle ofa;

		ofa.keyFrame = t;

		double curFrame, nextFrame;
		int cur, next;
		int size;
		double alpha;

		if(linear)	cur = binarySearch(t, oneFeatherAngles, 0, (int)oneFeatherAngles.size()-1);
		else		cur = binarySearch(t, value_spread_from[0], 0, (int)value_spread_from[0].size()-1);

		//cout << t << "       " << t * (size-1) << "        " << cur << endl;
		//cin.get();

		if(t==1.0f)				next = cur;
		else					next = cur+1;

		if(linear)
		{
			curFrame = oneFeatherAngles[cur].keyFrame;
			nextFrame = oneFeatherAngles[next].keyFrame;
		}
		else
		{
			curFrame = value_spread_from[0][cur].x;
			nextFrame = value_spread_from[0][next].x;
		}

		if(t==1.0f)				alpha = 0.0f;
		else					alpha = (t-curFrame)/(nextFrame-curFrame);

		//cout << "t: " << t << "    curFrame: " << curFrame << "     nextFrame: " << nextFrame << endl;
		//cin.get();

		assert( alpha>=0.0f && alpha<=1.0f );

		//alpha = 0;

		if(linear)
		{
			for(int i=0; i<3; i++)
			{
				ofa.spread_from[i]	= (1.0f-alpha) * oneFeatherAngles[cur].spread_from[i]	+ alpha * oneFeatherAngles[next].spread_from[i];
				ofa.spread_to[i]	= (1.0f-alpha) * oneFeatherAngles[cur].spread_to[i]		+ alpha * oneFeatherAngles[next].spread_to[i];
				ofa.bend_from[i]	= (1.0f-alpha) * oneFeatherAngles[cur].bend_from[i]		+ alpha * oneFeatherAngles[next].bend_from[i];
				ofa.bend_to[i]		= (1.0f-alpha) * oneFeatherAngles[cur].bend_to[i]		+ alpha * oneFeatherAngles[next].bend_to[i];
			}
		}
		else
		{
			for(int i=0; i<3; i++)
			{
				ofa.spread_from[i]	= (1.0f-alpha) * value_spread_from[i][cur].y			+ alpha * value_spread_from[i][next].y;
				ofa.spread_to[i]	= (1.0f-alpha) * value_spread_to[i][cur].y				+ alpha * value_spread_to[i][next].y;
				ofa.bend_from[i]	= (1.0f-alpha) * value_bend_from[i][cur].y				+ alpha * value_bend_from[i][next].y;
				ofa.bend_to[i]		= (1.0f-alpha) * value_bend_to[i][cur].y				+ alpha * value_bend_to[i][next].y;
			}

			/*cout << "-=-=-=-=-=-=-=-=" << endl;
			cout << cur << " " << next << " " << alpha << " " << curFrame << " " << nextFrame << endl;
			cout << ofa;*/
		}

		if(false)
		{
			cout << "---------------------------------------------" << endl;
			cout << ofa;
			cout << "---------------------------------------------" << endl;
		}
	
		return ofa;
	}

	void reset()
	{
		oneFeatherAngles.clear();

		readData();
	}

	void draw(const int i, const int j, bool linear=true)
	{
		if(!loaded()) return;

		int detail = 100;
		vector3 color = autoColor(i);

		glDisable(GL_LIGHTING);

		glColor3f(color[0], color[1], color[2]);
		glLineWidth(2.0f);
		glBegin(GL_LINE_STRIP);

			for(int k=0; k<detail; k++)
			{
				OneFeatherAngle ofa = getAngle((double)k/(detail-1), linear);
				float value;

				switch(i)
				{
				case 0: value = ofa.spread_from[j]; break;
				case 1: value = ofa.spread_to[j];	break;
				case 2: value = ofa.bend_from[j];	break;
				case 3: value = ofa.bend_to[j];		break;
				}

				glVertex2f(ofa.keyFrame, value);
			}

		glEnd();

		/*glBegin(GL_LINE_STRIP);
			for(int k=0; k<(int)value_spread_from[0].size(); k++)
			{
				switch(i)
				{
				case 0: glVertex2f( value_spread_from[j][k][0], value_spread_from[j][k][1] );	break;
				case 1: glVertex2f( value_spread_to[j][k][0], value_spread_to[j][k][1] );		break;
				case 2: glVertex2f( value_bend_from[j][k][0], value_bend_from[j][k][1] );		break;
				case 3: glVertex2f( value_bend_to[j][k][0], value_bend_to[j][k][1] );			break;
				}
			}
		glEnd();*/

		glEnable(GL_LIGHTING);
	}

	/*void draw_linear(int i, int j)
	{
		if(!loaded()) return;

		vector3 color = autoColor(i);

		glDisable(GL_LIGHTING);

		glColor3f(color[0], color[1], color[2]);
		glLineWidth(2.0f);
		glBegin(GL_LINE_STRIP);

			for(int k=0; k<numKeyFrame()-1; k++)
			{
				float v;
				float t = oneFeatherAngles[k].keyFrame;

				switch(i)
				{
				case 0: v = oneFeatherAngles[k].spread_from[j];		break;
				case 1: v = oneFeatherAngles[k].spread_to[j];		break;
				case 2: v = oneFeatherAngles[k].bend_from[j];		break;
				case 3: v = oneFeatherAngles[k].bend_to[j];			break;
				}

				glVertex2f(t, v);
			}

		glEnd();

		glEnable(GL_LIGHTING);
	}*/

	void draw_point(int i, int j)
	{
		vector3 color = autoColor(i);

		glDisable(GL_LIGHTING);

		glColor3f(1-color[0], 1-color[1], 1-color[2]);
		glPointSize(10.0f);
		glBegin(GL_POINTS);

			for(int k=0; k<numKeyFrame(); k++)
			{
				float v;
				float t = oneFeatherAngles[k].keyFrame;

				switch(i)
				{
				case 0: v = oneFeatherAngles[k].spread_from[j];		break;
				case 1: v = oneFeatherAngles[k].spread_to[j];		break;
				case 2: v = oneFeatherAngles[k].bend_from[j];		break;
				case 3: v = oneFeatherAngles[k].bend_to[j];			break;
				}

				glVertex2f(t, v);
			}

		glEnd();

		glEnable(GL_LIGHTING);
	}
};

class Mesh
{
public:
	struct Edge
	{
		int e[4];
	};
	vector<position>			node;
	vector<position>			element;
	vector<Edge>				edge;
	vector< vector<position> >	nodeID2element; // p[0] : element id, p[1] : where it has
	float						totalArea;

	inline int	getNumNodes()		{ return (int)node.size(); }
	inline int	getNumElements()	{ return (int)element.size(); }
	inline int	getNumEdges()		{ return (int)edge.size(); }
	inline position getNodePositionInElement(const int eleID, const int nodeID) 
	{ 
		assert(eleID < getNumElements());
		assert(nodeID < 3);
		
		return node.at( (int)element.at(eleID)[nodeID] );
	}

	void computeTotalArea()
	{
		totalArea = 0.0f;
		for(int i=0; i<getNumElements(); i++)
		{
			position p0 = getNodePositionInElement(i, 0);
			position p1 = getNodePositionInElement(i, 1);
			position p2 = getNodePositionInElement(i, 2);

			totalArea += triangleArea( p0, p1, p2 );
		}
	}
	void computeNodeID2element()
	{
		for(int i=0; i<getNumNodes(); i++)
		{
			vector<position> n2e;

			for(int j=0; j<getNumElements(); j++)
			for(int k=0; k<3; k++)
			{
				if( i == element[j][k] ) n2e.push_back(position(j,k,0));
			}

			nodeID2element.push_back(n2e);
		}
	}
	inline position getPinnedPosition() { return position(0,0,0); }
	inline vector3	getTwistAxis()		{ return vector3(0,0,-1); }
};

// state of each tries
struct EachPiece
{
	// node index
	position					nodeIdx;
	// positions of nodes
	position					pos[3];
	// position of center of piece
	position					centerOfPiece;
	// normal of piece
	vector3						normal;
	// orientation of nodes	(calc from init pos)
	quater						ori;	

	// drag force direction
	vector3						dragD;
	// lift force direction
	vector3						liftD;
	// relative velocity
	vector3						velocity;
	// angle of attack
	float						aoa;
	// lift coefficient
	float						coeffLift;
	// drag coefficient;
	float						coeffDrag;
	// piece area
	float						area;

	// drag force
	vector3						dragForce;		
	// lift force
	vector3						liftForce;
	// moment arm from the center of mass (link) to area center (feather)
	vector3						momentArmToBody;
	// net force
	vector3						netForce;
	//
	vector3						twistTorque;

	vector3						dragForce_fem;
	vector3						liftForce_fem;
	vector3						netForce_fem;

	EachPiece()									// triangle(3)
	{
	};

	~EachPiece()
	{
	};

	EachPiece& operator=( const EachPiece& _ep )
	{
		for(int i=0; i<3; i++)	pos[i] = _ep.pos[i];
		centerOfPiece	= _ep.centerOfPiece;
		normal			= _ep.normal;
		ori				= _ep.ori;
		dragD			= _ep.dragD;
		liftD			= _ep.liftD;
		velocity		= _ep.velocity;
		aoa				= _ep.aoa;
		coeffLift		= _ep.coeffLift;
		coeffDrag		= _ep.coeffDrag;
		area			= _ep.area;
		dragForce		= _ep.dragForce;
		liftForce		= _ep.liftForce;
		momentArmToBody	= _ep.momentArmToBody;
		netForce		= _ep.netForce;
		twistTorque		= _ep.twistTorque;

		return *this;
	}

	void	drawSurface()
	{
		glBegin(GL_TRIANGLES);
			glNormal3f(normal[0],normal[1],normal[2]);
			for(int j=0;j<3;j++) 
				glVertex3f(pos[j][0],pos[j][1],pos[j][2]);
		glEnd();
	}
	void	drawSurfaceBoundary()
	{
		glDisable(GL_LIGHTING);
		glColor3f(0.5,0.5,0.5);
		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		glBegin(GL_TRIANGLES);
			glNormal3f(normal[0],normal[1],normal[2]);
			for(int j=0;j<3;j++) SET_GL_VERTEX(pos[j]);
		glEnd();
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		glEnable(GL_LIGHTING);
	}
	void	drawDragForce()
	{
		drawLine_start_vector(centerOfPiece, dragForce);
	}
	void	drawLiftForce()
	{
		drawLine_start_vector(centerOfPiece, liftForce);
	}
	void	drawNetForce()
	{
		drawLine_start_vector(centerOfPiece, netForce);
		//cout << netForce << endl;
	}

	void	drawNormal()
	{
		drawLine_start_vector(centerOfPiece, normal);
	}

	void	drawVelocity()
	{
		drawLine_start_vector(centerOfPiece, velocity);
	}
	
	void	drawMarker(int i)
	{
		glDisable(GL_LIGHTING);
		glColor3f(0,0,0);
		
		glPushMatrix();
		glTranslatef(pos[i][0], pos[i][1], pos[i][2]);
			glutSolidSphere(0.002, 5, 5);
		glPopMatrix();

		glEnable(GL_LIGHTING);
	}

	// calculate aerodynamics force
	void	calcAeroForce(position bodyPos, float areaRatio);
	void	updateBasicInform();
};

class EachFeather
{
private:
	class Feathers*			this_feather;
public:
	int						cnt;

	bool					initialized;

	int						simMode;

	// linear velocity of nodes in global
	vector3					linVel;
	// linear accelaration of nodes in global
	vector3					linAcc;
	// linear velocity of nodes in global
	vector3					angVel;
	// angular accelaration of nodes in global
	vector3					angAcc;
	//
	float					totalTwistTorque;

	// a feather group that have this feather
	int			groupID;
	// index within group
	int			orderInGroup;
	// number of sibling(in group)
	int			numSibling;
	// total number of triangles
	int			numOfPiece;									// 마지막 조각은 항상 triangle로
	// long direction length
	float		length;										// 깃털 길이
	// end tri's length / total length
	float		lengthRatio;								// 전체 길이중에 끝에 뾰족한 부분의 비율
	// short direction length
	float		width;										// 넓이
	// 
	float		widthRatio;									// 깃털의 좌우 비율 (0.5보다 작으면 오른쪽으로 치우친 깃털, 0.5보다 크면 왼쪽으로 치우친 깃털)						
	float		linkLength;									// 연결되어 있는 link의 길이
	float		linkRatio;									// 연결되어 있는 link에서의 상대적 위치
	float		z_trans;									

	std::vector<float>			bendAng;
	float						twistAng;
	float						spreadAng;
	float						initSpreadAng;

	// geometric mesh
	Mesh						mesh;
	std::vector<EachPiece*>		init;
	std::vector<EachPiece*>		pprev;
	std::vector<EachPiece*>		prev;
	std::vector<EachPiece*>		current;
	
	// total area
	float						area;
	float						endTriArea;
	float						areaRatio;

	vector3						totalForce;
	vector3						totalTorque;

	// FEM mesh
	FeatherShellMesh*			femMesh;
	FeatherState				femMeshState[max_save];
	vector<vector3>				fem_dragForce_save[max_save];
	vector<vector3>				fem_liftForce_save[max_save];
	vector<vector3>				fem_netForce_save[max_save];

	// link transformation
	transf						linkTransf;
	// l to f transformation
	transf						linkToFeatherTransf;
	// initial transformation
	transf						initTransf;
	// currnet transformation
	transf						curTransf;

	EachFeather(int gID, int order, int numSib, int num)
	{
		cnt				= 0;
		initialized		= false;

		groupID			= gID;
		orderInGroup	= order;
		numSibling		= numSib;
		numOfPiece		= num;

		for(int i=0;i<numOfPiece;i++)
		{
			int nn;
			bendAng.push_back(0.0f);

			EachPiece*	pprevE		= new EachPiece();
			EachPiece*	prevE		= new EachPiece();
			EachPiece*	initE		= new EachPiece();
			EachPiece*	currE		= new EachPiece();

			pprev.push_back(pprevE);
			prev.push_back(prevE);
			init.push_back(initE);
			current.push_back(currE);
		}

		// FEM mesh
		femMesh = new FeatherShellMesh();
	}

	~EachFeather()
	{
		delete	femMesh;
	}

	EachFeather& operator=( const EachFeather& _ef )
	{
		cnt					= _ef.cnt;
		initialized			= _ef.initialized;

		linVel				= _ef.linVel;
		linAcc				= _ef.linAcc;
		angVel				= _ef.angVel;
		angAcc				= _ef.angAcc;

		totalTwistTorque	= _ef.totalTwistTorque;

		bendAng.clear();
		for(int i=0; i<(int)bendAng.size(); i++)
		{
			bendAng.push_back( _ef.bendAng[i] );
		}
		
		twistAng				= _ef.twistAng;
		spreadAng				= _ef.spreadAng;
		initSpreadAng			= _ef.initSpreadAng;

		for (int i=0; i<getNumElements(); i++)
		{
			*(prev[i])			= *(_ef.prev[i]);
			*(pprev[i])			= *(_ef.pprev[i]);
			//*(init[i])			= *(_ef.init[i]);
			*(current[i])		= *(_ef.current[i]);
		}

		area				= _ef.area;
		areaRatio			= _ef.areaRatio;

		totalForce			= _ef.totalForce;
		totalTorque			= _ef.totalTorque;

		linkTransf			= _ef.linkTransf;
		linkToFeatherTransf = _ef.linkToFeatherTransf;
		initTransf			= _ef.initTransf;
		curTransf			= _ef.curTransf;
		
		return *this;
	}

	void	setInformation(float length, float lengthRatio, float width, float widthRatio, float linkLength, float linkRatio, float sAng, float z_trans);

	void	initialize();
	void	constructFeather(bool left, const char* name, const vector3 scale=vector3(1,1,1));
	void	constructPrimaryFeather();
	void	constructSecondaryFeather();
	void	constructTertiaryFeather();
	void	constructTailFeather();
	void	loadFeatherMesh(const char* name, const vector3 scale=vector3(1,1,1));
	void	loadPrimaryFeatherMesh();
	void	loadSecondaryFeatherMesh();
	void	loadTertiaryFeatherMesh();
	void	loadTailFeatherMesh();
	// update bend, twist btw a piece and a piece
	void	updateLocalState();

	void	updateGlobalState(transf linkTransf, vector3 linkLinVel, vector3 linkAngVel, float areaRatio);
	void	updatePosition();
	void	updateRotation();
	void	updateOrientation(bool isLeft);
	void	updateBendAngle(double K_s, double K_d);
	void	updateTwistAngle(double K_s, double K_d);
	void	updateBasicInform();
	void	calcAeroForce(position bodyPos, float areaRatio);
	bool	isLeft();
	bool	isPrimary();
	bool	isSecondary();
	bool	isTertiary();
	bool	isTail();

	void	constructFemMesh(bool left);
	void	constructFemMesh_primary();
	void	constructFemMesh_secondary();
	void	constructFemMesh_tertiary();
	void	constructFemMesh_tail();
	void	updateFemMesh(float areaRatio);
	void	updateFemMesh_primary();
	void	updateFemMesh_secondary();
	void	updateFemMesh_tertiary();
	void	updateFemMesh_tail();
	void	saveFemMesh(int sID);
	void	restoreFemMesh(int sID);


	position	getPinnedPositionLocal();
	position	getPinnedPositionGlobal();
	vector3		getTwistAxis();
	int			getNumNodes();
	int			getNumElements();
	int			getNumEdges();
	transf		getInitTransf();
	transf		getInitToCurrentTransf();
	vector3		getLinearVel();
	vector3		getLinearAcc();
	vector3		getAngularVel();
	vector3		getAngularAcc();

	void	drawEachInform();
	void	drawEachFeather(bool depthTest=true);
};

struct GroupFeathers
{
	int		numOfeachFeather;
	int		numOfPiece;

	float	area;
	float	areaRatio;

	// <start , end>
	std::pair<float,float>	length;
	std::pair<float,float>	lengthRatio;
	std::pair<float,float>	width;
	std::pair<float,float>	widthRatio;
	std::pair<float,float>	spreadAngle;
	
	std::vector<EachFeather*>	each;			// 좌우 모두 왼쪽이 start feather

	GroupFeathers& operator=( const GroupFeathers& _gf )
	{
		area		= _gf.area;
		areaRatio	= _gf.areaRatio;
		spreadAngle	= _gf.spreadAngle;

		for (int i=0; i<getNumFeather(); i++)
		{
			*each[i] = *(_gf.each[i]);
		}
		
		return *this;
	}

	GroupFeathers()
	{
		area = 0.0f;
		areaRatio = 1.0f;
	};

	void	setEachFeather(int idx, float linkLength, float linkRatio, float z_trans)
	{
		// left : 0, right : 1
		float ratio		= (float)idx / (numOfeachFeather-1);
		// total length
		float t_leng	= length.first		* (1.0f-ratio) + length.second		* ratio;
		// end tri length
		float t_lengR	= lengthRatio.first	* (1.0f-ratio) + lengthRatio.second	* ratio;
		// total width
		float t_width	= width.first		* (1.0f-ratio) + width.second		* ratio;
		// shaft length
		float t_widthR	= widthRatio.first	* (1.0f-ratio) + widthRatio.second	* ratio;
		// spread angle
		float t_spread	= spreadAngle.first	* (1.0f-ratio) + spreadAngle.second	* ratio;
		
		each[idx]->setInformation(t_leng, t_lengR, t_width, t_widthR, linkLength, linkRatio, AngToRad(t_spread), z_trans);
	}
	
	void	swapInform()
	{
		float temp;

		temp = length.first;		length.first		= length.second;		length.second		= temp;
		temp = lengthRatio.first;	lengthRatio.first	= lengthRatio.second;	lengthRatio.second	= temp;
		temp = width.first;			width.first			= width.second;			width.second		= temp;
		temp = widthRatio.first;	widthRatio.first	= widthRatio.second;	widthRatio.second	= temp;
		temp = spreadAngle.first;	spreadAngle.first	= spreadAngle.second;	spreadAngle.second	= temp;

		widthRatio.first	= 1.0f - widthRatio.first;
		widthRatio.second	= 1.0f - widthRatio.second;
		spreadAngle.first	*= -1.0f;
		spreadAngle.second	*= -1.0f;
	}

	void	updateState(transf linkTransf, vector3 linkLinVel, vector3 linkAngVel)
	{
		for(int i=0;i<numOfeachFeather;i++)
		{
			each[i]->updateLocalState();
			each[i]->updateGlobalState(linkTransf, linkLinVel, linkAngVel, areaRatio);
		}
	}

	void	updateBasicInform()
	{
		for(int i=0;i<numOfeachFeather;i++)
		{
			each[i]->updateBasicInform();
		}
	}

	void	updateBendAngle(double K_s, double K_d)
	{
		for(int i=0;i<numOfeachFeather;i++)
		{
			each[i]->updateBendAngle(K_s, K_d);
		}
	}
	void	updateTwistAngle(double K_s, double K_d)
	{
		for(int i=0;i<numOfeachFeather;i++)
		{
			each[i]->updateTwistAngle(K_s, K_d);
		}
	}

	void	calcAeroForce(position pos)
	{
		for(int i=0;i<numOfeachFeather;i++)
		{
			each[i]->calcAeroForce(pos, areaRatio);
		}
	}

	int		getNumFeather()
	{
		return (int)each.size();
	}

	void	drawFeathers(bool depthTest)
	{
		for(int j=0;j<numOfeachFeather;j++)
		{
			each[j]->drawEachFeather(depthTest);
			each[j]->drawEachInform();
		}
	}
};

class Feathers
{
public:
	enum	FeatherName { PrimL=0, PrimR, SeconL, SeconR, TertL, TertR, Tail, MaxFeatherName };
	enum	FeatherSimulMode { None=0, MassSpring, ThinShell };

	Body*				thisBody; 
	bool				isFirstSimulation;

	vector3				color;

	std::vector<GroupFeathers*>		featherGroupSave[max_save];
	std::vector<GroupFeathers*>		featherGroup;
	FeatherSimulMode				simulMode;

	FeatherAngle					featherAngle;
		
	Feathers ( Body* body, FeatherSimulMode simMode )
	{
		thisBody			= body;
		simulMode			= simMode;
		isFirstSimulation	= true;

		loadFeathersInformation();
		setFeathers();
		updateState();
	};

	void				saveState(const int sID);
	void				restoreState(const int sID);

	// body id that connected to given feather id
	static int			convertID_feather2body(int id)
	{
		int idx;

		switch(id)
		{
			case PrimL:		idx = Body::Lwing3;		break;
			case PrimR:		idx = Body::Rwing3;		break;
			case SeconL:	idx = Body::Lwing2;		break;
			case SeconR:	idx = Body::Rwing2;		break;
			case TertL:		idx = Body::Lwing1;		break;
			case TertR:		idx = Body::Rwing1;		break;
			case Tail:		idx = Body::Trunk;		break;
		}

		return idx;
	}

	void				loadFeathersInformation();	
	void				setFeathers();
	void				updateState();
	void				applyAeroForce();
	void				updateBasicInform();
	void				updateOrientation();
	void				updateAreaRatio();
	void				updateBendAngle();
	void				updateTwistAngle();
	void				calcAeroForce();

	// update spread, bend angle btw body line and feather
	void				updateSpreadBend();
	void				updateSpreadBend2();
	void				updateSpreadBend3();
	void				updateSpreadBend_manual(const OneFeatherAngle& ofa);
	//
	void				spreadFromTo(int idx, float from, float to);
	//
	void				bendFromTo(int idx, float from, float to);
	//
	void				applyFeatherAngle();

	int					getNumGroup();

	RenderState_feather	getRenderState();

	// this T is global transformation 
	// from trunk trasforamtion to desired transformation
	void				putBirdAt(transf T);

	void				drawFeathers(bool depthTest=true);
};

struct RenderState_feather
{
	vector< GroupFeathers* > groups;

	RenderState_feather& operator=( const RenderState_feather& _rs_f )
	{
		groups.clear();
		for(int i=0; i<(int)_rs_f.groups.size(); i++)
		{
			groups.push_back( _rs_f.groups[i] );
		}
	}
};