#pragma once

#include <vector>
#include <iostream>
#include <ode/ode.h>
#include <MATHCLASS/mathclass.h>

#include "SystemState.h"

struct RigidState
{
	position pos;
	quater ori;
	vector3 linVel;
	vector3 angVel;

	RigidState()
	{
	}

	RigidState(const position p, const quater q, const vector3 lv, const vector3 av)
	{
		pos = p;
		ori = q;
		linVel = lv;
		angVel = av;
	}

	friend ostream& operator<<( ostream& os, RigidState& rs )
	{
		WRITE_ARRAY3_NOENDL(os, rs.pos);
		WRITE_ARRAY4_NOENDL(os, rs.ori);
		WRITE_ARRAY3_NOENDL(os, rs.linVel);
		WRITE_ARRAY3_NOENDL(os, rs.angVel);
		os << endl;
		
		return os;
	}

	friend istream& operator>>( istream& is, RigidState& rs )
	{
		READ_ARRAY3(is, rs.pos);
		READ_ARRAY4(is, rs.ori);
		READ_ARRAY3(is, rs.linVel);
		READ_ARRAY3(is, rs.angVel);
		
		return is;
	}

	RigidState& operator=( const RigidState& _rs )
	{
		pos = _rs.pos;
		ori = _rs.ori;
		linVel = _rs.linVel;
		angVel = _rs.angVel;

		return *this;
	}
};

class PhyEngine
{
public:
	dWorldID					world;
	dSpaceID					space;
	dGeomID						plane;
	dJointGroupID				contactgroup;

	std::vector<dBodyID>		bodyID;
	std::vector<dGeomID>		geomID;
	std::vector<dJointID>		jointID;

	enum		JointTypeName	{ Ball=0, Universal, Hinge };
	enum		Direction		{ Left=0, Right };

	int			count;

	// for save & restore
	int			count_save[max_save];
	std::vector<position>		pos_save[max_save];
	std::vector<quater>			ori_save[max_save];
	std::vector<vector3>		linVel_save[max_save];
	std::vector<vector3>		angVel_save[max_save];

	PhyEngine()
	{
		count = 0;
		initialize();
	};

	~PhyEngine()
	{
		finalize();
	}

	void		initialize();
	void		finalize();

	void		saveState(int sID);
	void		restoreState(int sID);

	void		printBodyState(int id, ofstream& fout);

	static void	nearCallback( void *data, dGeomID o1, dGeomID o2 );
	void		advanceOneStep(double timestep);

	void		setGeom( vector3 lxyz, double mass, vector3 pos, quater rot );
	void		setJoint( int type, int link1, int link2, Direction dir );
	void		setGround( int link );

	/* body functions */
	// all functions return a value by reference frame


	transf		getTransfBody(int id);

	position	getPositionBody(int id);

	quater		getOrientationBody(int id);

	vector3		getAngularVelocityBody(int id);

	vector3		getLinearVelocityBody(int id);

	RigidState	getRigidStateBody(int id);

	void		setTransfBody(int id, transf t);

	void		setPositionBody(int id, position p);

	void		setOrientationBody(int id, quater q);

	void		setAngularVelocityBody(int id, vector3 v);

	void		setLinearVelocityBody(int id, vector3 v);

	void		addTorqueBody( int id, vector3 t);
	void		addForceBody( int id, vector3 f);

	/* joint funtions */

	vector3		getPositionJoint(int id, int type);

	quater		getOrientationJoint(int id);

	vector3		getAngularVelocityJoint(int id);

	vector3		getLinearVelocityJoint(int id);

	void		getAxis(int id, int type, vector3& axis1, vector3& axis2, vector3& axis3);
	void		getAngle(int id, int type, float* ang1, float* ang2);
	void		getAngleRate(int id, int type, float* ang1, float* ang2);

	void		setTorqueJoint( int id, int type, float torque1, float torque2);

	vector3		endOfGeom(int id, Direction dir);
	vector3		bendAxis(int link1, int link2, Direction dir);
	vector3		twistAxis(int link);

	
};