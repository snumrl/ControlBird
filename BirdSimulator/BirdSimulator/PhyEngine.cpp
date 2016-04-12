#include "util.h"
#include "PhyEngine.h"

void
PhyEngine::initialize()
{
	dInitODE();

	world				= dWorldCreate();
	space				= dHashSpaceCreate(0);
	plane				= dCreatePlane(space,0,1,0,0);
	contactgroup		= dJointGroupCreate(0);
	
	dWorldSetGravity(world,0,-9.81f,0);
}
void
PhyEngine::finalize()
{
	bodyID.clear();
	geomID.clear();
	jointID.clear();

	dJointGroupDestroy(contactgroup);
	dSpaceDestroy(space);
	dWorldDestroy(world);

	dCloseODE();
}
void
PhyEngine::nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact))	return;

	const int N = 32; 
	dContact contact[N];   // up to MAX_CONTACTS contacts per box-box
	for (int i=0; i<N; i++) 
	{
		contact[i].surface.mode = dContactBounce;
		contact[i].surface.mu = dInfinity;
		contact[i].surface.mu2 = 0;
		contact[i].surface.bounce = 0.2f;
		contact[i].surface.bounce_vel = 0.1f;
	}
}
void
PhyEngine::advanceOneStep(double timestep)
{
	count++;

	dSpaceCollide(space, 0, &nearCallback);
	dWorldStep(world, timestep);
	//dJointGroupEmpty(contactgroup);	
}
void		
PhyEngine::setGeom( vector3 lxyz, double mass, vector3 pos, quater rot )
{
	dBodyID body = dBodyCreate(world);
	dGeomID geom = dCreateBox(space, lxyz[0], lxyz[1], lxyz[2]);

	dMass m;
	dMassSetBoxTotal(&m, mass, lxyz[0], lxyz[1], lxyz[2]);
	dBodySetMass( body, &m );

	bodyID.push_back(body);
	geomID.push_back(geom);

	dGeomSetBody(geomID.back(),bodyID.back());

	dQuaternion odeQuat;
	quatToOdeQuat(rot, odeQuat);

	dBodySetPosition(bodyID.back(), pos[0], pos[1], pos[2]);
	dBodySetQuaternion(bodyID.back(), odeQuat);

	dBodySetFiniteRotationMode(body, 1);
}
void
PhyEngine::setJoint( int type, int link1, int link2, Direction dir)
{
	Direction offDir;
	
	if(dir == Left)			offDir = Right;
	else if(dir == Right)	offDir = Left;

	dJointID joint;

	vector3 pos = endOfGeom( link2, offDir );

	switch(type)
	{
		case Ball:			joint = dJointCreateBall( world, contactgroup );		break;
		case Universal:		joint = dJointCreateUniversal( world, contactgroup );	break; 
		case Hinge:			joint = dJointCreateHinge( world, contactgroup );		break;
	}

	jointID.push_back(joint);
	dJointAttach( jointID.back(), bodyID[link1], bodyID[link2]);

	switch(type)
	{
		case Ball:			dJointSetBallAnchor(jointID.back(), pos[0], pos[1], pos[2]);		break;
		case Universal:		dJointSetUniversalAnchor(jointID.back(), pos[0], pos[1], pos[2]);	break; 
		case Hinge:			dJointSetHingeAnchor(jointID.back(), pos[0], pos[1], pos[2]);		break;
	}

	vector3 bAxis, tAxis;
	quater frame1, frame2;
	vector3 v1, v2, v3;

	switch(type)
	{
	case Universal:	
		bAxis = bendAxis(link1, link2, dir);
		tAxis = twistAxis(link2);		
		dJointSetUniversalAxis1(jointID.back(), bAxis[0], bAxis[1], bAxis[2]);
		dJointSetUniversalAxis2(jointID.back(), tAxis[0], tAxis[1], tAxis[2]);
		break;
	case Hinge:
		bAxis = bendAxis(link1, link2, dir);
		dJointSetHingeAxis(jointID.back(), bAxis[0], bAxis[1], bAxis[2]);
		break;
	}
}
void 
PhyEngine::setGround( int link )
{
	dJointID joint;

	joint = dJointCreateFixed( world, contactgroup );
	dJointAttach( joint, NULL, bodyID[link]);
}

position
PhyEngine::getPositionBody(int id)
{
	return vector2position( arrToVec(dBodyGetPosition( bodyID[id] )) );
}
quater
PhyEngine::getOrientationBody(int id)
{
	return arrToQuat( dBodyGetQuaternion( bodyID[id] ) );
}
vector3		
PhyEngine::getAngularVelocityBody(int id)
{
	return arrToVec( dBodyGetAngularVel( bodyID[id] ) );
}
vector3		
PhyEngine::getLinearVelocityBody(int id)
{
	return arrToVec( dBodyGetLinearVel( bodyID[id] ) );
}
RigidState
PhyEngine::getRigidStateBody(int id)
{
	RigidState rs;

	rs.pos = getPositionBody(id);
	rs.ori = getOrientationBody(id);
	rs.linVel = getLinearVelocityBody(id);
	rs.angVel = getAngularVelocityBody(id);

	return rs;
}
void
PhyEngine::setTransfBody(int id, transf t)
{
	setOrientationBody( id, t.getRotation() );
	setPositionBody( id, vector2position(t.getTranslation()) );
}
void
PhyEngine::setPositionBody(int id, position p)
{
	dBodySetPosition( bodyID[id], p[0], p[1], p[2] );
}
void
PhyEngine::setOrientationBody(int id, quater q)
{
	dQuaternion dq;
	dq[0] = q[0]; dq[1] = q[1]; dq[2] = q[2]; dq[3] = q[3];
	dBodySetQuaternion( bodyID[id] , dq );
}
void
PhyEngine::setAngularVelocityBody(int id, vector3 v)
{
	dBodySetAngularVel( bodyID[id], v[0], v[1], v[2] );
}
void
PhyEngine::setLinearVelocityBody(int id, vector3 v)
{
	dBodySetLinearVel( bodyID[id], v[0], v[1], v[2] );
}
void		
PhyEngine::addTorqueBody( int id, vector3 t)
{
	dBodyAddTorque( bodyID[id], t[0], t[1], t[2] );
}
void
PhyEngine::addForceBody( int id, vector3 force)
{
	dBodyAddForce( bodyID[id], force[0], force[1], force[2] );
}

vector3
PhyEngine::getPositionJoint(int id, int type)
{
	dVector3 r;
	
	switch(type)
	{
		case Ball:			dJointGetBallAnchor(jointID[id], r);			break;	
		case Hinge:			dJointGetHingeAnchor(jointID[id], r);			break;	
		case Universal:		dJointGetUniversalAnchor(jointID[id], r);		break;	
	}

	return arrToVec( r );
}
quater
PhyEngine::getOrientationJoint(int id)
{
	return arrToQuat( dBodyGetQuaternion( dJointGetBody(jointID[id], 1) ) );
}
void
PhyEngine::getAxis(int id, int type, vector3& axis1, vector3& axis2, vector3& axis3)
{
	axis1 = vector3(0,0,0);
	axis2 = vector3(0,0,0);
	axis3 = vector3(0,0,0);

	dVector3 res;
	quater q;

	switch(type)
	{
	case Ball:
		q = arrToQuat( dBodyGetQuaternion( dJointGetBody(jointID[id], 1) ) );
		axis1 = rotate(q, vector3(1,0,0));
		axis2 = rotate(q, vector3(0,1,0));
		axis3 = rotate(q, vector3(0,0,1));
		break;
	case Universal:	
		dJointGetUniversalAxis1( jointID[id], res );	axis1 = arrToVec( res );
		dJointGetUniversalAxis2( jointID[id], res );	axis2 = arrToVec( res );
		break;
	case Hinge:
		dJointGetHingeAxis( jointID[id], res );			axis1 = arrToVec( res );
		break;
	default: cin.get(); break;
	}
}
void
PhyEngine::getAngle(int id, int type, float* ang1, float* ang2)
{
	*ang1 = 0.0f;
	*ang2 = 0.0f;

	switch(type)
	{
	case Universal:	
		*ang1 = dJointGetUniversalAngle1( jointID[id] );
		*ang2 = dJointGetUniversalAngle2( jointID[id] );	
		break;
	case Hinge:
		*ang1 = dJointGetHingeAngle( jointID[id] );
		break;
	default: cin.get(); break;
	}
}
void
PhyEngine::getAngleRate(int id, int type, float* ang1, float* ang2)
{
	*ang1 = 0.0f;
	*ang2 = 0.0f;

	switch(type)
	{
	case Universal:	
		*ang1 = dJointGetUniversalAngle1Rate( jointID[id] );
		*ang2 = dJointGetUniversalAngle2Rate( jointID[id] );
		break;
	case Hinge:
		*ang1 = dJointGetHingeAngleRate( jointID[id] );
		break;
	}
}
vector3		
PhyEngine::getAngularVelocityJoint(int id)
{
	return vector3();
}

void
PhyEngine::setTorqueJoint( int id, int type, float torque1, float torque2)
{
	switch(type)
	{
	case Universal:	dJointAddUniversalTorques( jointID[id], torque1, torque2 );	break;
	case Hinge:		dJointAddHingeTorque( jointID[id], torque1 );				break;
	default: cin.get(); break;
	}	
}

vector3
PhyEngine::endOfGeom(int id, Direction dir)
{	
	float sign;

	switch(dir)
	{
		case Left:	sign = 1.0f;	break;
		case Right:	sign = -1.0f;	break;
	}

	dVector3 boxLength;
	dGeomBoxGetLengths( geomID[id], boxLength );
	vector3 pos(sign * (boxLength[0] / 2.0f) , 0, boxLength[2]/2.0f);

	pos = getPositionBody(id) + rotate( getOrientationBody(id), pos );

	return pos;
}
vector3
PhyEngine::bendAxis(int link1, int link2, Direction dir)
{
	Direction offDir;

	if(dir == Left)		offDir = Right;
	else				offDir = Left;

	vector3 v1 = endOfGeom( link1, offDir );
	vector3 v2 = endOfGeom( link1, dir );
	vector3 v3 = endOfGeom( link2, dir );

	vector3 axis;

	switch(dir)
	{
		case Left:	axis = (v3-v2)*(v1-v2);	break;
		case Right:	axis = (v1-v2)*(v3-v2);	break;
	}

	axis = normalize( axis );

	return axis;
}
vector3
PhyEngine::twistAxis(int link)
{
	vector3 v1 = endOfGeom( link, Left );
	vector3 v2 = endOfGeom( link, Right );
	
	return normalize(v1-v2);
}

void
PhyEngine::saveState(int sID)
{
	if(sID > max_save-1)
	{
		cerr << "Max number of state for save : " << max_save << endl;
		cin.get(); cin.get();
	}

	count_save[sID] = count;

	pos_save[sID].clear();
	ori_save[sID].clear();
	linVel_save[sID].clear();
	angVel_save[sID].clear();

	for(int i=0; i<(int)bodyID.size(); i++)
	{
		pos_save[sID].push_back( getPositionBody(i) );
		ori_save[sID].push_back( getOrientationBody(i) );
		linVel_save[sID].push_back( getLinearVelocityBody(i) );
		angVel_save[sID].push_back( getAngularVelocityBody(i) );
	}
}

void
PhyEngine::restoreState(int sID)
{
	if(pos_save[sID].empty())
	{
		cerr << "Phy Enging : not saved at " << sID << endl;
		cin.get(); cin.get();
	}

	if(sID > max_save-1)
	{
		cerr << "Max number of state for save : " << max_save << endl;
		cin.get(); cin.get();
	}

	count = count_save[sID];

	for(int i=0; i<(int)bodyID.size(); i++)
	{
		setPositionBody(i, pos_save[sID][i]);
		setOrientationBody(i, ori_save[sID][i]);
		setLinearVelocityBody(i, linVel_save[sID][i]);
		setAngularVelocityBody(i, angVel_save[sID][i]);
	}
}

void
PhyEngine::printBodyState(int id, ofstream& fout)
{
	fout << getRigidStateBody(id);
}