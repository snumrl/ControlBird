#include "Body.h"
#include "Bird.h"
#include "CaptureMotion.h"
#include "Controller.h"
#include "Coeff.h"
#include "util.h"
#include "SystemState.h"
#include "MeshModel.h"
#include "RenderBird.h"
#include "GL/glut.h"

extern Coeff*			coeff;
extern Bird*			simBird;
extern CaptureMotion*	captureBird;
extern PhyEngine*		phyEngine;
extern Controller*		controller;
extern SystemState*		sysState;
extern MeshModel*		meshModel;
extern RenderBird*		rendBird;

bool
Body::isFirstSimulation()
{
	return (count==1);
}
void
Body::loadBodyInformation()
{
	ifstream fin;
	util::fileOpenWrapper(fin, INFORM_DIR+string("Body_inform.txt"));

	for(int i=0;i<3;i++)	initialPosition[i]	= atof(getNext(fin, "//").c_str());			
	for(int i=0;i<3;i++)	initialVelocity[i]	= atof(getNext(fin, "//").c_str());
	for(int i=0;i<2;i++)	wingPosition[i]		= atof(getNext(fin, "//").c_str());

	totalMass = 0.0f;

	for(int i=0;i<maxLinkName;i++)
	{	
		length[i][0]	= atof(getNext(fin, "//").c_str());	
		length[i][1]	= atof(getNext(fin, "//").c_str());	
		length[i][2]	= atof(getNext(fin, "//").c_str());	
		mass[i]			= atof(getNext(fin, "//").c_str());	
	
		length[i] /= 100.0f;
		totalMass += mass[i];
	}

	for(int i=0;i<maxJointName;i++)
	{
		jointLink[i].first		= atoi(getNext(fin, "//").c_str());
		jointLink[i].second		= atoi(getNext(fin, "//").c_str());
		jointType[i]			= atoi(getNext(fin, "//").c_str());
	}		

	linkDirection[Trunk]	= None;
	linkDirection[Lwing1]	= Left;
	linkDirection[Lwing2]	= Left;
	linkDirection[Lwing3]	= Left;
	linkDirection[Rwing1]	= Right;
	linkDirection[Rwing2]	= Right;
	linkDirection[Rwing3]	= Right;

	util::fileCloseWrapper(fin);
}
void
Body::setBody()
{
	for(int i=0;i<maxLinkName;i++)		setLink((LinkName)i);

	for(int i=0;i<maxJointName;i++)		setJoint((JointName)i);		

#ifdef ROOT_FIX
	setGroundBody(Trunk);
#endif
}
void
Body::setGroundBody( LinkName id )
{
	phyEngine->setGround(id);
}
void
Body::setLink(LinkName id)
{
	if(id == Trunk)
	{
#ifdef ROOT_FIX
		phyEngine->setGeom(length[id], mass[id], vector3(0,0,0), quater(1,0,0,0));
#else
		phyEngine->setGeom(length[id], mass[id], initialPosition, EulerAngle2Quater(vector3(AngToRad(0),0,0)));
#endif
	}
	else
	{
		vector3 pos, pos1, pos2, pos1V, pos2V;
		quater rot, tRot;

		LinkName	prevID;
		vector3		prevP;
		quater		prevR;	

		switch(id)
		{
			case Lwing1:	prevID = Trunk;			break;
			case Lwing2:	prevID = Lwing1;		break;
			case Lwing3:	prevID = Lwing2;		break;
			case Rwing1:	prevID = Trunk;			break;
			case Rwing2:	prevID = Rwing1;		break;
			case Rwing3:	prevID = Rwing2;		break;
		}

		prevP	= phyEngine->getPositionBody( prevID );
		prevR	= phyEngine->getOrientationBody( prevID );

		pos1	= length[prevID]/2.0f;
		pos2	= length[id]/2.0f;
	
		float sign;
		int index;

		if(linkDirection[id] == Left)			{	sign = 1.0f;	index = Left;	}
		else if(linkDirection[id] == Right) 	{	sign = -1.0f;	index = Right;	}

		switch(id)
		{
		case Lwing1: case Rwing1:
			tRot = initS.shoulder[index];
			pos1V = vector3(sign, wingPosition[0], wingPosition[1]);
			pos2V = vector3(sign, 0, -1);
			break;
		case Lwing2: case Rwing2:
			//tRot = EulerAngle2Quater(vector3(0.0f, -(M_PI - initS.elbowBend[index])*sign, 0.0f)) * EulerAngle2Quater(vector3(-initS.elbowTwist[index], 0.0f, 0.0f));
			tRot = EulerAngle2Quater(vector3(0.0f, initS.elbowBend[index], 0.0f)) * EulerAngle2Quater(vector3(initS.elbowTwist[index], 0.0f, 0.0f));
			pos1V = vector3(sign, 0, 1);
			pos2V = vector3(sign, 0, -1);
			break;
		case Lwing3: case Rwing3:
			//tRot = EulerAngle2Quater(vector3(0.0f, -(initS.wristBend[index] - M_PI)*sign, 0.0f));
			tRot = EulerAngle2Quater(vector3(0.0f, initS.wristBend[index], 0.0f));
			pos1V = vector3(sign, 0, 1);
			pos2V = vector3(sign, 0, -1);
			break;
		}

		for(int i=0;i<3;i++)
		{
			pos1[i] *= pos1V[i];
			pos2[i] *= pos2V[i];
		}

		rot	= prevR * tRot;
		pos = prevP + rotate(prevR, pos1) + rotate(rot, pos2);

		phyEngine->setGeom(length[id], mass[id], pos, rot);
	}
}
void
Body::setJoint(JointName id)
{
	phyEngine->setJoint(jointType[id], jointLink[id].first, jointLink[id].second, (PhyEngine::Direction)linkDirection[ jointLink[id].second ]);
}
void				
Body::setTorque(LinkName id, vector3 torque)
{
	phyEngine->addTorqueBody( id, torque );
}
void
Body::setTorque(JointName id, float torque1, float torque2)
{
	phyEngine->setTorqueJoint( id, jointType[id], torque1, torque2 );
}
void
Body::setForce(LinkName id, vector3 force)
{
	phyEngine->addForceBody( id, force );
}
vector3				
Body::getJointPosition(JointName id)
{
	return phyEngine->getPositionJoint(id, jointType[id]);
}
quater
Body::getJointOrientation(JointName id)
{
	return phyEngine->getOrientationJoint(id);
}
void				
Body::getJointAxis(JointName id, vector3& axis1, vector3& axis2, vector3& axis3)
{
	phyEngine->getAxis(id, jointType[id], axis1, axis2, axis3);
}
void
Body::getJointAngle(JointName id, float* ang1, float* ang2)
{
	phyEngine->getAngle(id, jointType[id], ang1, ang2);
}
void
Body::getJointAngleRate(JointName id, float* ang1, float* ang2)
{
	phyEngine->getAngleRate(id, jointType[id], ang1, ang2);
}
vector3
Body::getJointAngularVelocity(JointName id)
{
	return phyEngine->getAngularVelocityJoint(id);
}
position	
Body::getLinkPosition(LinkName id)
{
	return phyEngine->getPositionBody(id);
}
quater				
Body::getLinkOrientation(LinkName id)
{
	return phyEngine->getOrientationBody(id);
}
vector3	
Body::getLinkTranslation(LinkName id)
{
	return phyEngine->getPositionBody(id) -  position(0,0,0);
}
transf
Body::getLinkTransformation(LinkName id)
{
	return transf(getLinkOrientation(id), getLinkTranslation(id));
}
vector3				
Body::getLinkAngularVelocity(LinkName id)
{
	return phyEngine->getAngularVelocityBody(id);
}
vector3				
Body::getLinkLinearVelocity(LinkName id)
{
	return phyEngine->getLinearVelocityBody(id);
}
void 
Body::setLinkPosition(LinkName id, position p)
{
	phyEngine->setPositionBody(id, p);
}
void 
Body::setLinkOrientation(LinkName id, quater q)
{
	phyEngine->setOrientationBody(id, q);
}
void 
Body::setLinkTransformation(LinkName id, transf t)
{
	phyEngine->setTransfBody(id, t);
}
void 
Body::setLinkAngularVelocity(LinkName id, vector3 v)
{
	phyEngine->setAngularVelocityBody(id, v);
}
void 
Body::setLinkLinearVelocity(LinkName id, vector3 v)
{
	phyEngine->setLinearVelocityBody(id, v);
}
BodyState
Body::getCurrentState()
{
	return currentS;
}
void
Body::incrementCount()
{
	count++;

	//cout << "count : " << count << endl;
}
void
Body::resetCount()
{
	count = 0;
}
void
Body::setWingbeat()
{
	// at the start and end of a current wingbeat
	
}
void
Body::popWingbeat()
{
}
void
Body::initializeDesiredState()
{
	//cout << controller->realWingbeat.avgWbParam << endl;

	count = controller->setCurrentFrame( count );
	controller->realWingbeat.setCurrentFromWbParam( controller->realWingbeat.avgWbParam );
	controller->realWingbeat.copyCurrentToPast();
	controller->realWingbeat.setBlendedCurrent();

	desiredS	= controller->getDesiredState( QI, desiredS );
	initS		= desiredS;
	currentS	= desiredS;
	prevS		= desiredS;
	desiredS_modify = desiredS;

	//beforeTrunk = phyEngine->getRigidStateBody(Trunk);
	//currentTrunk = phyEngine->getRigidStateBody(Trunk);
}
void
Body::updateDesiredState()
{
	int &mode = controller->simulStep;
	static transf randT, tempT, initT, curT, T;

	//cout << "q0" << endl;
	count = controller->setCurrentFrame( count );
	//cout << "q1" << endl;

	if(count==0)
	{
		num_wb++;
		//cout << num_wb << "     " << mode << "        " << endl;
		//cout << controller->glidePoint[0] << "    " << controller->glidePoint[1] << "     " << controller->glideParam << endl;

		

		cout << "----------------------------" << endl;
		T =  getLinkTransformation(Trunk);
		//cout << T.getTranslation() << endl;
		//cout << T.getRotation() << endl;
		cout << getLinkLinearVelocity(Trunk) << " || ";
		cout << getLinkLinearVelocity(Trunk).length() << " m/s" << endl;
		cout << "----------------------------" << endl;

		controller->do_None();
		/*if( BETWEEN(num_wb, controller->glidePoint[0], controller->glidePoint[1]) )
		{
			controller->genOneWingbeat_Glide( controller->glideParam );
		}
		else controller->setNewWbParam( controller->realWingbeat.avgWbParam );*/
	}

	desiredS = controller->getDesiredState( getLinkOrientation( Trunk ), desiredS );
	desiredS_modify = controller->modifyDesiredState( desiredS );

#ifndef ROOT_FIX
	if(init)
	{
		for(int j=0;j<maxLinkName;j++)	setLinkLinearVelocity( (LinkName)j, initialVelocity );

		init = false;
	}
#endif

}
void
Body::updateCurrentState()
{
	float dT = sysState->TIME_STEP;
	float temp;

	// cur to prev
	prevS = currentS;

	// s

	currentS.trunkPosition		= getLinkPosition( Trunk );
	currentS.trunkOrientation	= getLinkOrientation( Trunk );
	
	currentS.shoulder[0]		= getLinkOrientation( Lwing1 );
	currentS.shoulder[1]		= getLinkOrientation( Rwing1 );
	
	getJointAngle( ElbowL, &currentS.elbowBend[0], &currentS.elbowTwist[0] );
	getJointAngle( ElbowR, &currentS.elbowBend[1], &currentS.elbowTwist[1] );
	getJointAngle( WristL, &currentS.wristBend[0], &temp );
	getJointAngle( WristR, &currentS.wristBend[1], &temp );

	// ds/dt

	currentS.dTrunkPosition		= getLinkLinearVelocity( Trunk );
	currentS.dTrunkOrientation	= getLinkAngularVelocity( Trunk );

	currentS.dShoulder[0]		= getLinkAngularVelocity( Lwing1 );
	currentS.dShoulder[1]		= getLinkAngularVelocity( Rwing1 );

	getJointAngleRate( ElbowL, &currentS.dElbowBend[0], &currentS.dElbowTwist[0] );
	getJointAngleRate( ElbowR, &currentS.dElbowBend[1], &currentS.dElbowTwist[1] );
	getJointAngleRate( WristL, &currentS.dWristBend[0], &temp );
	getJointAngleRate( WristR, &currentS.dWristBend[1], &temp );

	// dds/ddt

	currentS.ddTrunkPosition	= (currentS.dTrunkPosition - prevS.dTrunkPosition)			/ dT;
	currentS.ddTrunkOrientation	= (currentS.dTrunkOrientation - prevS.dTrunkOrientation)	/ dT;

	currentS.ddShoulder[0]		= (currentS.dShoulder[0] - prevS.dShoulder[0])				/ dT;
	currentS.ddShoulder[1]		= (currentS.dShoulder[1] - prevS.dShoulder[1])				/ dT;

	currentS.ddElbowTwist[0]	= (currentS.dElbowTwist[0] - prevS.dElbowTwist[0])			/ dT;
	currentS.ddElbowTwist[1]	= (currentS.dElbowTwist[1] - prevS.dElbowTwist[1])			/ dT;
	currentS.ddElbowBend[0]		= (currentS.dElbowBend[0] - prevS.dElbowBend[0])			/ dT;
	currentS.ddElbowBend[1]		= (currentS.dElbowBend[1] - prevS.dElbowBend[1])			/ dT;

	for(int i=0;i<2;i++)
	{
		currentS.elbowTwist[i]	*= -1.0f;
		currentS.wristBend[i]	*= -1.0f;

		currentS.elbowBend[i]	+= initS.elbowBend[i];
		currentS.elbowTwist[i]	+= initS.elbowTwist[i];
		currentS.wristBend[i]	+= initS.wristBend[i];
	}

	// push current state to trajectory
	if((int)trajectory.size() > 5000) trajectory.clear();
	if(count==0)
		trajectory.push_back( SE3(getLinkOrientation(Trunk),getLinkPosition(Trunk)) );

	////////////////////
	/*WRITE_VECTOR3(cout, currentS.elbowBend[0]);
	WRITE_VECTOR3(cout, currentS.elbowBend[1]);
	WRITE_VECTOR3(cout, currentS.elbowTwist[0]);
	WRITE_VECTOR3(cout, currentS.elbowTwist[1]);*/

	/*cout << "elbow bend L: " << RadToAng(desiredS.elbowBend[0]) << "," << RadToAng(currentS.elbowBend[0]) << " (" << RadToAng(initS.elbowBend[0]) << ")" << endl;
	cout << "elbow bend R: " << RadToAng(desiredS.elbowBend[1]) << "," << RadToAng(currentS.elbowBend[1]) << " (" << RadToAng(initS.elbowBend[1]) << ")" << endl;
	cout << "elbow twist L: " << RadToAng(desiredS.elbowTwist[0]) << "," << RadToAng(currentS.elbowTwist[0]) << " (" << RadToAng(initS.elbowTwist[0]) << ")" << endl;
	cout << "elbow twist R: " << RadToAng(desiredS.elbowTwist[1]) << "," << RadToAng(currentS.elbowTwist[1]) << " (" << RadToAng(initS.elbowTwist[1]) << ")" << endl;
	cout << "wrist bend L: " << RadToAng(desiredS.wristBend[0]) << "," << RadToAng(currentS.wristBend[0]) << " (" << RadToAng((currentS.wristBend[0]-initS.wristBend[0])) << ")" << endl;
	cout << "wrist bend R: " << RadToAng(desiredS.wristBend[1]) << "," << RadToAng(currentS.wristBend[1]) << " (" << RadToAng((currentS.wristBend[1]-initS.wristBend[1]))  << ")" << endl;
	cout << "~~~~~~~~~~~~~~" << endl;*/
}
void
Body::calcPDcontrolTorque()
{
	if(count<=1) return;

	for(int i=0;i<2;i++)
	{
		vector3 temp1	= DIFFERENCE2( desiredS_modify.shoulder[i], currentS.shoulder[i] );
		vector3 temp2	= desiredS_modify.dShoulder[i] - currentS.dShoulder[i];
				
		torque.shoulder[i]		= coeff->linkCoeff->pdGain[0] * temp1														+ coeff->linkCoeff->pdGain[1] * temp2;
		torque.elbowBend[i]		= coeff->linkCoeff->pdGain[2] * ( desiredS_modify.elbowBend[i] - currentS.elbowBend[i] )	+ coeff->linkCoeff->pdGain[3] * ( desiredS_modify.dElbowBend[i] - currentS.dElbowBend[i] );
		torque.elbowTwist[i]	= coeff->linkCoeff->pdGain[4] * ( desiredS_modify.elbowTwist[i] - currentS.elbowTwist[i] )	+ coeff->linkCoeff->pdGain[5] * ( desiredS_modify.dElbowTwist[i] - currentS.dElbowTwist[i] );
		torque.wristBend[i]		= coeff->linkCoeff->pdGain[6] * ( desiredS_modify.wristBend[i] - currentS.wristBend[i] )	+ coeff->linkCoeff->pdGain[7] * ( desiredS_modify.dWristBend[i] - currentS.dWristBend[i] );

#ifdef ROOT_FIX
		torque.shoulder[i]		*= 1.0f;
		torque.elbowBend[i]		*= 1.0f;
		torque.elbowTwist[i]	*= 1.0f;
		torque.wristBend[i]		*= 1.0f;
#endif

	}

	if(false)
	{
		cout << "-------------------------------------------" << endl;
		WRITE_ARRAY3(cout, torque.shoulder[0]); WRITE_ARRAY3(cout, torque.shoulder[1]);
		cout << torque.elbowBend[0] << endl; cout << torque.elbowBend[1] << endl;
		cout << torque.elbowTwist[0] << endl; cout << torque.elbowTwist[1] << endl;
		cout << torque.wristBend[0] << endl; cout << torque.wristBend[1] << endl;
		cout << "-------------------------------------------" << endl;
	}
}
void
Body::applyPDcontrolTorque()
{
	setTorque( Lwing1, torque.shoulder[0] );		setTorque( Trunk, -torque.shoulder[0] );
	setTorque( Rwing1, torque.shoulder[1] );		setTorque( Trunk, -torque.shoulder[1] );
	setTorque( ElbowL, torque.elbowBend[0], -torque.elbowTwist[0] );
	setTorque( ElbowR, torque.elbowBend[1], -torque.elbowTwist[1] );
	setTorque( WristL, -torque.wristBend[0], 0);
	setTorque( WristR, -torque.wristBend[1], 0);
}
void
Body::saveState(const int sID)
{
	count_save[sID]				= count;
	num_wb_save[sID]			= num_wb;

	initS_save[sID]				= initS;
	prevS_save[sID]				= prevS;
	currentS_save[sID]			= currentS;
	desiredS_save[sID]			= desiredS;
	desiredS_modify_save[sID]	= desiredS_modify;

	torque_save[sID]			= torque;

	phyEngine->saveState(sID);
}
void
Body::restoreState(const int sID)
{
	count			= count_save[sID];
	num_wb			= num_wb_save[sID];

	initS			= initS_save[sID];
	prevS			= prevS_save[sID];
	currentS		= currentS_save[sID];
	desiredS		= desiredS_save[sID];
	desiredS_modify = desiredS_modify_save[sID];

	torque			= torque_save[sID];

	phyEngine->restoreState(sID);
}
RenderState_body 
Body::getRenderState()
{
	RenderState_body rs_b;

	rs_b.trunk = getLinkTransformation( Trunk );
	rs_b.upperArm[0] = getLinkTransformation( Lwing1 );
	rs_b.upperArm[1] = getLinkTransformation( Rwing1 );
	rs_b.lowerArm[0] = getLinkTransformation( Lwing2 );
	rs_b.lowerArm[1] = getLinkTransformation( Rwing2 );
	rs_b.hand[0] = getLinkTransformation( Lwing3 );
	rs_b.hand[1] = getLinkTransformation( Rwing3 );

	return rs_b;
}
void
Body::putBirdAt(transf T, float v_linear_scale, float v_angular_scale)
{
	for(int i=0; i<maxLinkName; i++)
	{
		transf t = getLinkTransformation( (LinkName)i );
		setLinkTransformation( (LinkName)i, t * T );

		vector3 v_linear	= getLinkLinearVelocity( (LinkName)i );
		vector3 v_angular	= getLinkAngularVelocity( (LinkName)i );

		if(i==Trunk)
		{
			v_linear *= v_linear_scale;
			v_angular *= v_angular_scale;
		}

		setLinkLinearVelocity( (LinkName)i, v_linear * T );
		setLinkAngularVelocity( (LinkName)i, v_angular * T );
	}

	desiredS = controller->getDesiredState( getLinkOrientation( Trunk ), desiredS );
	desiredS_modify = controller->modifyDesiredState( desiredS );
}
void
Body::drawJoint()
{ 
	glDisable(GL_DEPTH_TEST);

	glPushMatrix();
	for(int i=0;i<maxJointName;i++)
	{
		//if( (JointName)i == JointName::WristL || (JointName)i == JointName::WristR ) continue;

		//if(i%2==0)	glSetMaterial(239/255.0f * 0.6f, 65/255.0f * 0.6f, 86/255.0f * 0.6f);
		//else		glSetMaterial(255/255.0f * 0.6f, 215/255.0f * 0.6f, 105/255.0f * 0.6f);
		glSetMaterial(0.7, 0.0, 0.0, 0.7);

		vector3 pos = getJointPosition( (JointName)i );

		drawSphere(pos[0],pos[1],pos[2],0.004f);

		// draw axis
		if(false)	
		{
			vector3 axis1, axis2, axis3;
			float leng = 0.05f;

			getJointAxis( (JointName)i, axis1, axis2, axis3);

			axis1 = pos + axis1*leng;
			axis2 = pos + axis2*leng;
			axis3 = pos + axis3*leng;

			glDisable(GL_LIGHTING);
			glColor3f(1,0,0);	glBegin( GL_LINES );	glVertex3f( pos[0], pos[1], pos[2] );	glVertex3f( axis1[0], axis1[1], axis1[2] );		glEnd();
			glColor3f(0,1,0);	glBegin( GL_LINES );	glVertex3f( pos[0], pos[1], pos[2] );	glVertex3f( axis2[0], axis2[1], axis2[2] );		glEnd();
			glColor3f(0,0,1);	glBegin( GL_LINES );	glVertex3f( pos[0], pos[1], pos[2] );	glVertex3f( axis3[0], axis3[1], axis3[2] );		glEnd();
			glEnable(GL_LIGHTING);
		}
	}

	// tail joint
	glPushMatrix();
	glTransform(getLinkTransformation( Trunk ));
		glSetMaterial(0.7, 0.0, 0.0, 0.7);
		drawSphere(0,0,-0.5*length[Trunk][2],0.004f);
	glPopMatrix();

	glPopMatrix();

	glEnable(GL_DEPTH_TEST);
}
void
Body::drawEllipseLink()
{
	//glSetMaterial(239/255.0f, 65/255.0f, 86/255.0f, 0.8f);
	glSetMaterial(0.7, 0.7, 0.7, 0.7);
	simpleEllipse(getJointPosition(ShoulderL),getJointPosition(ElbowL));
	simpleEllipse(getJointPosition(ElbowL),getJointPosition(WristL));

	//glSetMaterial(255/255.0f, 215/255.0f, 105/255.0f, 0.8f);
	glSetMaterial(0.7, 0.7, 0.7, 0.7);
	simpleEllipse(getJointPosition(ShoulderR),getJointPosition(ElbowR));
	simpleEllipse(getJointPosition(ElbowR),getJointPosition(WristR));	
}
void
Body::drawLink()
{
	switch(sysState->drawBody)
	{
	case SystemState::BASIC:
		for(int i=0;i<maxLinkName;i++)	
		{
			if(i==0)		glSetMaterial(105/255.0f * 0.6f, 205/255.0f * 0.6f, 86/255.0f * 0.6f, 0.7f);
			else if(i<4)	glSetMaterial(239/255.0f * 0.6f, 65/255.0f * 0.6f, 86/255.0f * 0.6f, 0.7f);
			else			glSetMaterial(255/255.0f * 0.6f, 215/255.0f * 0.6f, 105/255.0f * 0.6f, 0.7f);

			glPushMatrix();
				glTransform(getLinkTransformation( (LinkName)i ));
				glScalef(length[i][0],length[i][1],length[i][2]);
				glutSolidCube(1.0f);
			glPopMatrix();
		}
		drawJoint();
		drawEllipseLink();
		drawDesiredBody();
		break;
	case SystemState::MESH:
		{
			glPushMatrix();
				glTransform(getLinkTransformation(Trunk));
				/*glScalef(0.85f,0.6f,1.f);
				glScalef(0.8f,0.8f,0.8f);*/
				glScalef(0.6f,0.5f,1.3f);
				glScalef(0.8f,0.8f,0.8f);
				glRotatef(-25, 1, 0, 0);
				glTranslatef(0, 0.015, 0);
				meshModel->drawModel();
			glPopMatrix();
		}
		drawJoint();
		drawEllipseLink();
		break;
	case SystemState::NONE_B:
		break;
	}
}
void
Body::drawBody()
{
	drawLink();

	drawTrajectory();
}
void
Body::drawDesiredBody()
{
	BodyState &desBody = desiredS_modify;

	vector3 shoulder, elbow, wrist, hand, ss, ee, ww, temp;
	quater rotQ;
	float sign;
	
	vector3 color = autoColor(3)*0.9f;
	glSetMaterial(color[0], color[1], color[2], 0.8f);

	ss = vector3( length[Lwing1][0],0,0 );
	ee = vector3( length[Lwing2][0],0,0 );
	ww = vector3( length[Lwing3][0],0,0 );

	shoulder = getJointPosition( ShoulderL );

	rotQ = desiredS_modify.shoulder[0];
	elbow = rotate(rotQ, ss ) + shoulder;

	temp = vector3(desiredS_modify.elbowTwist[0], desiredS_modify.elbowBend[0] ,0);
	rotQ = rotQ * EulerAngle2Quater(temp);
	wrist = rotate(rotQ, ee ) + elbow;

	temp = vector3(0, desiredS_modify.wristBend[0],0);
	rotQ = rotQ * EulerAngle2Quater(temp);
	hand = rotate(rotQ, ww ) + wrist;

	simpleEllipse(shoulder, elbow);		drawSphere(elbow[0],	elbow[1],	elbow[2], 0.005f);
	simpleEllipse(elbow, wrist);		drawSphere(wrist[0],	wrist[1],	wrist[2], 0.005f);
	simpleEllipse(wrist, hand);			drawSphere(hand[0],	hand[1],	hand[2], 0.005f);

	ss = vector3( -length[Rwing1][0],0,0 );
	ee = vector3( -length[Rwing2][0],0,0 );
	ww = vector3( -length[Rwing3][0],0,0 );

	shoulder = getJointPosition( ShoulderR );

	rotQ = desiredS_modify.shoulder[1];
	elbow = rotate(rotQ, ss) + shoulder;

	temp = vector3(desiredS_modify.elbowTwist[1], desiredS_modify.elbowBend[1] ,0);
	rotQ = rotQ * EulerAngle2Quater(temp);
	wrist = rotate(rotQ, ee) + elbow;

	temp = vector3(0, desiredS_modify.wristBend[1],0);
	rotQ = rotQ * EulerAngle2Quater(temp);
	hand = rotate(rotQ, ww ) + wrist;

	simpleEllipse(shoulder, elbow);		drawSphere(elbow[0],	elbow[1],	elbow[2], 0.005f);
	simpleEllipse(elbow, wrist);		drawSphere(wrist[0],	wrist[1],	wrist[2], 0.005f);
	simpleEllipse(wrist, hand);			drawSphere(hand[0],	hand[1],	hand[2], 0.005f);

	// draw axis
	if(false)	
	{
		for(int i=0; i<2; i++)
		{
			vector3 axis1, axis2, axis3;
			float leng = 0.05f;

			vector3 pos = getJointPosition( (i==0)?ShoulderL:ShoulderR );
			quater rot = currentS.shoulder[i];

			axis1 = rotate( rot, vector3(1,0,0) );
			axis2 = rotate( rot, vector3(0,1,0) );
			axis3 = rotate( rot, vector3(0,0,1) );

			axis1 = pos + axis1*leng;
			axis2 = pos + axis2*leng;
			axis3 = pos + axis3*leng;

			glDisable(GL_LIGHTING);
			glColor3f(1,0,0);	glBegin( GL_LINES );	glVertex3f( pos[0], pos[1], pos[2] );	glVertex3f( axis1[0], axis1[1], axis1[2] );		glEnd();
			glColor3f(0,1,0);	glBegin( GL_LINES );	glVertex3f( pos[0], pos[1], pos[2] );	glVertex3f( axis2[0], axis2[1], axis2[2] );		glEnd();
			glColor3f(0,0,1);	glBegin( GL_LINES );	glVertex3f( pos[0], pos[1], pos[2] );	glVertex3f( axis3[0], axis3[1], axis3[2] );		glEnd();

			glEnable(GL_LIGHTING);
		}

		for(int i=0; i<2; i++)
		{
			vector3 axis1, axis2, axis3;
			float leng = 0.05f;

			vector3 pos = getJointPosition( (i==0)?ShoulderL:ShoulderR );
			quater rot = desiredS_modify.shoulder[i];

			axis1 = rotate( rot, vector3(1,0,0) );
			axis2 = rotate( rot, vector3(0,1,0) );
			axis3 = rotate( rot, vector3(0,0,1) );

			axis1 = pos + axis1*leng;
			axis2 = pos + axis2*leng;
			axis3 = pos + axis3*leng;

			glDisable(GL_LIGHTING);
			glColor3f(0.5,0,0);	glBegin( GL_LINES );	glVertex3f( pos[0], pos[1], pos[2] );	glVertex3f( axis1[0], axis1[1], axis1[2] );		glEnd();
			glColor3f(0,0.5,0);	glBegin( GL_LINES );	glVertex3f( pos[0], pos[1], pos[2] );	glVertex3f( axis2[0], axis2[1], axis2[2] );		glEnd();
			glColor3f(0,0,0.5);	glBegin( GL_LINES );	glVertex3f( pos[0], pos[1], pos[2] );	glVertex3f( axis3[0], axis3[1], axis3[2] );		glEnd();

			vector3 t = torque.shoulder[i]; t = normalize(t);
			glColor3f(1,1,0.5);	glBegin( GL_LINES );	glVertex3f( pos[0], pos[1], pos[2] );	glVertex3f( t[0], t[1], t[2] );		glEnd();

			glEnable(GL_LIGHTING);
		}
	}
}
void
Body::drawTrajectory()
{
	vector3 color = vector3(0.5, 0.5, 0.5);
	int length;

	glLineWidth(3.0);
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);

	switch(sysState->drawTrajectory)
	{
	case SystemState::FIXED:

		glColor3f(color[0], color[1], color[2]);
		glBegin(GL_LINE_STRIP);

			for(int i=0;i<(int)trajectory.size();i++) 
				glVertex3f(trajectory[i].pos[0],trajectory[i].pos[1],trajectory[i].pos[2]);

		glEnd();
		break;

	case SystemState::VANISNING:

		length = 1000;

		glBegin(GL_LINE_STRIP);
			if((int)trajectory.size() > length)
			{
				for(int i=0;i<length;i++)
				{
					position p = trajectory[(int)trajectory.size()-1-i].pos;
					glColor4f(color[0], color[1], color[2], 1.0-i/(float)length);
					glVertex3f(p[0], p[1],p[2]);
				}
			}
			else	
			{
				for(int i=0;i<(int)trajectory.size();i++) 
				{
					glColor3f(color[0], color[1], color[2]);
					glVertex3f(trajectory[i].pos[0],trajectory[i].pos[1],trajectory[i].pos[2]);
				}
			}
		glEnd();
		break;
	case SystemState::NONE_T:
		break;
	}


	glEnable(GL_LIGHTING);
}