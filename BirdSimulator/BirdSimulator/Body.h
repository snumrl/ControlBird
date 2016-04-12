#pragma once

#include <iostream>
#include <vector>
#include <MATHCLASS/mathclass.h>
#include "util.h"
#include "PhyEngine.h"
#include "SystemState.h"

using namespace std;

struct BodyState;
struct Torque;
class Body;

struct RenderState_body
{
	transf				trunk;
	transf				upperArm[2];
	transf				lowerArm[2];
	transf				hand[2];

	RenderState_body& operator=( const RenderState_body& _b )
	{
		trunk		= _b.trunk;
		upperArm[0] = _b.upperArm[0];
		upperArm[1] = _b.upperArm[1];
		lowerArm[0]	= _b.lowerArm[0];
		lowerArm[1]	= _b.lowerArm[1];
		hand[0]		= _b.hand[0];
		hand[1]		= _b.hand[1];
	}
};

struct BodyState
{
	position			trunkPosition;
	quater				trunkOrientation;

	quater				shoulder[2];
	float				elbowBend[2];
	float				elbowTwist[2];
	float				wristBend[2];

	vector3				dTrunkPosition;
	vector3				dTrunkOrientation;

	vector3				dShoulder[2];
	float				dElbowBend[2];
	float				dElbowTwist[2];
	float				dWristBend[2];

	vector3				ddTrunkPosition;
	vector3				ddTrunkOrientation;

	vector3				ddShoulder[2];
	float				ddElbowBend[2];
	float				ddElbowTwist[2];
	float				ddWristBend[2];

	// body

	BodyState()
	{
		trunkPosition.setZero();
		trunkOrientation = identity_transf.getRotation();
		
		for(int i=0;i<2;i++)
		{
			float		sign = 1.0f;
			if(i==0)	sign = -1.0f;

			shoulder[i]		= EulerAngle2Quater( vector3(0.0f, AngToRad(0.0f)*sign, 0.0f) );
			elbowBend[i]	= AngToRad(0.0f);
			elbowTwist[i]	= AngToRad(0.0f);
			wristBend[i]	= AngToRad(0.0f);

			dShoulder[i]	= vector3();
			dElbowBend[i]	= 0.0f;
			dElbowTwist[i]	= 0.0f;
			dWristBend[i]	= 0.0f;

			ddShoulder[i]	= vector3();
			ddElbowBend[i]	= 0.0f;
			ddElbowTwist[i]	= 0.0f;
			ddWristBend[i]	= 0.0f;
		}
	}

	BodyState& operator=( const BodyState& _b )
	{	
		trunkPosition		= _b.trunkPosition;
		trunkOrientation	= _b.trunkOrientation;

		dTrunkPosition		= _b.dTrunkPosition;
		dTrunkOrientation	= _b.dTrunkOrientation;

		ddTrunkPosition		= _b.ddTrunkPosition;
		ddTrunkOrientation	= _b.ddTrunkOrientation;

		for(int i=0;i<2;i++)
		{
			shoulder[i]		= _b.shoulder[i];
			elbowBend[i]	= _b.elbowBend[i];
			elbowTwist[i]	= _b.elbowTwist[i];
			wristBend[i]	= _b.wristBend[i];

			dShoulder[i]	= _b.dShoulder[i];
			dElbowBend[i]	= _b.dElbowBend[i];
			dElbowTwist[i]	= _b.dElbowTwist[i];
			dWristBend[i]	= _b.dWristBend[i];

			ddShoulder[i]	= _b.ddShoulder[i];
			ddElbowBend[i]	= _b.ddElbowBend[i];
			ddElbowTwist[i]	= _b.ddElbowTwist[i];
			ddWristBend[i]	= _b.ddWristBend[i];
		}

		return *this;
	}

	friend ostream& operator<<( ostream& os, const BodyState& _bs )
	{
		os << _bs.trunkPosition;
		os << _bs.trunkOrientation;

		os << _bs.shoulder[0];
		os << _bs.shoulder[1];

		os << _bs.elbowBend[0];
		os << _bs.elbowBend[1];
		os << _bs.elbowTwist[0];
		os << _bs.elbowTwist[1];
		os << _bs.wristBend[0];
		os << _bs.wristBend[1];

		return os;
	}

	void toRigidState( RigidState& rs )
	{
		rs.pos = trunkPosition;
		rs.ori = trunkOrientation;
		rs.linVel = dTrunkPosition;
		rs.angVel = dTrunkOrientation;
	}
/*
	BodyState operator-( const BodyState& b )
	{
		BodyState c;

		c.trunkPosition			= this->trunkPosition - b.trunkPosition;
		c.trunkOrientation		= this->trunkOrientation - b.trunkOrientation;

		c.dTrunkPosition		= this->dTrunkPosition - b.dTrunkPosition;
		c.dTrunkOrientation		= this->dTrunkOrientation - b.dTrunkOrientation;
		
		for(int i=0;i<2;i++)
		{
			c.shoulder[i]			= this->shoulder[i] * b.shoulder[i].inverse();
			c.elbowBend[i]			= this->elbowBend[i] - b.elbowBend[i];
			c.elbowTwist[i]			= this->elbowTwist[i] - b.elbowTwist[i];
			c.wristBend[i]			= this->wristBend[i] - b.wristBend[i];

			c.dShoulder[i]			= this->dShoulder[i] - b.dShoulder[i];
			c.dElbowBend[i]			= this->dElbowBend[i] - b.dElbowBend[i];
			c.dElbowTwist[i]		= this->dElbowTwist[i] - b.dElbowTwist[i];
			c.dWristBend[i]			= this->dWristBend[i] - b.dWristBend[i];
		}

		return c;
	}*/
};

struct Torque
{
	vector3				shoulder[2];
	float				elbowBend[2];
	float				elbowTwist[2];
	float				wristBend[2];

	Torque()
	{
		for(int i=0;i<2;i++)
		{
			shoulder[i]		= vector3();
			elbowBend[i]	= 0.0f;
			elbowTwist[i]	= 0.0f;
			wristBend[i]	= 0.0f;
		}
	}

	Torque& operator=(const Torque& _t)
	{
		for(int i=0; i<2; i++)
		{
			shoulder[i]		= _t.shoulder[i];
			elbowBend[i]	= _t.elbowBend[i];
			elbowTwist[i]	= _t.elbowTwist[i];
			wristBend[i]	= _t.wristBend[i];
		}

		return *this;
	}
};

class Body
{
public:
	bool				init;

	enum				LinkName		{ Trunk=0, Lwing1, Lwing2, Lwing3, Rwing1, Rwing2, Rwing3, maxLinkName };
	enum				JointName		{ ShoulderL=0, ShoulderR, ElbowL, ElbowR, WristL, WristR, maxJointName };
	enum				JointType		{ Ball = 0, Universal, Hinge };
	enum				Direction		{ None = -1, Left, Right };

	vector3*			length;
	double*				mass;
	double				totalMass;
	int*				jointType;
	std::pair<int,int>*	jointLink;
	Direction*			linkDirection;

	int					numLink;
	int					numJoint;
	int					totalJointDOF;

	vector3				initialPosition;
	vector3				initialVelocity;
	float				wingPosition[2];		

	int					count;
	int					count_wb;
	int					num_wb;

	BodyState			initS;
	BodyState			prevS;
	BodyState			currentS;
	BodyState			desiredS;
	BodyState			desiredS_modify;

	Torque				torque;
	//float				pdGain[8];
	//float				springCoeff[4];

	// for trainig data
	ofstream			f_TrainData;
	ofstream			f_ControlAnalysis;

	// for rendering
	vector<RenderState_body> renderState;

	// 초기화 된 후에 3개의 이전 정보가 있어야함
	vector<SE3>			trajectory;

	RigidState			beforeTrunk;
	RigidState			currentTrunk;

	// for save
	int					count_save[max_save];
	int					num_wb_save[max_save];

	BodyState			initS_save[max_save];
	BodyState			prevS_save[max_save];
	BodyState			currentS_save[max_save];
	BodyState			desiredS_save[max_save];
	BodyState			desiredS_modify_save[max_save];

	Torque				torque_save[max_save];

	Body()
	{
		init		= true;

		numLink		= maxLinkName;
		numJoint	= maxJointName;

		count		= 0;
		num_wb		= 0;
		
		length		= new vector3[numLink];
		mass		= new double[numLink];
		
		jointType	= new int[numJoint];
		jointLink	= new std::pair<int,int>[numJoint];

		linkDirection	= new Direction[numLink];

		loadBodyInformation();

		// initialize desired motion to be tracked
		initializeDesiredState();

		// create simulation body and joint
		// when boides are created, their orientations is set
		// as orientation of a first disired motion
		setBody();

		/*springCoeff[0] = 0.0128f/2;
		springCoeff[1] = 0.000098f;
		springCoeff[2] = 0.0000128f;
		springCoeff[3] = 0.0000098f;*/

		for(int i=2; i>0; i--)
		{
			SE3 se3;
			se3.pos = position(initialPosition[0],initialPosition[1],initialPosition[2]-i);
			se3.frame = quater(1,0,0,0);

			trajectory.push_back(se3);
		}

		totalJointDOF = 0;
		for(int i=0; i<numJoint; i++)
		{
			switch(jointType[i])
			{
			case Ball:		totalJointDOF += 3; break;
			case Universal: totalJointDOF += 2; break;
			case Hinge:		totalJointDOF += 1; break;
			}
		}

		updateCurrentState();
	};

	void				saveState(const int sID);
	void				restoreState(const int sID);

	bool				isFirstSimulation();
	void				loadBodyInformation();			
	void				setBody();
	void				setGroundBody(LinkName id);
	void				setLink(LinkName id);
	void				setJoint(JointName id);
	void				setTorque(LinkName id, vector3 torque);
	void				setTorque(JointName id, float torque1, float torque2=0.0f);
	void				setForce(LinkName id, vector3 force);

	/* link(body) functions */

	vector3				getJointPosition(JointName id);

	quater				getJointOrientation(JointName id);

	void				getJointAxis(JointName id, vector3& axis1, vector3& axis2, vector3& axis3);

	void				getJointAngle(JointName id, float* ang1, float* ang2);

	void				getJointAngleRate(JointName id, float* ang1, float* ang2);

	vector3				getJointAngularVelocity(JointName id);

	/* joint functions */

	transf				getLinkTransformation(LinkName id);

	position			getLinkPosition(LinkName id);

	quater				getLinkOrientation(LinkName id);

	vector3				getLinkTranslation(LinkName id);

	vector3				getLinkAngularVelocity(LinkName id);

	vector3				getLinkLinearVelocity(LinkName id);

	void				setLinkTransformation(LinkName id, transf t);

	void				setLinkPosition(LinkName id, position p);

	void				setLinkOrientation(LinkName id, quater q);

	void				setLinkAngularVelocity(LinkName id, vector3 v);

	void				setLinkLinearVelocity(LinkName id, vector3 v);



	BodyState			getCurrentState();

	void				incrementCount();
	void				resetCount();

	void				setWingbeat();
	void				popWingbeat();

	void				initializeDesiredState();
	void				updateDesiredState();
	void				updateCurrentState();

	void				calcPDcontrolTorque();
	void				applyPDcontrolTorque();

	RenderState_body	getRenderState();

	// this T is global transformation 
	// from trunk trasforamtion to desired transformation
	void				putBirdAt(transf T, float v_linear_scale=1.0f, float v_angular_scale=1.0f);

	// for trainig data
	void				begin_TrainData();
	void				end_TrainData();
	void				printBodyState_TrainData();
		
	void				drawJoint();
	void				drawLink();
	void				drawEllipseLink();
	void				drawMeshTrunk();
	void				drawBody();
	void				drawDesiredBody();
	void				drawTrajectory();
};