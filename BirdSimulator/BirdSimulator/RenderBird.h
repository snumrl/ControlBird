#pragma once

#include "Bird.h"
#include "Controller.h"
#include "util.h"
#include <MATHCLASS/mathclass.h>
#include <vector>
#include <fstream>

extern Bird*			simBird;

struct EachJoint
{
	vector3		init;
	quater		rot;
	
	vector3		gPos;							// for rendering

	std::vector<vector3>	pos;
	
	EachJoint::EachJoint()
	{
		initialize();
	};

	void initialize()
	{
		init = vector3(0,0,0);
		gPos = init;

		initializeOri();
	};

	void initializeOri()
	{
		rot = QI;
	};

	void setShoulderRotation(quater qq)
	{
		rot = rot*qq;
	};

	void setElbowAngle(float bend, float twist)
	{
		quater q1 = EulerAngle2Quater(vector3(0,bend,0));
		quater q2 = EulerAngle2Quater(vector3(twist,0,0));

		rot = rot*q1*q2;
	};

	void setWristAngle(float bend)
	{
		quater q1 = EulerAngle2Quater(vector3(0,bend,0));

		rot = rot*q1;
	};

	void setFeatherRotation(quater qq)
	{
		rot = qq;
	}
	
	void updateStateBack(vector3 v, quater q)			
	{
		rot = rot*q;
		gPos = rotate(rot, init) + v;
	};
	
	void updateStateFront(vector3 v, quater q)			
	{
		rot = q*rot;
		gPos = rotate(q, init) + v;
	};

	void drawInit()
	{
		drawSphere(init[0], init[1], init[2], 0.001f);
	};

	void drawGpos()
	{
		drawSphere(gPos[0], gPos[1], gPos[2], 0.001f);
	};

	void drawSurface()
	{
		if(pos.size()==3)
		{
			float ang = 2.0f*acos(rot.real());
			vector3 vec = normalize( rot.imaginaries() );

			glNormal3f(0,1,0);
			glPushMatrix();
			glTranslatef(gPos[0],gPos[1],gPos[2]);
			glRotatef(RadToAng(ang), vec[0], vec[1], vec[2]);
			drawTraiangle(pos[0], pos[1], pos[2]);
			glPopMatrix();
		}
		else if(pos.size()==4)
		{
			float ang = 2.0f*acos(rot.real());
			vector3 vec = normalize( rot.imaginaries() );

			glNormal3f(0,1,0);
			glPushMatrix();
			glTranslatef(gPos[0],gPos[1],gPos[2]);
			glRotatef(RadToAng(ang), vec[0], vec[1], vec[2]);
			drawQuad(pos[0], pos[1], pos[2], pos[3]);
			glPopMatrix();
		}
	};

};

struct Joints
{
	std::vector<EachJoint>	joints;

	void initialize()
	{
		for(int i=0;i<joints.size();i++)
			joints[i].initialize();
	}

	void initializeOri()
	{
		for(int i=0;i<joints.size();i++)
			joints[i].initializeOri();
	}
};

struct FeatherJoints
{
	std::vector<Joints>		fjoint;

	void initialize()
	{
		for(int i=0;i<fjoint.size();i++)
			fjoint[i].initialize();
	}

	void initializeOri()
	{
		for(int i=0;i<fjoint.size();i++)
			fjoint[i].initializeOri();
	}
};

class JointState
{
public:
	EachJoint	root;
	EachJoint	shoulder[2];
	EachJoint	elbow[2];
	EachJoint	wrist[2];

	std::vector<FeatherJoints>		feathers;

	JointState::JointState()
	{
		for(int i=0;i<Feathers::MaxFeatherName;i++)
		{
			FeatherJoints fj;

			int numOfFeathers	= simBird->birdFeathers->featherGroup[i]->numOfeachFeather;
			int numOfPieces		= (simBird->birdFeathers->featherGroup[i]->numOfPiece-1)/2 + 1;
			
			for(int j=0;j<numOfFeathers;j++)
			{
				Joints jj;

				for(int k=0;k<numOfPieces;k++)
				{
					EachJoint ej;

					jj.joints.push_back(ej);
				}

				fj.fjoint.push_back(jj);
			}

			feathers.push_back(fj);
		}
	};

	void initialize()
	{
		root.initialize();

		for(int i=0;i<2;i++)
		{
			shoulder[i].initialize();
			elbow[i].initialize();
			wrist[i].initialize();
		}

		for(int i=0;i<feathers.size();i++)
			feathers[i].initialize();
	};

	void initializeOri()
	{
		root.initializeOri();

		for(int i=0;i<2;i++)
		{
			shoulder[i].initializeOri();
			elbow[i].initializeOri();
			wrist[i].initializeOri();
		}

		for(int i=0;i<feathers.size();i++)
			feathers[i].initializeOri();
	};
};

class RenderBird
{
public:

	JointState							current;
	vector<JointState>					saveState;
	vector<vector3>						trajectory;

	int									dataSize;
	std::ofstream						foutLocal;
	std::ofstream						foutFeatherInfo;
	
	std::vector<position>				path;

	string	fileName;

	int									currentFrame;;

	vector3 bodyColor;
	vector3 featherColor;
	vector3 trajColor;
	vector3 pathColor;

	RenderBird()
	{
		dataSize		= 0;
		currentFrame	= 0;

		initializeFromCurrentBird();
		//initializeFromTemplateFile();
	}

	inline int	getNumState(){ return (int)saveState.size(); }
	inline int	getStartFrame(){ return 0; }
	inline int	getEndFrame(){ return (int)saveState.size()-1; }

	void		initializeFromCurrentBird();
	void		initializeFromTemplateFile();

	void		update();
	void		updateBody();
	void		updateFeather();

	void		updateFromImportedFramesLocal();
	
	void		saveTemplate();
	void		loadTemplate();
	void		saveCurrentState();
	void		deleteSavedFrame();
	void		loadCurrentFrameState();

	void		drawBird();
	void		drawJoint();
	void		drawFeather();
	void		drawPath();
	void		drawTrajectory();
	
	void		exportFramesLocalRightNow();
	void		convertData();
	void		downSample();

	bool		importFrames();
	bool		importFrames(const char* name);
	void		importPath(const char* name);
	void		importBlendPath(const char* name);
	void		exportBlendPath();

	void		reset();
};