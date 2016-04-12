
#include "definitions.h"
#include "Controller.h"
#include "GUI.h"
#include "SystemState.h"
#include "Bird.h"

extern SystemState*		sysState;
extern CaptureMotion*	captureBird;
extern Bird*			simBird;
extern PhyEngine*		phyEngine;

extern GLUquadricObj*	qObj;

m_real
doNormalize(const m_real value, const m_real max, const m_real min)
{
	util::assert_message(
		max != min,
		"doNormalize >> max is equal to min");

	m_real t = ( value - (max+min)/2.0 ) * 2.0 / (max-min);

	if(t > 1.0)		t = 1.0;
	if(t < -1.0)	t = -1.0;

	return t;
}
m_real
unNormalize(const m_real t, const m_real max, const m_real min)
{
	m_real value = (max+min) / 2.0 + t * (max-min) / 2.0;

	if(value > max) value = max;
	if(value < min) value = min;

	return value;
}

void
Wingbeats::pushWbParamToWbStack(const WingbeatParam& wbParam)
{

}

void	
Wingbeats::copyCurrentToPrevious()
{
	previous.clear();

	// copy current to previous wingbeats

	if(current.size() != 0)
	{
		float* pp;	pp = new float[CaptureMotion::maxDOFname];

		for(int j=0;j<CaptureMotion::maxDOFname;j++)
			pp[j] = current.back()[j];

		previous.push_back(pp);
	}

	for(int i=1;i<(int)current.size()-1;i++)
	{
		float* pp;	pp = new float[CaptureMotion::maxDOFname];

		for(int j=0;j<CaptureMotion::maxDOFname;j++)
			pp[j] = current[i][j];

		previous.push_back(pp);
	}

	prevFrame = previous.size();
}
void
Wingbeats::copyPreviousToPrevious2()
{
	previous2.clear();

	// copy previous to previous2
	for(int i=0;i<(int)previous.size();i++)
	{
		float* pp;	pp = new float[CaptureMotion::maxDOFname];

		for(int j=0;j<CaptureMotion::maxDOFname;j++)
			pp[j] = previous[i][j];

		previous2.push_back(pp);
	}

	prev2Frame = prevFrame;
}
void
Wingbeats::copyCurrentToPast()
{
	previous.clear();
	previous2.clear();

	for(int i=0; i<(int)current.size(); i++)
	{
		previous.push_back( current[i] );
		previous2.push_back( current[i] );
	}

	prevFrame = frame;
	prev2Frame = frame;
}
void
Wingbeats::setAvgWingbeatParam()
{
	//요기
	avgWbParam.frame = captureBird->refWingbeat->timewarp * (captureBird->refWingbeat->duration-1) + 1;	

	for(int i=0;i<CaptureMotion::maxDOFname;i++)
	{
		avgWbParam.angleScale[i] = captureBird->refWingbeat->scale[i];			
		avgWbParam.angleTrans[i] = captureBird->refWingbeat->translation[i];
	}

	/*ofstream fout("param_default.txt");
	for(int i=0;i<CaptureMotion::maxDOFname;i++)
		fout << avgWbParam.angleScale[i] << " " << avgWbParam.angleTrans[i] << endl;
	fout << avgWbParam.frame << endl;*/

	
	ifstream fin;

	if(util::fileOpenWrapper(fin, INFORM_DIR+string("wingbeat/param_default.txt")))
	{
		for(int i=0;i<CaptureMotion::maxDOFname;i++)
		{
			fin >> avgWbParam.angleScale[i];
			fin >> avgWbParam.angleTrans[i];
		}
		fin >> avgWbParam.frame;

		util::fileCloseWrapper(fin);
	}
}
void
Wingbeats::setCurrentFromDeltaWbParam(const WingbeatParam& deltaWbParam)
{
	setCurrentFromWbParam( avgWbParam + deltaWbParam );
}
void
Wingbeats::setCurrentFromWbParam(const WingbeatParam& wbParam)
{
	restFrame = 0;

	if(current.size() != 0)
	{
		float* cc;
		cc = new float[CaptureMotion::maxDOFname];
		for(int j=0;j<CaptureMotion::maxDOFname;j++)	cc[j] = current.back()[j];

		current.clear();
		current.push_back(cc);				
	}

	frame = (int)wbParam.frame;

	float plusRest	= 1.0f - restFrame;	

	if(SAME_VALUE(restFrame, 0.0f))		plusRest = 0.0f;
	if(plusRest < 0.0f)
	{
		cout << "negative plus rest" << endl;
		cin.get(); cin.get();
	}
	
	float thisFrame		= wbParam.frame - plusRest;
	float fThisFrame	= thisFrame - (int)thisFrame;

	for(int i=0;i<frame;i++)
	{
		float* pp;
		pp = new float[CaptureMotion::maxDOFname];

		int	idx;
		float ww;
			
		ww	= (float)(captureBird->refWingbeat->duration-1)/(frame-1)*i + plusRest;
		idx	= (int)ww;	
		ww	= ww - idx;
		
		for(int j=0;j<CaptureMotion::maxDOFname;j++)
		{
			if(i == frame-1)	pp[j] = captureBird->refWingbeat->pos[j][idx] + (captureBird->refWingbeat->pos[j][0]		- captureBird->refWingbeat->pos[j][idx])*ww;
			else				pp[j] = captureBird->refWingbeat->pos[j][idx] + (captureBird->refWingbeat->pos[j][idx+1]	- captureBird->refWingbeat->pos[j][idx])*ww;

			pp[j] = (pp[j] - captureBird->refWingbeat->minPoint[j]) * wbParam.angleScale[j] + captureBird->refWingbeat->minPoint[j] + wbParam.angleTrans[j];
		}

		current.push_back(pp);
	}

	/*float* bb;
	bb = new float[CaptureMotion::maxDOFname];
	for(int j=0;j<CaptureMotion::maxDOFname;j++)	bb[j] = current.front()[j];
	current.push_back(bb);*/

	frame = current.size();

	restFrame = fThisFrame;

	//cout << "frame: " << frame << " / restFrame: " << fThisFrame << endl;
}
void
Wingbeats::setCurrentFromWbParam(const float weight)
{
	if(current.size() != 0)
	{
		float* cc;
		cc = new float[CaptureMotion::maxDOFname];
		for(int j=0;j<CaptureMotion::maxDOFname;j++)	cc[j] = current.back()[j];

		current.clear();
		current.push_back(cc);				
	}

	frame = (int)avgWbParam.frame;

	float plusRest	= 1.0f - restFrame;	

	if(SAME_VALUE(restFrame, 0.0f))		plusRest = 0.0f;
	
	float thisFrame		= avgWbParam.frame - plusRest;
	float fThisFrame	= thisFrame - (int)thisFrame;

	float desiredframe	= frame * weight;

	int idx				= (int)desiredframe;
	float ww			= desiredframe - idx;

	for(int i=0;i<frame;i++)
	{
		float* pp;
		pp = new float[CaptureMotion::maxDOFname];
		
		for(int j=0;j<CaptureMotion::maxDOFname;j++)
		{
			if(idx == frame-1)	pp[j] = captureBird->refWingbeat->pos[j][idx] + (captureBird->refWingbeat->pos[j][0]		- captureBird->refWingbeat->pos[j][idx])*ww;
			else				pp[j] = captureBird->refWingbeat->pos[j][idx] + (captureBird->refWingbeat->pos[j][idx+1]	- captureBird->refWingbeat->pos[j][idx])*ww;

			pp[j] = (pp[j] - captureBird->refWingbeat->minPoint[j]) * avgWbParam.angleScale[j] + captureBird->refWingbeat->minPoint[j] + avgWbParam.angleTrans[j];
		}

		current.push_back(pp);
	}

	/*float* bb;
	bb = new float[CaptureMotion::maxDOFname];
	for(int j=0;j<CaptureMotion::maxDOFname;j++)	bb[j] = current.front()[j];
	current.push_back(cc);		
*/
	frame = current.size();

	restFrame = fThisFrame;
}
void	
Wingbeats::setReference()
{
	float plusRest	= 1.0f - restFrame;

	if(SAME_VALUE(restFrame, 0.0f))		plusRest = 0.0f;
	frame = (int)avgWbParam.frame;

	float thisFrame = avgWbParam.frame - plusRest;
	float fThisFrame = thisFrame - (int)thisFrame;

	for(int i=0;i<frame;i++)
	{
		float* pp;
		pp = new float[CaptureMotion::maxDOFname];

		for(int j=0;j<CaptureMotion::maxDOFname;j++)
		{
			int	idx;
			float ff;
			
			ff	= (float)(captureBird->refWingbeat->duration)/(frame-1)*i + plusRest;
			idx	= (int)ff;	
			ff	= ff - idx;
			
			if(i == frame-1)	pp[j] = captureBird->refWingbeat->pos[j][idx] + (captureBird->refWingbeat->pos[j][0]		- captureBird->refWingbeat->pos[j][idx])*ff;
			else				pp[j] = captureBird->refWingbeat->pos[j][idx] + (captureBird->refWingbeat->pos[j][idx+1]	- captureBird->refWingbeat->pos[j][idx])*ff;

			pp[j] = (pp[j] - captureBird->refWingbeat->minPoint[j]) * avgWbParam.angleScale[j] + captureBird->refWingbeat->minPoint[j] + avgWbParam.angleTrans[j];
		}

		reference.push_back(pp);
	}

	reference.push_back( reference[0] );		// 맨 앞에 frame을 맨 끝에 복사해서 붙여두기
}
void
Wingbeats::setBlendedCurrent(const float ratio)
{
	blendedCurrent.clear();

	//cout << "f1" << endl;
	for(int i=0; i<(int)current.size(); i++)
	{
		float* pp = new float[CaptureMotion::maxDOFname];
		float alpha;

		//cout << "f2" << endl;
		if( i/((float)current.size()-1.0f) >= ratio )
			alpha = 1.0;
		else
			alpha = (i/((float)current.size()-1.0f)) / ratio;

		//cout << "f3" << endl;
		//cout << "ratio: " << ratio << " / a: " << i/((float)current.size()-1.0f) << " / alpha: " << alpha << endl;

		if( alpha >= 1.0)
			for(int j=0; j<CaptureMotion::maxDOFname; j++)
				pp[j] = current[i][j];
		else
		{
			for(int j=0; j<CaptureMotion::maxDOFname; j++)
				pp[j] = (1.0 - alpha) * previous[i][j] + alpha * current[i][j];
		}

		//cout << "f4" << endl;

		blendedCurrent.push_back(pp);
	}

}
void
Wingbeats::setBlendedReference()
{

}
float	
Wingbeats::getInterpolation(int frame, float weight, int idx)
{
	float res;

	// linear interpolation
	//res = current[frame][idx] * (1.0f - weight) + current[frame+1][idx] * weight;
	res = blendedCurrent[frame][idx] * (1.0f - weight) + blendedCurrent[frame+1][idx] * weight;

	return res;
}

quater
Wingbeats::getShoulder(int frame, float weight, bool isLeft)
{
	quater	resQ;
	int		dofIndex;
	float	sign;

	if(isLeft)	{	dofIndex = CaptureMotion::d_lshoulder1;		sign = 1.0f;	}
	else		{	dofIndex = CaptureMotion::d_rshoulder1;		sign = -1.0f;	}

	vector3 tempV(getInterpolation(frame, weight, dofIndex), sign*getInterpolation(frame, weight, dofIndex+1), sign*getInterpolation(frame, weight, dofIndex+2));
	resQ = exp(tempV);

	//resQ.Identity();

	return resQ;
}
float	
Wingbeats::getElbowBend(int frame, float weight, bool isLeft)
{
	float	res;
	int		dofIndex;
	float	sign;

	if(isLeft)	{	dofIndex = CaptureMotion::d_lelbow_bend;		sign = 1.0f;	}
	else		{	dofIndex = CaptureMotion::d_relbow_bend;		sign = -1.0f;	}

	res = (getInterpolation(frame, weight, dofIndex) - M_PI) * sign;

	return res;
}
float	
Wingbeats::getElbowTwist(int frame, float weight, bool isLeft)
{
	float	res;
	int		dofIndex;
	float	sign;

	if(isLeft)	{	dofIndex = CaptureMotion::d_lelbow_twist;	sign = -1.0f;	}
	else		{	dofIndex = CaptureMotion::d_relbow_twist;	sign = -1.0f; 	}

	res = getInterpolation(frame, weight, dofIndex) * sign;

	//res = -M_PI/4.0f;

	return res;
}
float	
Wingbeats::getWrist(int frame, float weight, bool isLeft)
{
	float	res;
	int		dofIndex;
	float	sign;

	if(isLeft)	{	dofIndex = CaptureMotion::d_lwrist_bend;		sign = 1.0f;	}
	else		{	dofIndex = CaptureMotion::d_rwrist_bend;		sign = -1.0f;	}

	res = (M_PI - getInterpolation(frame, weight, dofIndex)) * sign;

	//res = M_PI/2.0f * sign;

	return res;
}
float	
Wingbeats::getTailBend(int frame, float weight)
{
	return getInterpolation(frame, weight, CaptureMotion::d_tail_bend);
}
float	
Wingbeats::getTailSpread(int frame, float weight)
{
	return getInterpolation(frame, weight, CaptureMotion::d_tail_spread);
}
////////////////////////////////////////////////////////////

Controller::Controller()
{
	mode = None;

	initOffset = QI;

	sysState->simul_interval	= sysState->STEPS * sysState->FPS / captureBird->captureFPS;

	realWingbeat.setAvgWingbeatParam();
}
Controller::~Controller()
{
}
void
Controller::reset()
{
	realWingbeat.setAvgWingbeatParam();

	simulStep = 0;
}

WingbeatParam
Controller::captureParam2controlParam(int k)
{
	WingbeatParam resP;
	
	resP.frame = captureBird->wingbeats[k]->timewarp * (captureBird->refWingbeat->duration-1) + 1;	

	for(int i=0;i<CaptureMotion::maxDOFname;i++)
	{
		resP.angleScale[i] = captureBird->wingbeats[k]->scale[i];
		resP.angleTrans[i] = captureBird->wingbeats[k]->translation[i];
	}

	return resP;
}
// given simul_cnt is for checking btw wingbeats
// return simul_cnt as 0 when current wingbeat ends.
int
Controller::setCurrentFrame(const int simul_cnt)
{
	int res = simul_cnt;
	// to ejjoo : f가 simul_cnt로 이름이 바뀜

	// cature timestep is lower than simulation timestep. 
	// So, should interpolate btw cature frames

	// frame to be simulate
	currentFrame	= (int)(simul_cnt / sysState->simul_interval);
	// weight btw a frame and a frame
	currentWeight	= (simul_cnt % sysState->simul_interval)/(float)(sysState->simul_interval);

	if( (currentFrame == 0 || currentFrame == realWingbeat.frame-1 ) && SAME_VALUE(currentWeight, 0.0f) )
	{
		//cout << simBird->birdBody->count << "          " << simBird->birdBody->count_wb << endl;

		currentFrame = 0;

		realWingbeat.copyPreviousToPrevious2();
		realWingbeat.copyCurrentToPrevious();

		//realWingbeat.setCurrentFromWbParam( wbParam );
		// to ejjoo : 이부분이 아래 setNewWbParam으로 내려감( body부분에서 날개짓한 횟수를 보고 날개짓을 만들기 위해서)

		res = 0;
	}

	
	wbRatio =  (float)simul_cnt / (float)(sysState->simul_interval * (realWingbeat.frame-1));
	//cout << "wbRatio : " << wbRatio << endl;
	
	return res;
}
void
Controller::setNewWbParam(const vectorN& wb)
{
	WingbeatParam wbParam;
	wbParam.fromVectorN( wb );

	realWingbeat.setCurrentFromWbParam( wbParam );
	realWingbeat.setBlendedCurrent();
}
void
Controller::setNewWbParam(const WingbeatParam& wbParam)
{
	//cout << "r0" << endl;
	realWingbeat.setCurrentFromWbParam( wbParam );
	//cout << "r1" << endl;
	realWingbeat.setBlendedCurrent();
}
void
Controller::setNewWbParam(const float t)
{
	realWingbeat.setCurrentFromWbParam( t );
	realWingbeat.setBlendedCurrent(t);
}
BodyState	
Controller::getDesiredState(quater trunk, const BodyState& prevState)
{
	// 요기

	BodyState bs;
	float dT = sysState->TIME_STEP;

	bool	isLeft;
#ifdef OLD
	float offsetX = -20;
	float offsetY = -5;
	float offsetZ = 3;//10;
#else
	float offsetX = -25;
	float offsetY = 0;
	float offsetZ = 0;//10;
#endif
	
	quater offset;

	for(int i=0;i<2;i++)
	{
		if(i==0)	isLeft = true;
		else		isLeft = false;

		

		if(isLeft)
		{
			offset = rotate_transf(DegToRad(offsetX), vector3(1,0,0)).getRotation();
			offset = offset * rotate_transf(DegToRad(offsetY), vector3(0,1,0)).getRotation();
			offset = offset * rotate_transf(DegToRad(offsetZ), vector3(0,0,1)).getRotation();
		}
		else
		{
			offset = rotate_transf(DegToRad(offsetX), vector3(1,0,0)).getRotation();
			offset = offset * rotate_transf(DegToRad(-offsetY), vector3(0,1,0)).getRotation();
			offset = offset * rotate_transf(DegToRad(-offsetZ), vector3(0,0,1)).getRotation();
		}

		bs.shoulder[i]		= trunk * offset * realWingbeat.getShoulder(currentFrame, currentWeight, isLeft);
		bs.elbowBend[i]		= realWingbeat.getElbowBend(currentFrame, currentWeight, isLeft);
		bs.elbowTwist[i]	= realWingbeat.getElbowTwist(currentFrame, currentWeight, isLeft);
		bs.wristBend[i]		= realWingbeat.getWrist(currentFrame, currentWeight, isLeft);

		bs.dShoulder[i]		= GET_ANGULAR_VELOCITY_QUATER( bs.shoulder[i], prevState.shoulder[i], dT );
		bs.dElbowBend[i]	= (bs.elbowBend[i]	- prevState.elbowBend[i])	/ dT;
		bs.dElbowTwist[i]	= (bs.elbowTwist[i] - prevState.elbowTwist[i])	/ dT;
		bs.dWristBend[i]	= (bs.wristBend[i]	- prevState.wristBend[i])	/ dT;

		bs.ddShoulder[i]	= (bs.dShoulder[i] - prevState.dShoulder[i])		/ dT;
		bs.ddElbowBend[i]	= (bs.dElbowBend[i] - prevState.dElbowBend[i])		/ dT;
		bs.ddElbowTwist[i]	= (bs.dElbowTwist[i] - prevState.dElbowTwist[i])	/ dT;
		bs.ddWristBend[i]	= (bs.dWristBend[i] - prevState.dWristBend[i])		/ dT;

		// print
		//WRITE_ARRAY4(cout, bs.shoulder[i]);
		//cout << "angle " << RadToAng(bs.elbowBend[i]) << "," << RadToAng(bs.elbowTwist[i]) << "," << RadToAng(bs.wristBend[i]) << endl;

		//WRITE_ARRAY3(cout, bs.dShoulder[i]);
		//cout << "angle vel " << RadToAng(bs.dElbowBend[i]) << "," << RadToAng(bs.dElbowTwist[i]) << "," << RadToAng(bs.dWristBend[i]) << endl;
	}

	return bs;
}
BodyState
Controller::modifyCurrentState(const BodyState& curState)
{
	BodyState mcs;
	float dT = sysState->TIME_STEP;

	for(int i=0; i<2; i++)
	{
		mcs.shoulder[i]		= exp(0.5f * dT * curState.dShoulder[i]) * curState.shoulder[i];
		mcs.elbowBend[i]	= curState.elbowBend[i]		+ dT * curState.dElbowBend[i];
		mcs.elbowTwist[i]	= curState.elbowTwist[i]	+ dT * curState.dElbowTwist[i];
		mcs.wristBend[i]	= curState.wristBend[i]		+ dT * curState.dWristBend[i];

		mcs.dShoulder[i]	= curState.dShoulder[i]		+ dT * curState.ddShoulder[i];
		mcs.dElbowBend[i]	= curState.dElbowBend[i]	+ dT * curState.ddElbowBend[i];
		mcs.dElbowTwist[i]	= curState.dElbowTwist[i]	+ dT * curState.ddElbowTwist[i];
		mcs.dWristBend[i]	= curState.dWristBend[i]	+ dT * curState.ddWristBend[i];

		mcs.ddShoulder[i]	= curState.ddShoulder[i];
		mcs.ddElbowBend[i]	= curState.ddElbowBend[i];
		mcs.ddElbowTwist[i]	= curState.ddElbowTwist[i];
		mcs.ddWristBend[i]	= curState.ddWristBend[i];
	}

	return mcs;
}
BodyState
Controller::modifyDesiredState(const BodyState& desState)
{
	BodyState mds;
	float dT = sysState->TIME_STEP;

	for(int i=0; i<2; i++)
	{
		mds.shoulder[i]		= exp(0.5f * dT * desState.dShoulder[i]) * desState.shoulder[i];
		mds.elbowBend[i]	= desState.elbowBend[i]		+ dT * desState.dElbowBend[i];
		mds.elbowTwist[i]	= desState.elbowTwist[i]	+ dT * desState.dElbowTwist[i];
		mds.wristBend[i]	= desState.wristBend[i]		+ dT * desState.dWristBend[i];

		mds.dShoulder[i]	= desState.dShoulder[i]		+ dT * desState.ddShoulder[i];
		mds.dElbowBend[i]	= desState.dElbowBend[i]	+ dT * desState.ddElbowBend[i];
		mds.dElbowTwist[i]	= desState.dElbowTwist[i]	+ dT * desState.ddElbowTwist[i];
		mds.dWristBend[i]	= desState.dWristBend[i]	+ dT * desState.ddWristBend[i];

		mds.ddShoulder[i]	= desState.ddShoulder[i];
		mds.ddElbowBend[i]	= desState.ddElbowBend[i];
		mds.ddElbowTwist[i]	= desState.ddElbowTwist[i];
		mds.ddWristBend[i]	= desState.ddWristBend[i];
	}

	return mds;
}
void
Controller::getDesiredTailState(float* ret)
{//요기
	ret[0] = realWingbeat.getTailBend(currentFrame, currentWeight); + Deg2Rad(-5);
	ret[1] = realWingbeat.getTailSpread(currentFrame, currentWeight);

#ifdef PEACOCK
	ret[1] = AngToRad(-35);
#endif
	
}
void
Controller::saveState(const int sID)
{
	controller_save[sID].simulStep				= simulStep;
	controller_save[sID].currentFrame			= currentFrame;
	controller_save[sID].currentWeight			= currentWeight;
	controller_save[sID].wbRatio				= wbRatio;
	controller_save[sID].realWingbeat			= realWingbeat;
}
void
Controller::restoreState(const int sID)
{
	simulStep				= controller_save[sID].simulStep;
	currentFrame			= controller_save[sID].currentFrame;
	currentWeight			= controller_save[sID].currentWeight;
	wbRatio					= controller_save[sID].wbRatio;
	realWingbeat			= controller_save[sID].realWingbeat;
}

void
Controller::do_None()
{
	setNewWbParam( realWingbeat.avgWbParam );
}

void
Controller::draw()
{	
	
}
void			
Controller::drawEachAxis(float x, float y, float frame)
{
	float sx = 0.03f;
	float sy = 0.45f;

	/*glColor3f(0.96f, 0.96f, 0.96f);
	glBegin(GL_QUADS);
	glVertex2f(x, y);
	glVertex2f(x + frame*sx, y);
	glVertex2f(x + frame*sx, y + M_PI*sy);
	glVertex2f(x, y + M_PI*sy);
	glEnd();*/

	glLineWidth(1.0f);
	glColor3f(0.8f, 0.8f, 0.8f);	
	glBegin(GL_LINES);	
	glVertex2f(x, y);					glVertex2f(x + frame*sx, y);		
	glVertex2f(x, y + M_PI/2.0f*sy);	glVertex2f(x + frame*sx, y + M_PI/2.0f*sy);		
	glEnd();
}
void			
Controller::drawEachGraph(int idx, float y, vector3 color)
{
	float x= 0.0f;

	float sx = 0.03f;
	float sy = 0.45f;

	glLineWidth(1.0f);
	glColor3f(0.8f, 0.8f, 0.8f);	
	glBegin(GL_LINES);	
	glVertex2f(x + (realWingbeat.prevFrame-1) * sx, y - M_PI/2.0f*sy);					
	glVertex2f(x + (realWingbeat.prevFrame-1) * sx, y + M_PI/2.0f*sy);		
	glEnd();

	color *= 0.5f;
	glColor3f(color[0], color[1], color[2]);

	glBegin(GL_LINE_STRIP);
	for(int i=0;i<realWingbeat.prevFrame;i++)
	{
		float value = realWingbeat.previous[i][idx];
		glVertex2f(x + i*sx, y + value *sy);
	}

	for(int i=0;i<realWingbeat.frame;i++)
	{
		float value = realWingbeat.current[i][idx];
		glVertex2f(x + realWingbeat.prevFrame * sx + i*sx, y + value *sy);
	}

	glEnd();

	color *= 2.0f;
	glColor3f(color[0], color[1], color[2]);

	glPushMatrix();
	glTranslatef(sx*30,0,0);

	glBegin(GL_LINE_STRIP);
	for(int i=0;i<realWingbeat.prevFrame;i++)
	{
		float value = realWingbeat.previous[i][idx];
		glVertex2f(x + i*sx, y + value *sy);
	}

	for(int i=0;i<realWingbeat.frame;i++)
	{
		float value = realWingbeat.blendedCurrent[i][idx];
		glVertex2f(x + realWingbeat.prevFrame * sx + i*sx, y + value *sy);
	}

	glEnd();

	glPopMatrix();

	/*glColor3f(color[1], color[0], color[2]);

	glBegin(GL_LINE_STRIP);
	for(int i=0;i<realWingbeat.prevFrame;i++)
	{
		float value = realWingbeat.previous[i][idx];
		glVertex2f(x + i*sx, y + value *sy);
	}

	for(int i=0;i<realWingbeat.prevFrame;i++)
	{
		float value = realWingbeat.previous[i][idx];
		glVertex2f(x + realWingbeat.prevFrame * sx + i*sx, y + value *sy);
	}

	glEnd();*/
}
void			
Controller::extractEachGraph(int idx, float y, vector3 color)
{
	float x= 0.0f;

	float sx = 0.03f;
	float sy = 0.45f;

	char buf_t[1204], buf_d[1024];

	sprintf(buf_t, "%s%d%s", "Informations/graph/",idx,"_t.txt");
	sprintf(buf_d, "%s%d%s", "Informations/graph/",idx,"_d.txt");
	
	ofstream fout_t(buf_t);
	ofstream fout_d(buf_d);

	float min = 1.0e6;
	float max = -1.0e6;

	/*for(int i=0;i<realWingbeat.prevFrame;i++)
	{
		float value = realWingbeat.previous[i][idx];

		if(value > max) max = value;
		if(value < min) min = value;
	}

	for(int i=0;i<realWingbeat.prevFrame;i++)
	{
		float value = realWingbeat.previous[i][idx];

		fout_t << i/(realWingbeat.prevFrame-1.0) << endl;
		fout_d <<  (value-min)/(max-min) << endl;
	}*/

	for(int i=0;i<realWingbeat.prevFrame;i++)
	{
		float value = realWingbeat.previous[i][idx];
		//float value_t = realWingbeat.previous[i][idx];

		fout_t << i/(realWingbeat.prevFrame-1.0) << endl;
		fout_d << value << endl;
	}

	glEnd();

	fout_t.close();
	fout_d.close();
}
void
Controller::drawGraph()
{
	float sx = 0.03f;
	float sy = 0.45f;

	float xOffset = 0.0f;

	glDisable(GL_LIGHTING);
	
	float unit = 2.0f, temp;
	char title[256];

	for(int i=0;i<1;i++)
	{
		float frame;

		frame = realWingbeat.prevFrame + realWingbeat.frame;
		
		drawEachAxis(xOffset, unit*6.0f, frame);
		drawEachGraph( CaptureMotion::d_lshoulder1, unit*6.0f, autoColor(0));
		//drawEachGraph( CaptureMotion::d_rshoulder1, unit*6.0f, autoColor(1));

		/*drawEachAxis(xOffset, unit*5.0f, frame);
		drawEachGraph( CaptureMotion::d_lshoulder2, unit*5.0f, autoColor(0));
		drawEachGraph( CaptureMotion::d_rshoulder2, unit*5.0f, autoColor(1));

		drawEachAxis(xOffset, unit*4.0f, frame);
		drawEachGraph( CaptureMotion::d_lshoulder3, unit*4.0f, autoColor(0));
		drawEachGraph( CaptureMotion::d_rshoulder3, unit*4.0f, autoColor(1));*/

		/*extractEachGraph( CaptureMotion::d_lshoulder1, unit*6.0f, autoColor(0));
		extractEachGraph( CaptureMotion::d_lshoulder2, unit*5.0f, autoColor(0));
		extractEachGraph( CaptureMotion::d_lshoulder3, unit*4.0f, autoColor(0));

		extractEachGraph( CaptureMotion::d_lelbow_bend, unit*5.0f, autoColor(0));
		extractEachGraph( CaptureMotion::d_lelbow_twist, unit*4.0f, autoColor(0));

		extractEachGraph( CaptureMotion::d_lwrist_bend, unit*5.0f, autoColor(0));
		
		extractEachGraph( CaptureMotion::d_tail_bend, unit*5.0f, autoColor(0));
		extractEachGraph( CaptureMotion::d_tail_spread, unit*4.0f, autoColor(0));

		cin.get();
		cin.get();*/

		/*drawEachAxis(xOffset, unit*3.0f, frame);
		drawEachGraph( CaptureMotion::d_lelbow_bend, unit*3.0f, autoColor(0));
		drawEachGraph( CaptureMotion::d_relbow_bend, unit*3.0f, autoColor(1));

		drawEachAxis(xOffset, unit*2.0f, frame);
		drawEachGraph( CaptureMotion::d_lelbow_twist, unit*2.0f, autoColor(0));
		drawEachGraph( CaptureMotion::d_relbow_twist, unit*2.0f, autoColor(1));

		drawEachAxis(xOffset, unit*1.0f, frame);
		drawEachGraph( CaptureMotion::d_lwrist_bend, unit*1.0f, autoColor(0));
		drawEachGraph( CaptureMotion::d_rwrist_bend, unit*1.0f, autoColor(1));

		drawEachAxis(xOffset, unit*0.0f, frame);
		drawEachGraph( CaptureMotion::d_tail_bend, unit*0.0f, autoColor(2));
		drawEachGraph( CaptureMotion::d_tail_spread, unit*0.0f, autoColor(3));	*/
	}

	glEnable(GL_LIGHTING);
}
void
Controller::drawExample()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////