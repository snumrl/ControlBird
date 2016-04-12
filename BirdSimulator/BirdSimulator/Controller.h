#pragma once

#include <MATHCLASS/mathclass.h>
#include <vector>
#include <iterator>
#include <fstream>
#include "util.h"
#include "Body.h"
#include "CaptureMotion.h"

using namespace std;

extern string date;

const int rUniformHalf[20] = {6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44};
const int rUniformQuad[20] = {4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23};

quater 
getFrameFromTangent( vector3 tangent );

struct 
WingbeatParam
{
	vector<float>	angleScale;
	vector<float>	angleTrans;
	float			frame;

	WingbeatParam()
	{
		for(int i=0; i<CaptureMotion::maxDOFname; i++)
		{
			angleScale.push_back(0);
			angleTrans.push_back(0);
		}
		frame	= 0;
	}

	inline void clear()
	{
		for(int i=0; i<CaptureMotion::maxDOFname; i++)
		{
			angleScale.push_back(0);
			angleTrans.push_back(0);
		}
		frame	= 0;
	}

	inline void random(vector<float>& angS, float angS_range, vector<float>& angT, float angT_range, float f, float f_range)
	{
		for(int i=0; i<CaptureMotion::maxDOFname; i++)
		{
			angleScale[i] = M_RANDOM(angS[i]-angS_range, angS[i]+angS_range);
			angleTrans[i] = M_RANDOM(angT[i]-angT_range, angT[i]+angT_range);
		}
		frame = M_RANDOM(f-f_range, f+f_range);
	}

	WingbeatParam& operator=( const WingbeatParam& _w )
	{	
		for(int i=0; i<CaptureMotion::maxDOFname; i++)
		{
			this->angleScale[i] = _w.angleScale[i];
			this->angleTrans[i] = _w.angleTrans[i];
		}
		this->frame		= _w.frame;
		
		return *this;
	}

	WingbeatParam operator+( const WingbeatParam& b )
	{
		WingbeatParam c;

		for(int i=0; i<CaptureMotion::maxDOFname; i++)
		{
			c.angleScale[i] = this->angleScale[i] + b.angleScale[i];
			c.angleTrans[i] = this->angleTrans[i] + b.angleTrans[i];
		}
		c.frame = this->frame + b.frame;

		return c;
	}

	WingbeatParam operator-( const WingbeatParam& b )
	{
		WingbeatParam c;

		for(int i=0; i<CaptureMotion::maxDOFname; i++)
		{
			c.angleScale[i] = this->angleScale[i] - b.angleScale[i];
			c.angleTrans[i] = this->angleTrans[i] - b.angleTrans[i];
		}
		c.frame = this->frame - b.frame;

		return c;
	}

	friend ostream& operator<<( ostream& os, WingbeatParam& wb )
	{
		for(int i=0; i<CaptureMotion::maxDOFname; i++)
		{
			os << wb.angleScale[i] << " ";
			os << wb.angleTrans[i] << endl;
		}
		os << wb.frame << endl;

		return os;
	}

	friend istream& operator>>( istream& is, WingbeatParam& wb )
	{
		for(int i=0; i<CaptureMotion::maxDOFname; i++)
		{
			is >> wb.angleScale[i];
			is >> wb.angleTrans[i];
		}
		is >> wb.frame;

		return is;
	}

	WingbeatParam* mirror()
	{
		WingbeatParam *wb = new WingbeatParam();

		// shoulder
		for(int i=0; i<=2; i++)
		{
			wb->angleScale[i] = angleScale[i+3];
			wb->angleTrans[i] = angleTrans[i+3];
		}
		for(int i=3; i<=5; i++)
		{
			wb->angleScale[i] = angleScale[i-3];
			wb->angleTrans[i] = angleTrans[i-3];
		}

		// elbow
		for(int i=6; i<=7; i++)
		{
			wb->angleScale[i] = angleScale[i+2];
			wb->angleTrans[i] = angleTrans[i+2];
		}
		for(int i=8; i<=9; i++)
		{
			wb->angleScale[i] = angleScale[i-2];
			wb->angleTrans[i] = angleTrans[i-2];
		}

		// wrist
		wb->angleScale[10] = angleScale[11];
		wb->angleScale[11] = angleScale[10];

		// tail
		wb->angleScale[12] = angleScale[12];
		wb->angleScale[13] = angleScale[13];

		// time warp
		wb->frame = frame;

		return wb;
	}

	void toVectorN( vectorN& v )
	{
		v.setSize(29);

		for(int i=0; i<14; i++)
		{
			v[2*i+0] = angleScale[i];
			v[2*i+1] = angleTrans[i];
		}
		v[28] = frame;
	}

	void fromVectorN( const vectorN& v )
	{
		for(int i=0; i<14; i++)
		{
			angleScale[i] = v[2*i+0];
			angleTrans[i] = v[2*i+1];
		}
		frame = v[28];
	}
};
struct 
ControlState
{
	quater			gOri;
	vector3			gOriE;
	vector3			frameAngVel;
	vector3			frameLinVel;

	//position		pos;
	//quater			ori;
	//WingbeatParam	wb;

	ControlState& operator=( const ControlState& _cs )
	{
		gOri	= _cs.gOri;
		gOriE	= _cs.gOriE;
		frameAngVel = _cs.frameAngVel;
		frameLinVel = _cs.frameLinVel;

		return *this;
	}

	friend ostream& operator<<( ostream& os, ControlState& cs )
	{
		os << cs.gOriE[0] << " " << cs.gOriE[2] << endl;
		WRITE_ARRAY3( os, cs.frameAngVel );
		WRITE_ARRAY3( os, cs.frameLinVel );

		return os;
	}
};
struct
OneWingbeat
{
	vector< array<float> > p;
};
struct 
Wingbeats
{
	// previous2 wingbeat frame;
	int						prev2Frame;
	// previous wingbeat frame;
	int						prevFrame;
	// current wingbeat frame;
	int						frame;

	float					restFrame;

	WingbeatParam			avgWbParam;

	// wingbeat param stack
	vector<WingbeatParam>	wbParamStack;

	vector<float*>			previous2;
	vector<float*>			previous;		// [frame][dof]
	vector<float*>			current;
	vector<float*>			blendedPrevious;
	vector<float*>			blendedCurrent;
	vector<float*>			reference;
	vector<float*>			blendedReference;

	Wingbeats()
	{
		restFrame = 0.0f;
		prevFrame = 0;
		frame = 0;
	};

	Wingbeats& operator=(const Wingbeats& _wb)
	{
		prev2Frame	= _wb.prev2Frame;
		prevFrame	= _wb.prevFrame;
		frame		= _wb.frame;
		restFrame	= _wb.restFrame;
		avgWbParam	= _wb.avgWbParam;

		wbParamStack = _wb.wbParamStack;

		clean_copy_floatStar_STL_vector(CaptureMotion::maxDOFname, previous2, _wb.previous2);
		clean_copy_floatStar_STL_vector(CaptureMotion::maxDOFname, previous, _wb.previous);
		clean_copy_floatStar_STL_vector(CaptureMotion::maxDOFname, current, _wb.current);
		clean_copy_floatStar_STL_vector(CaptureMotion::maxDOFname, blendedPrevious, _wb.blendedPrevious);
		clean_copy_floatStar_STL_vector(CaptureMotion::maxDOFname, blendedCurrent, _wb.blendedCurrent);
		clean_copy_floatStar_STL_vector(CaptureMotion::maxDOFname, reference, _wb.reference);
		clean_copy_floatStar_STL_vector(CaptureMotion::maxDOFname, blendedReference, _wb.blendedReference);

		return *this;
	}

	void	pushWbParamToWbStack(const WingbeatParam& wbParam);

	void	setAvgWingbeatParam();	
	
	void	setCurrentFromWbParam(const WingbeatParam& wbParam);
	void	setCurrentFromWbParam(const float t);
	void	setCurrentFromDeltaWbParam(const WingbeatParam& deltaWbParam);
	//		average wingbeat param
	void	setReference();
	void	setBlendedCurrent(const float ratio=0.333333);
	void	setBlendedReference();
	void	copyCurrentToPrevious();
	void	copyPreviousToPrevious2();
	void	copyCurrentToPast();

	float	getInterpolation(int frame, float weight, int idx);

	quater	getShoulder(int frame, float weight, bool isLeft);
	float	getElbowBend(int frame, float weight, bool isLeft);
	float	getElbowTwist(int frame, float weight, bool isLeft);
	float	getWrist(int frame, float weight, bool isLeft);
	float	getTailBend(int frame, float weight);
	float	getTailSpread(int frame, float weight);
};
struct
ControllerState
{
	int									simulStep;

	int									currentFrame;
	float								currentWeight;

	float								wbRatio;

	// average wingbeat으로 날릴때 true
	bool								isFakeSim;

	// 새는 realWingbeat의 blended_current에 의해 움직임
	// fake simulation 할때는 blended_reference에 의해
	Wingbeats							realWingbeat;
	
	// blend length btw local, global path
	float								localPathBlendStride;

	// glide로 날릴때 필요한 정보
	bool								frontIsGlide;
	bool								isGlideMode;


	ControllerState& operator=(const ControllerState& _cs)
	{
		simulStep		= _cs.simulStep;

		currentFrame	= _cs.currentFrame;
		currentWeight	= _cs.currentWeight;

		wbRatio			= _cs.wbRatio;

		isFakeSim		= _cs.isFakeSim;
		realWingbeat	= _cs.realWingbeat;

		localPathBlendStride = _cs.localPathBlendStride;
		frontIsGlide	= _cs.frontIsGlide;
		isGlideMode		= _cs.isGlideMode;

		return *this;
	}
};

/////////////////////

class 
Controller
{
public:
	enum ControlMode	
	{ 
		None=0, 
		Train_Data, 
		Control_Analysis, 
		K_Regression, LW_Regression, NeuralNet, GP, GPLVM, 
		Manual,
		Opt_Once,
		Opt_Grid,
		Grid_Filter,
		Interactive
	};

	// control mode
	ControlMode							mode;
	int									simulStep;

	int									currentFrame;
	float								currentWeight;

	float								wbRatio;

	// 새는 realWingbeat의 blended_current에 의해 움직임
	// fake simulation 할때는 blended_reference에 의해
	Wingbeats							realWingbeat;

	quater								initOffset;
	BodyState							current;
	BodyState							desired;

	WingbeatParam						controlWbParam;

	// for save
	ControllerState						controller_save[max_save];
											
	WingbeatParam	captureParam2controlParam(int k);								// capture data의 k 번째 wingbeats의 param의 converting
	int				setCurrentFrame(const int f);
	void			setNewWbParam(const vectorN& wb);
	void			setNewWbParam(const WingbeatParam& wbParam);
	void			setNewWbParam(const float t);
	BodyState		getDesiredState(quater trunk, const BodyState& prevState);
	BodyState		modifyCurrentState(const BodyState& desState);
	BodyState		modifyDesiredState(const BodyState& desState);
	void			getDesiredTailState(float* ret);

	Controller();
	~Controller();

	void			saveState(const int sID);
	void			restoreState(const int sID);
	
	//
	ControlState	calcControlState(const BodyState& curState, const BodyState& nextState, const float frame);
	ControlState	calcControlState(const RigidState& curState, const RigidState& nextState, const float frame);

	// do something along control method
	void			do_None();

	// for draw something about controller
	void			draw();

	void			drawGraph();
	void			drawEachAxis(float x, float y, float frame);
	void			drawEachGraph(int idx, float y, vector3 color);
	void			extractEachGraph(int idx, float y, vector3 color);

	void			drawExample();


	// initialization
	void	reset();

	//////////////////////////////////////////////////////
	void	initializeState();

	void	setParameter(WingbeatParam* wp);
	void	setParameterGlide(WingbeatParam* wp, float leftWing, float rightWing, float tailWing);
	void	setNewWingbeat(float blendRatio=1.f/3.f);

	vectorN	getWbparam();
	void	setWbparam(vectorN v);
	vectorN convertWbParam(WingbeatParam* wp);

	void	setController();

	void	drawState(int drawNum=2);
	void	drawTemp();

	void	updateState_Wingbeat(WingbeatParam *wb, bool isFront=true);
	void	updateState_Wingbeat_Glide(float leftWing=0.f, float rightWing=0.f, float tailWing=0.f, bool isFront=true);
	void	updateState_Regression(void* ud);
	void	updateState_Regression_Glide(void* ud);
	void	updateState_Regression_ControlData(void* ud);

	void	copyWbParam(WingbeatParam *des, WingbeatParam *src);
	void	printWbParam(WingbeatParam*	wb, bool console, bool file);

	void	render();

	void	pushCtrPnt(std::vector<double> &ctrPnts, vector3 start, vector3 sOffset, float rad);
	void	pushCtrPntsTurn(std::vector<double> &ctrPnts, vector3 sOffset, vector3 mOffset, vector3 eOffset, float rad);
	void	pushCtrPntsStraight(std::vector<double> &ctrPnts, vector3 sOffset, vector3 eOffset, float rad);

	void	clearStateStack(std::vector< std::vector<float>* >&	stack, int saveNum=0);
};
