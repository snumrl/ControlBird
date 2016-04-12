#pragma once

#include <string>
#include <vector>
#include <MATHCLASS/mathclass.h>

using namespace std;

struct CaptureWingbeat
{
	std::vector<float>*		pos;				// pos[dof][frame]
	std::vector<float>*		normalizedPos;		// pos[dof][avgFrame]
	std::vector<float>*		reconstructPos;		// pos[dof][avgFrame]

	int						duration;

	int						k_clip;
	int						k_frame;			// 현재 wingbeat은 k_clip번째 클립의 k_frame번째 frame에서 시작

	float*					scale;
	float*					translation;
	float					timewarp;

	float*					minPoint;
	float*					maxPoint;				

	CaptureWingbeat(int dof)
	{
		pos				= new std::vector<float>[dof];
		normalizedPos	= new std::vector<float>[dof];
		reconstructPos	= new std::vector<float>[dof];
		scale			= new float[dof];
		translation		= new float[dof];
		minPoint		= new float[dof];
		maxPoint		= new float[dof];
	}
	~CaptureWingbeat()
	{
		delete[] pos;
		delete[] normalizedPos;
		delete[] reconstructPos;
		delete[] scale;
		delete[] translation;
		delete[] minPoint;
		delete[] maxPoint;
	}
};

struct OneClip
{
	int			numOfFrames;
	
	vector3**		pos;						// pos[frame][marker]

	vector3**		root;						// root[frame][axis]
	quater*			rotQ;						// rotQ[frame]
	float**			DOF;						// DOF[frame][dof]

	std::vector<int>	delimFrame;

	OneClip(int marker, int dof, int frames)
	{
		numOfFrames = frames;

		pos			= new vector3*[frames];
		root		= new vector3*[frames];
		DOF			= new float*[frames];

		rotQ	= new quater[frames];

		for(int i=0;i<frames;i++)
		{
			pos[i]		= new vector3[marker];
			root[i]		= new vector3[3];
			DOF[i]		= new float[dof];

			for(int j=0;j<marker;j++)	pos[i][j]		= vector3();
			for(int j=0;j<3;j++)		pos[i][j]		= vector3();
			for(int j=0;j<dof;j++)		DOF[i][j]		= 0.0f;
		}
	}
};

class CaptureMotion
{
public:
	enum	BirdName				{PIGEON, ARONG, maxBirdName	};
	enum	MarkerName				{m_head, m_cback, m_spine, m_lshoulder, m_lelbow, m_lwrist, m_lhand, 
									 m_rshoulder, m_relbow, m_rwrist, m_rhand, m_ctail, m_ltail, m_rtail, maxMarkerName	};
	enum	DOFname					{d_lshoulder1, d_lshoulder2, d_lshoulder3, d_rshoulder1, d_rshoulder2, d_rshoulder3,
									 d_lelbow_bend, d_lelbow_twist, d_relbow_bend, d_relbow_twist, d_lwrist_bend, d_rwrist_bend, 
									 d_tail_bend, d_tail_spread, maxDOFname	};

	BirdName						whatBird;

	std::string						str_birdName;
	int								numOfClips;
	int								captureFPS;

	std::vector<OneClip*>			clips;
	std::vector<CaptureWingbeat*>	wingbeats;
	
	float							avgWingbeatDuration;
	CaptureWingbeat*				refWingbeat;

	int								currentClip;
	int								currentFrame;
	
	CaptureMotion()
	{
		currentClip = 0;
		currentFrame = 0;

		setCaptureDataInformation();
		
		loadCaptureData();

		getWingbeat();
		reflectWingbeat();
		normalizeWingbeat();		
		getReferenceWingbeat();		
		parameterizeWingbeat();		
		reconstructWingbeat();		

		if(clips.size() == 0)	currentClip = -1;
	};

	void	setCaptureDataInformation();
	void	loadCaptureData();
	void	dataConverting(OneClip* one);

	void	getRoot(OneClip* one);
	void	getShoulder(OneClip* one);
	void	getElbow(OneClip* one);
	void	getWrist(OneClip* one);
	void	getTail(OneClip* one);
	void	getDelimiter(OneClip* one);

	void	getWingbeat();
	void	reflectWingbeat();
	void	normalizeWingbeat();
	void	getReferenceWingbeat();
	void	parameterizeWingbeat();
	void	reconstructWingbeat();
	
	vector3	getCurrentMarker(MarkerName nn);
	void	drawTitle(float x, float y, char* title);
	void	drawEachAxis(float x, float y);
	void	drawEachGraph(int idx, float x, float y, vector3 color);
	void	drawAllGraph();
	void	drawRootAxis();
	void	drawCapturedBird();
	void	drawReconsBird();
};