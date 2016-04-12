#pragma once

#include "Body.h"
#include "Feathers.h"

struct RenderState_bird 
{
	vector< RenderState_body >		renderState_body;
	vector< RenderState_feather >	RenderState_feather;
};

class Bird
{
public:

	Body*			birdBody;
	Feathers*		birdFeathers;

	RenderState_bird	renderState_bird;

	Bird()
	{
		initializeBird();
	};
	~Bird()
	{
		finalizeBird();
	}

	void			resetBird();

	// save full bird state to sID(state ID)
	void			saveState(int sID);
	// restore full bird state from sID(state ID)
	void			restoreState(int sID);
	void			saveStateToFile(const char* name);
	void			restoreStateFromFile(const char *name);

	void			saveOneRenderState();

	void			initializeBird();
	void			finalizeBird();

	void			stateUpdata_beforeSim();
	void			stateUpdata_afterSim();

	// T is transformation from global frame to desired frame
	void			putBirdAt(transf T, float v_linear_scale=1.0f, float v_angular_scale=1.0f);
	
	void			drawBird();
};