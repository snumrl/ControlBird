#pragma once

#define OLD
//#define ROOT_FIX
//#define MOUSE_CTRL

static const int max_save = 10;

struct SystemState
{
	enum	SystemMode			{	BASIC_SIMULATION=0, CAPTURE_PLAY, RENDER_PLAY, MULTI_RENDER_PLAY	};
	enum	PlayState			{	PLAY=0, STOP, PAUSE, REVERSE_PLAY, PLAY_ONESTEP, REVERSE_PLAY_ONESTEP	};
	
	enum	BodyDrawMode		{	BASIC=0, MESH, NONE_B  };
	enum	FeatherDrawMode		{	DEPTH_TEST_ON=0, DEPTH_TEST_OFF, NONE_F  };
	enum	AeroDrawMode		{	NET_FORCE=0, LIFT_FORCE, DRAG_FORCE, NONE_A  };
	enum	TrajDrawMode		{	FIXED=0, VANISNING, NONE_T  };
	enum	ControlPathDrawMode	{	GP_LP=0, GP, NONE_C  };
	
	SystemMode				MODE;
	int						FPS;
	int						STEPS;
	double					TIME_STEP;

	PlayState				STATE;
	int						PLAY_SPEED;
	int						PLAY_SPEED_SAVE;

	int						simul_interval;				// (FPS*STEPS / capture_bird's FPS)

	bool					recordOn;
	bool					trackOn;

	// draw mode
	bool					drawOn;
	bool					drawResolution;
	bool					drawFPS;
	BodyDrawMode			drawBody;				
	FeatherDrawMode			drawFeather;
	AeroDrawMode			drawAeroForce;
	TrajDrawMode			drawTrajectory;
	ControlPathDrawMode		drawControlPath;

	SystemState();

	void		setSystemInformation();
	bool		get(PlayState ps)				{	if(STATE == ps)		return true;	else	return false;	}
	bool		get(SystemMode sm)				{	if(MODE == sm)		return true;	else	return false;	}
	
	void		set(PlayState ss)				{	STATE = ss;				}
	void		set(SystemMode ss)				{	MODE = ss;				}
	void		set(BodyDrawMode ss)			{	drawBody = ss;			}
	void		set(FeatherDrawMode ss)			{	drawFeather = ss;		}
	void		set(AeroDrawMode ss)			{	drawAeroForce = ss;		}
	void		set(TrajDrawMode ss)			{	drawTrajectory = ss;	}
	void		set(ControlPathDrawMode ss)		{	drawControlPath = ss;	}

	void		full_speed_play(int i)			
	{	
		if(i==0)
		{ 
			PLAY_SPEED = PLAY_SPEED_SAVE;
		}
		else
		{
			PLAY_SPEED_SAVE = PLAY_SPEED;
			PLAY_SPEED = 1000;
		}
	}
};