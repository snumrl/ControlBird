#include "SystemState.h"
#include "util.h"

SystemState::SystemState()
{
	MODE				= BASIC_SIMULATION;
	STATE				= STOP;

	recordOn			= false;
	trackOn				= true;

	drawOn				= true;
	drawResolution		= true;
	drawFPS				= true;
	drawBody			= BASIC;
	drawFeather			= DEPTH_TEST_OFF;
	drawAeroForce		= NET_FORCE;
	drawTrajectory		= FIXED;
	drawControlPath		= GP_LP;

	setSystemInformation();

	omp_set_num_threads(2);
};
void
SystemState::setSystemInformation()
{
	ifstream fin;
	util::fileOpenWrapper(fin, "Informations/System_inform.txt");

	FPS			= atoi(getNext(fin, "//").c_str());
	STEPS		= atoi(getNext(fin, "//").c_str());
	PLAY_SPEED	= STEPS / atoi(getNext(fin, "//").c_str());
	
	TIME_STEP	= 1.0f/(float)(FPS*STEPS);
	
	fin.close();
	fin.clear();
}