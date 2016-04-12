#include "Bird.h"
#include "Controller.h"
#include "SystemState.h"
#include "GUI.h"

extern PhyEngine*		phyEngine;
extern SystemState*		sysState;
extern Controller*		controller;

void
Bird::resetBird()
{
	phyEngine->finalize();		
	finalizeBird();

	phyEngine->initialize();
	initializeBird();
}
void
Bird::saveState(const int sID)
{
	if ( (sID < 0) || (sID > max_save) )
	{
		cerr << "Error : id should be <0," << max_save << ">" << endl;
		cin.get(); cin.get();
	}

	birdBody->saveState(sID);
	birdFeathers->saveState(sID);
	controller->saveState(sID);
}
void
Bird::restoreState(const int sID)
{
	if ( (sID < 0) || (sID > max_save) )
	{
		cerr << "Error : id should be <0," << max_save << "9>" << endl;
		cin.get(); cin.get();
	}

	birdBody->restoreState(sID);
	birdFeathers->restoreState(sID);
	controller->restoreState(sID);
}
void
Bird::saveStateToFile(const char* name)
{
	ofstream fout;

	if(util::fileOpenWrapper(fout, name))
	{
		//birdBody->saveStateToFile(fout);
		//birdFeathers->saveStateToFile(fout);

		util::fileCloseWrapper(fout);
	}
}
void
Bird::restoreStateFromFile(const char* name)
{
	ifstream fin;

	if(util::fileOpenWrapper(fin, name))
	{
		//birdBody->restoreStateFromFile(fin);
		//birdBody->restoreStateFromFile(fin);
		
		util::fileCloseWrapper(fin);
	}
}
void
Bird::saveOneRenderState()
{
	renderState_bird.renderState_body.push_back( birdBody->getRenderState() );
	renderState_bird.RenderState_feather.push_back( birdFeathers->getRenderState() );
}
void
Bird::initializeBird()
{
	birdBody		= new Body();
	birdFeathers	= new Feathers((Body*)birdBody, Feathers::ThinShell);
}
void
Bird::finalizeBird()
{
	delete birdBody;
	delete birdFeathers;
}
void
Bird::stateUpdata_beforeSim()
{
	birdBody->incrementCount();				

	// here, set new wingbeat

	birdBody->updateDesiredState();
	birdBody->calcPDcontrolTorque();		
	birdBody->applyPDcontrolTorque();		

	birdFeathers->calcAeroForce();
	birdFeathers->applyAeroForce();		
}
void
Bird::stateUpdata_afterSim()
{
	birdBody->updateCurrentState();
	birdFeathers->updateState();
}
// all argrument are global
void
Bird::putBirdAt(transf T, float v_linear_scale, float v_angular_scale)
{
	transf trunkT = birdBody->getLinkTransformation( Body::Trunk );

	birdBody->putBirdAt( trunkT.inverse() * T, v_linear_scale, v_angular_scale );
	//birdFeathers->putBirdAt( trunkT.inverse() * T );
}
void
Bird::drawBird()
{
	switch(sysState->drawFeather)
	{
	case sysState->DEPTH_TEST_ON:
		birdFeathers->drawFeathers(true); 
		break;
	case sysState->DEPTH_TEST_OFF:
		birdFeathers->drawFeathers(false); 
		break;
	case sysState->NONE_F:
		break;
	}

	switch(sysState->drawBody)
	{
	case sysState->NONE_B:	
		break;
	default: 
		birdBody->drawBody(); 
		break;
	}
}