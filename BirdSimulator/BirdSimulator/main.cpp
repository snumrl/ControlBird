#include "CaptureMotion.h"
#include "Bird.h"
#include "Controller.h"
#include "SystemState.h"
#include "Coeff.h"
#include "util.h"
#include "GUI.h"
#include "MeshModel.h"
#include "RenderBird.h"


using namespace std;



PhyEngine*		phyEngine;
CaptureMotion*	captureBird;
Bird*			simBird;
SystemState*	sysState;
Controller*		controller;
Coeff*			coeff;
MeshModel*		meshModel;
RenderBird*		rendBird;

GUI*			guiWindow;

string			date;
GLUquadricObj*	qObj;

int
main(int argc, char** argv)
{
	cout << "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-" << endl;
	cout << "--------------- hello bird -------------------" << endl;
	cout << "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-" << endl;
	
	date=util::getLocalTimeString();

	cout << endl;
	cout << "-------------------------------------------" << endl;
	cout << "date : " << date << endl;
	cout << "-------------------------------------------" << endl;
	cout << "usage : q(play), w(pause), e(reset)" << endl;
	cout << "-------------------------------------------" << endl;
	cout << endl;

	//meshModel			= new MeshModel("songbird2",0.5f);
	coeff				= new Coeff(
		string(INFORM_DIR)+"coeff/simul.txt", 
		string(INFORM_DIR)+"coeff/feather.txt", 
		string(INFORM_DIR)+"coeff/featherAngle.txt");
	sysState			= new SystemState();
	captureBird			= new CaptureMotion();
	controller			= new Controller();
	phyEngine			= new PhyEngine();
	simBird				= new Bird();
	rendBird			= new RenderBird();

	guiWindow		= new GUI(argc, argv);

	cout << "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-" << endl;
	cout << "--------------- good bye bird ------------------" << endl;
	cout << "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-" << endl;

	delete coeff;
	delete sysState;
	delete captureBird;
	delete controller;
	delete phyEngine;
	delete simBird;
	delete guiWindow;

	return 1;
}

// featherDemo.cpp : main project file.

#include <fstream>
#include <FemFeather/FeatherConstants.h>
#include <FemFeather/FeatherShellMesh.h>

using namespace std;

int maaain()
{
	char	tokenName[256];

	ifstream	is("feather.txt");

	FeatherShellMesh*	femMesh = new FeatherShellMesh;
	femMesh->setProfiling(true);

	// Scale factor
	femMesh->scaleFactor = 0.01;	// centimeter

	// Twist position
	position	pinnedPosition;
	is >> tokenName >> pinnedPosition;
	cerr << "tokenName = " << tokenName << endl;

	femMesh->setPinnedPosition(pinnedPosition);

	// Twist axis
	vector3	twistAxis;
	is >> tokenName >> twistAxis;
	cerr << "tokenName = " << tokenName << endl;

	femMesh->setTwistAxis(twistAxis);


	// Simulation mesh
	//

	// Nodes
	int	num_nodes;
	is >> tokenName >> num_nodes;
	cerr << "tokenName = " << tokenName << endl;

	femMesh->setNumNodes(num_nodes);

	position	p;
	for (int i = 0; i < num_nodes; i++)
	{
		is >> p;
		femMesh->setMaterialCoord(i, p);
	}

	// Elements: triangle elements in this case
	int	num_elements;
	is >> tokenName >> num_elements;
	cerr << "tokenName = " << tokenName << endl;

	femMesh->setNumElements(num_elements);

	int	vtx1, vtx2, vtx3, vtx4;
	for (int i = 0; i < num_elements; i++)
	{
		is >> vtx1 >> vtx2 >> vtx3;

		// (vtx1, vtx2, vtx3) is a triangle element.
		femMesh->element(i).setNodeId(vtx1, vtx2, vtx3);
	}

	// Edges
	int	num_edges;
	is >> tokenName >> num_edges;
	cerr << "tokenName = " << tokenName << endl;

	femMesh->setNumEdges(num_edges);

	for (int i = 0; i < num_edges; i++)
	{
		is >> vtx1 >> vtx2 >> vtx3 >> vtx4;

		// vtx3 and vtx4 are vertices of the faces sharing the edge (vtx1, vtx2).
		// "-1" implies that there is not such a vertex.
		femMesh->edges[i].setNodeId(vtx1, vtx2, vtx3, vtx4);
	}

	// Material binding
	femMesh->prepareMaterialBinding();


	// Static constrained nodes
	femMesh->beginStaticConstSetup();
		femMesh->setStaticPositionConst(0);		// At least 3 vertices should be constrained.
		femMesh->setStaticPositionConst(6);		// In this demo, we constrain 3 vertices,
		femMesh->setStaticPositionConst(11);	// nearest to the twist joint.
	femMesh->endStaticConstSetup();


	// FPS
	femMesh->fps = 2400.0;		// 2400Hz
	femMesh->startFrame = 0;

	// femMesh->diagnosis = true;

	// Material properties
	//
	femMesh->rho = 25.0;		// density
	femMesh->height = 0.001;	// 1mm
	femMesh->mu1 = 0.001;		// bending stiffness
	femMesh->mu2 = 0.05;		// bending stiffness
	femMesh->lambda = 2500.0;	// stretching stiffness
	femMesh->num_modes = 4;		// 4 is enough for bending motion

	femMesh->damping = 0.001;	// damping for thin shell

	femMesh->twistStiffness = 6.0e-5;
	femMesh->twistDampingFactor = 0.13;	// damping for twist joint

//	femMesh->integrationScheme = FEM_MA_SHAPE;	// To show mode shape....
//	femMesh->twistEnalbed = false;;
//	femMesh->damping = 0.5;


	// Simulation loop
	//
	for (int currFrame = 0; currFrame < 1500; currFrame++)
	{
		int	frame;
		transf	T;		// Relative transform with respect to the inital. i.e T0.inverse() * Tcurrent
		vector3	lv, av;	// Linear and angular velocity
		vector3	la, aa;	// Linear and angular acceleration

		is >> frame >> T >> lv >> av >> la >> aa;	// velocityAndAcceleration

		femMesh->simulateMain(frame, T, lv, av, la, aa);
		if (femMesh->wasFailed())
		{
			cerr << "Simulation failed!" << endl;
			break;
		}

	//	femMesh->p[i];	// the position of the i-th node in the global coordinates
	//	femMesh->addExternalForce(i, f_i);	// f_i is applied to the i-th node
	}

	delete	femMesh;

	cin.get();

    return 0;
}
