#ifndef	_FEATHER_SHELL_MESH_H_
#define	_FEATHER_SHELL_MESH_H_

#include <mathclass\mathclass.h>
#include <GTL/array.h>
#include "FeatherShellElement.h"

//
struct	FeatherState
{
	// Transform
	transf	Tglobal;				// rigid body transformation supplied by the user

	vector3	v_angular, v_linear;		// rigid body angular & linear velocities
	vector3	a_angular, a_linear;		// rigid body angular & linear accelerations

	// Twist
	m_real		twistAngle;			// twist angle
	m_real		twistVelocity;		// angular velocity of twist

	m_real		twistInertia;		// with respect to the fixed axis of rotation in the local coordinate system

	// Modal system
	matrixN		q, v, a;			// modal amplitudes and their velocities

	friend ostream& operator<<( ostream& os, FeatherState const& fs )
	{	
		os << fs.Tglobal;
		os << fs.v_angular;
		os << fs.v_linear;
		os << fs.a_angular;
		os << fs.a_linear;
		os << fs.twistAngle << fs.twistVelocity << fs.twistInertia;
		os << fs.q << " " << fs.v << " " << fs.a << " ";
		os << endl;
		
		return os;
	}
	friend istream& operator>>( istream& is, FeatherState& fs )
	{
		is >> fs.Tglobal;
		is >> fs.v_angular;
		is >> fs.v_linear;
		is >> fs.a_angular;
		is >> fs.a_linear;
		is >> fs.twistAngle >> fs.twistVelocity >> fs.twistInertia;
		is >> fs.q >> fs.v >> fs.a;
		
		return is;
	}
};

// FemShellMesh
//
class	FeatherShellMesh
{
public:
	bool	diagnosis;

public:
	array<vector3> dragForce_aero;		// dragForce due to aerodynamics
	array<vector3> liftForce_aero;		// liftForce due to aerodynamics
	array<vector3> netForce_aero;		// netForce due to aerodynamics
	m_real		areaRatio;

	// Rigid transform where the deformable feather attached
public:
	transf	Ttotal;						// twisting transformation * rigid body transformation
	transf	Tinverse;
	transf	Ttwist;						// twisting transformation
	transf	Tglobal;					// rigid body transformation supplied by the user

	vector3	v_angular, v_linear;		// rigid body angular & linear velocities
	vector3	a_angular, a_linear;		// rigid body angular & linear accelerations

	bool	velocityDueToTwist;			// Include velocity change due to twist?
	bool	veloictyDueToDefomration;	// Include velocity change due to deformation?

	// Twist
public:
	position	pinnedPosition;			// position of the twisting joint in the local coordinate system
	vector3		twistAxis;				// axis of rotation in the local coordinate system

	m_real		twistAngle;				// twist angle
	m_real		twistVelocity;			// angular velocity of twist

	m_real		twistInertia;			// with respect to the fixed axis of rotation in the local coordinate system

	// Twist control variable
public:
	bool	twistEnalbed;
	bool	twsitQuasiDynamicSimulation;
	bool	twistImplicitScheme;

	m_real	twistStiffness;
	m_real	twistDampingFactor;

	bool	twistRateLimiting;
	m_real	twistRateMax;				// Maximum revolutions per second

	bool	twistLimitingUpperBound;	// Unidirectional
	m_real	twistUpperBoundValue;

	bool	twistLimitingLowerBound;
	m_real	twistLowerBoundValue;

public:
	void	setPinnedPosition(const position& p)	{ pinnedPosition = p * scaleFactor; }
	void	setTwistAxis(const vector3& v)			{ twistAxis = normalize(v); }

	transf	twistTransf();

	m_real	getTwistInertia()	{ return twistInertia; }	//	return T.affine().transpose() * (inertia_init) * T.affine();
	void	evaluateTwistInertia();

	// Center of mass
public:
//	position	com;					// in the local coordinate system

public:
//	position	getCOM()		{ return com * Ttotal; }	// in the global coordinate system
//	void		evaluateCOM()	{ com = evaluateCOM(p) * Tinverse; }	// in the local cooridnate system
//	position	evaluateCOM(array<position>& p);

	// Rigid body
public:
	m_real	total_volume;		// volume of the element
	m_real	total_mass;			// total mass

public:
	m_real	mass() const			{ return total_mass; }
	m_real	volume() const;
	m_real	static_volume() const	{ return total_volume; }


	// Nodes
public:
	int	num_nodes;			// number of nodes
	int	num_unodes;			// number of unconstrained nodes

	array<position>	x;				// material coordinates: position in the local cooridnate systemat the initial time
	array<vector3>	u;				// displacement in the local coordinate system
	array<position>	p;				// position in the global coordinate system (i.e. p = (x + u) * T)

	array<matrix>	R;				// Rotation tensor field
	array<vector3>	r;				// Rotation vector3 field

	array<bool>		consts;			// Is the node is statically constrained?

public:
	void		setNumNodes(int n);
	inline int	numNodes() const			{ return num_nodes; }
	inline int	numUnodes() const			{ return num_unodes; }

	void			setMaterialCoord(int i, const position& p);
	inline position	materialCoord(int i) const	{ return x[i]; }
	
	inline position	pos(int i) const		{ return (1.0 / scaleFactor) * p[i]; }
	inline position	pos(position& p) const	{ return (1.0 / scaleFactor) * p; }

	vector3	getRotationAxis(int i_node, int j_axis);	// Rotation field


	// External force
public:
	bool	flagExternalForce;		// flag for using external force
	array<vector3>	eforce;			// external force
	array<bool>		einteraction;
	m_real	etorque_twist;			// external force

public:
	void	useExternalForce(bool flag)					{ flagExternalForce = flag; }
	bool	isExternalForce() const						{ return flagExternalForce; }
	bool	isExternalInteraction(int i) const			{ return einteraction[i]; }
	void	clearExternalForce();
	void	addExternalForce(int i, const vector3& v)	{ eforce[i] += v; einteraction[i] = true; }
	void	setExternalForce(int i, const vector3& v)	{ eforce[i] = v; einteraction[i] = true; }
	vector3	getExternalForce(int i)	const				{ return eforce[i]; }
	void	setExternalTorqueTwist(const m_real t)		{ etorque_twist = t; }


	// Statically-constrained nodes
public:
	array<int>	unodemap, iunodemap;// unconstrained and used node map

public:
	void	beginStaticConstSetup();
	void	setStaticPositionConst(int nid);
	void	endStaticConstSetup();

	
	// Volume elements
public:
	array<FeatherShellElement*>	elements;
	
public:
	void		setNumElements(int n);
	inline int	numElements() const					{ return elements.length(); }

	void		setNodeId(int eid, int i0, int i1, int i2)	{ elements[eid]->setNodeId(i0, i1, i2); }
	inline int	nodeId(int eid, int index) const	{ return elements[eid]->nodeId(index); }
	
	// Surface elements (= Volume elements)
	position	COM(int i) const		{ return elements[i]->COM(); }
	vector3		areaNormal(int i) const	{ return elements[i]->areaNormal(); }
	m_real		areaInitial(int i) const	{ return elements[i]->static_area(); }


	// Edges
public:
	array<FeatherShellEdge>	edges;

public:
	void		setNumEdges(int e)	{ edges.setSize(e); }
	inline int	numEdges() const	{ return edges.length(); }

	void		setEdgeNodeId(int eid, int i0, int i1, int i2, int i3)	{ edges[eid].setNodeId(i0, i1, i2, i3); }
	inline int	nodeIdOfEdge(int eid, int index) const { return edges[eid].nodeId(index); }


	// Material constants
public:
	m_real	rho;			// material density
	m_real	height;			// average height of the shell

	m_real	mu1;			// bending stiffness
	m_real	mu2;
	m_real	lambda;			// stretching stiffness
	m_real	damping;		// damping coefficients
	
public:
	void	setMaterialConstants(m_real, m_real, m_real, m_real, m_real);	// Set material constants
	
public:
	// System matrix
	smatrixN	SM, USM;		// Sparse mass matrix and unconstrained one
	smatrixN	SK, USK;		// Sparse stiffness matrix and unconstrained one
	smatrixN	SC, USC;		// Sparse damping matrix and unconstrained one

	vectorN		UFext;			// External force for unconstrained ndoes

public:
	// System matrix
	bool	assembleElementMatrices();
	bool	reduceSystemMatrices();


	// Simulation
public:
	int		currFrame;

	int		startFrame;			// simulation start frame
	m_real	fps;				// simulation speed

	m_real	fidelity;			// acceleration fieldity
	m_real	scaleFactor;		// meter conversion

	int		integrationScheme;	// Integration scheme
	m_real	integrationTheta;

	bool	useGravity;			// gravity field
	vector3	gravity;			// graivity constant

	bool	useViscosity;

public:
	bool	isWarping()			{ return warping; }
	void	setWarping(bool w)	{ warping = w; }

	// Simulation
	void	prepareMaterialBinding();
	bool	precompute();

	bool	initialize();

	void	simulateMain(int, const transf& Q, const vector3& lv, const vector3& av, const vector3& la, const vector3& aa);
	void	simulate(m_real h);

	// Simulation state control
	void	getState(FeatherState& state) const;
	void	setState(const FeatherState& state);
	
	
public:
	// Modal warping
	bool	flagPreloaded;		// Modal anaysis result is loaded from the outside
	bool	flagModalAnalyzed;

	bool	warping;			// modal warping

	int		num_modes;			// number of dominant frequency mode shapes

	bool	coriolis;			// add corriolis' force

	matrixN		H1, H1R;		// gravity transfer matrix
	matrixN		H2, H2R;		// rigid motion transfer matrix
				
	vectorN		F, Q;			// total modal force
	vectorN		Fg, Fr;			// forces due to gravity and rigid motion
	
	matrixN		q, v, a;		// modal amplitudes and their velocities
	
	m_real		dampM, dampK;	// Rayleigh damping

	matrixN		Phi, PhiT;		// mode shape and transpose of the mode shapes
	matrixN		PhiTM;			// inverse of the mode shapes

	smatrixN	W;
	matrixN		Psi;			// rotation transfer matrix

	vectorN		modalMass;		// modal mass
	vectorN		modalFreq;		// natural frequency of vibration
	
	vectorN		du;				// displacement of the unconstrained nodes

	// Strain limiting
	bool	strainLimit;		// enforce strain limit?
	m_real	maxStrainEnergy;	// maximum strain energy for strain limit

public:	
	// Modal system
	bool	precomputeModalAnalysis();
	
	void	precomputeGravityTransferMatrix();
	void	precomputeRigidMotionTransferMatrix();
	void	precomputeRotationTransferMatrix();

	vector3	integrateUsingRodriguezFormula(const vector3& w, const vector3& x);

	void	computeForcingTerm()		{ (warping) ? computeWarpedForcingTerm() : computeLinearForcingTerm(); }
	void	computeLinearForcingTerm();
	void	computeWarpedForcingTerm();

	void	computeDisplacements()		{ (warping)	? computeWarpedDisplacements() : computeLinearDisplacements(); }
	void	computeLinearDisplacements();
	void	computeWarpedDisplacements();

	void	updatePositions();	// using displacements and current twist and rigid transform

	void	addGeneralizedExternalForce(vectorN&);
	void	addGeneralizedCoriolisForce(vectorN&);

	vector3	getLinearVelocityOfNode(int time_step, int i);	// velocity of the i-th node
	vector3	getLinearVelocityOfCOM(int i);	// velocity of the i-th node
	vector3	getAngularVelocity(int time_step, int i);	// velocity of the i-th node

	// Lifting force for test
	void	addAerodynamicForces(m_real& twistTorque);

	// Integration
	void	integrate(m_real h);
	void	integrate_IE(m_real h);			// Implicit Euler
	void	integrate_ModeShape(m_real h);	// Mode Shape
	
	void	getDamping(m_real&, m_real&);

public:
	FeatherShellMesh();
	~FeatherShellMesh();

	// Bodies & elements
	FeatherShellElement*	newElement();
	FeatherShellElement&	element(int i)	{ return *((FeatherShellElement*)elements[i]);}
	FeatherShellEdge&		edge(int i)		{ return edges[i]; }

public:
	// Flags
	bool	flagPrecomputed;
	bool	flagProfiling;			// ODE and Constraint profiling
	bool	flagFailure;			// the method fails in somewhere during setup

//	bool	diagnostic;				// diagnostic for calling sequence

public:
	bool	isProfiling()				{ return flagProfiling; }
	void	setProfiling(bool d = true)	{ flagProfiling = d; }
	bool	wasFailed()					{ return flagFailure; }
	void	setFailure(bool failure = true)				{ flagFailure = failure; }

	// Scale conversion
	void	downscaling(transf& t) const	{ t.setTranslation(t.translation() * scaleFactor); }	// External to internal
	void	upscaling(transf& t) const		{ t.setTranslation(t.translation() / scaleFactor); }	// Internal to external

	// visualization
	inline position	world(const position& p) const { return (1.0 / scaleFactor) * p; }
	
	// file I/O
	bool	readMeshFepp(const char* filename);
	
	bool	isReady()		{ return x.length() && elements.length(); }
};


#endif	// _FEATHER_SHELL_MESH_H_
