#ifndef	_FEATHER_CONSTANTS_H_
#define	_FEATHER_CONSTANTS_H_

#include <mathclass/mathclass.h>

#define	FEM_SCALE_FACTOR	0.01

#define	FEM_BONE_RADIUS		0.015

#define	FEM_FIDELITY			 1.0
#define	FEM_NEGATIVE_FIDELITY	-1.0

#define	FEM_MIN_TIME_STEP_LEVEL	0
#define	FEM_MAX_TIME_STEP_LEVEL	3

#define	FEM_NUM_TIME_STEP	0
#define	NumTimeStep(numTimeStep)	((int)pow(2.0, (numTimeStep)))

//#define	FEM_NUM_TIME_STEP	1
//#define	NumTimeStep(numTimeStep)	max((numTimeStep), 1)

#define	FEM_MAX_NUM_SPLIT_NODES	1000

// material density
#define	FEM_RHO				1.5

// lame constant for stress-strain tensor relation
#define	FEM_MU				0.10
#define	FEM_LAMBDA			0.25

// constant for stress-strain rate tensor relation
#define	FEM_DAMPING			0.01
#define	FEM_PSI				(FEM_MU * FEM_DAMPING)
#define	FEM_PHI				(FEM_LAMBDA * FEM_DAMPING)

// constants for plastic flow control
#define	FEM_GAMMA1				0.3
#define	FEM_GAMMA2				1.0
#define	FEM_CREEP				1.0

// constants for fracture
#define	FEM_TAU					250.0

#define	FEM_THICKNESS				0.001		// 1mm
#define	FEM_ROUNDING_ERROR_RATIO	0.001		// 0.001mm
#define	FEM_K_REPULSION_SPRING		1.0E8

// strain limting
#define	FEM_SL_COMP_RATIO		-0.5	// Compression ratio
#define	FEM_SL_STRETCH_RATIO	0.5		// Stretch ratio
#define	FEM_SL_FRACTION			1.01

// fem class identification
#define	FEM_MESH				0x0000
#define	FEM_LINEAR_MESH			0x1000
#define	FEM_MODAL_MESH			0x1100
#define	FEM_SHELL_MESH			0x1110
#define	FEM_COROTATIONAL_MESH	0x1200

#define	FEM_LUMPED_MESH			0x2000
#define	FEM_DSHELL_MESH			0x4000

// fem methods
#define	FEM_METHOD_MODAL		1
#define	FEM_METHOD_LUMPED		2
#define	FEM_METHOD_COROTATIONAL	4

// integration scheme
#define	FEM_INTEGRATION_DEFAULT	1
#define	FEM_INTEGRATION_EULER	1
#define	FEM_INTEGRATION_RK2		2
#define	FEM_INTEGRATION_RK4		3
#define	FEM_INTEGRATION_RKQS	4

// integration scheme for Modal Synthesis
#define	FEM_MA_IIR				1
#define	FEM_MA_IE				2
#define	FEM_MA_EE				3
#define	FEM_MA_SHAPE			4
#define	FEM_MA_USER				5
#define	FEM_MA_NODE_GEN			6

// integration scheme for Corotational Method
#define	FEM_CR_EE				1
#define	FEM_CR_IE				2

// num of maximum modes
#define	FEM_MA_MAX_NUM_MODES	8

// rotation field scheme
#define	FEM_RF_AV				1	// Adjacent vertices
#define	FEM_RF_SVD				2	// Singular Value Decomposition with Rotors
#define	FEM_RF_POLAR			3	// Polar decomposition
#define	FEM_RF_RT				4	// Rotation tensor

// excitation scheme
#define	FEM_EXCITATION_REST		1
#define	FEM_EXCITATION_DEFORMED	2

// filtering scheme
#define	FEM_FILTERING_NONE		0
#define	FEM_FILTERING_FP		1
#define	FEM_FILTERING_FP_FV		2

// MAX Strain
#define	FEM_MAX_STRAIN			5.0


// Well-known Properties
#define	FEM_GRAVITY			vector3(0, -9.8, 0)

#define	FEM_STEEL_DENSITY	7480.0
#define	FEM_STEEL_YOUNG		200.0	// GPa
#define	FEM_STEEL_POISSON	0.3

#define	FEM_RUBBER_DENSITY	1.100E3
#define	FEM_RUBBER_YOUNG	0.021
#define	FEM_RUBBER_POISSON	0.5


#endif	// _FEATHER_CONSTANTS_H_
